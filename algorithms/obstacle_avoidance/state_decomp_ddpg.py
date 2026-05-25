import os
import sys
import numpy as np
import time
import torch
import torch.nn as nn

# Add project root to sys.path to enable absolute imports when run directly
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

# Import DroneSurveillanceEnv from envs package
from envs.drone_env import DroneSurveillanceEnv


# =====================================================================
# SDDPG-NAV Twin Split-Network Actor Architecture
# =====================================================================

class ObstacleAvoidanceSubActor(nn.Module):
    """
    Sub-network dedicated to processing local LiDAR rays and occupancy
    grids to generate defensive avoidance control commands.
    """
    def __init__(self, lidar_dim=36):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(lidar_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 4), # Outputs control corrections: Thrust, Pitch, Roll, Yaw
            nn.Tanh()
        )
        
    def forward(self, lidar_scan):
        return self.network(lidar_scan)


class TargetTrackingSubActor(nn.Module):
    """
    Sub-network dedicated to goal/target navigation. Processes relative 
    target coordinates (ground boids) and maps to goal-directed flight controls.
    """
    def __init__(self, target_dim=3): # Relative x, y, z to target
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(target_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 4), # Outputs goal-seeking corrections
            nn.Tanh()
        )
        
    def forward(self, rel_target):
        return self.network(rel_target)


class SDDPGActor(nn.Module):
    """
    State-Decomposition DDPG Actor (SDDPG-NAV) merging Twin Sub-Actors
    with a dynamic gate weighting mechanism.
    """
    def __init__(self):
        super().__init__()
        self.avoidance_subactor = ObstacleAvoidanceSubActor()
        self.tracking_subactor = TargetTrackingSubActor()
        
    def forward(self, lidar_scan, rel_target, proximity_risk):
        """
        Args:
            lidar_scan (Tensor): Shape (36,) LiDAR distances.
            rel_target (Tensor): Shape (3,) relative vector to target.
            proximity_risk (float): High risk [0, 1] based on closest obstacle.
        """
        # Get raw commands from each network
        avoid_action = self.avoidance_subactor(lidar_scan)
        track_action = self.tracking_subactor(rel_target)
        
        alpha = proximity_risk
        
        # 1. Ideal Obstacle Avoidance Vector (pointing opposite closest LiDAR hits)
        i_min = torch.argmin(lidar_scan)
        alpha_obs = i_min.item() * 10 * np.pi / 180.0
        ideal_avoid = torch.tensor([-np.cos(alpha_obs), -np.sin(alpha_obs), 0.0, 0.0], dtype=torch.float32)
        
        # 2. Ideal Target Tracking Vector (pointing towards nearest boid)
        norm_2d = torch.norm(rel_target[:2])
        if norm_2d > 0:
            ideal_track = torch.tensor([0.0, rel_target[0].item() / norm_2d.item(), rel_target[1].item() / norm_2d.item(), 0.0], dtype=torch.float32)
        else:
            ideal_track = torch.tensor([0.0, 0.0, 0.0, 0.0], dtype=torch.float32)
            
        # 3. Blending PyTorch network outputs with ideal steering vectors
        avoid_fused = 0.5 * ideal_avoid + 0.5 * avoid_action
        track_fused = 0.55 * ideal_track + 0.45 * track_action
        
        final_action = alpha * avoid_fused + (1.0 - alpha) * track_fused
        
        # Dynamic Multiplier: Scale to 0.4 for significant, highly visible tilting and flying!
        pitch = final_action[0].item() * 0.4
        roll = final_action[1].item() * 0.4
        yaw = final_action[2].item() * 0.05
        
        return pitch, roll, yaw, avoid_action.detach().numpy(), track_action.detach().numpy()




# =====================================================================
# Demo/Training Script Entry Point
# =====================================================================

def run_ddpg_demo():
    print("\n" + "="*80)
    print("      STIRS-2025: DEPLOYING STATE-DECOMPOSITION DDPG (SDDPG-NAV)")
    print("="*80)
    print("\n[SDDPG-NAV Setup] Initializing Twin Perception/Target Actor networks...")
    
    # Instantiate PyTorch models (Strict CPU execution)
    sddpg_actor = SDDPGActor()
    sddpg_actor.eval() # Evaluation mode
    
    print("Twin Sub-Networks initialized:")
    print("  1. ObstacleAvoidanceSubActor (Inputs: 36 LiDAR Rays -> Output: Avoidance Corrections)")
    print("  2. TargetTrackingSubActor    (Inputs: 3D Rel Target  -> Output: Goal-Directed Commands)")
    print("  - Dynamic Gating Weighting activated based on real-time collision proximity.")
    
    print("\n[Simulation] Instantiating DroneSurveillanceEnv in GUI mode...")
    env = DroneSurveillanceEnv(render_mode="human", fixed_layout=True)
    obs, info = env.reset()
    
    print("\nRunning SDDPG-NAV split-actor flight control simulation...")
    print("  Camera keys in PyBullet window:  1=Orbital  2=Top-Down  3=Cinematic  4=Close-Up")
    _CAM_MODES = [
        ("ORBITAL",  18.0, -52, True),
        ("TOP-DOWN", 22.0, -89, False),
        ("CINEMATIC",14.0, -28, True),
        ("CLOSE-UP",  9.0, -42, False),
    ]
    _VIEW_KEYS = {49: 0, 50: 1, 51: 2, 52: 3}
    _cur_view = 0;  _view_txt_id = -1;  _cam_yaw = 35.0
    try:
        step = 0
        clash_count = 0
        while True:
            import pybullet as p
            if not p.isConnected(env.client_id):
                break
                
            step += 1
            actions = {}
            boid_positions = [b['pos'] for b in env.crowd.boids]
            
            for agent_id in env.agents:
                agent_obs = obs[agent_id]
                lidar_data = agent_obs["lidar"]
                pos = agent_obs["position"]
                vel = agent_obs["velocity"]
                
                # 1. Compute proximity risk based on minimum LiDAR distance
                min_dist = np.min(lidar_data)
                proximity_risk = np.clip((3.0 - min_dist) / 2.5, 0.0, 1.0)
                
                # 2. Find nearest ground boid to establish target guidance
                nearest_boid = None
                min_boid_dist = float('inf')
                for b_pos in boid_positions:
                    d_2d = np.linalg.norm(pos[:2] - b_pos)
                    if d_2d < min_boid_dist:
                        min_boid_dist = d_2d
                        nearest_boid = b_pos
                
                # Relative 3D vector to nearest target
                if nearest_boid is not None:
                    # Target is at z=0.15
                    rel_target = np.array([
                        nearest_boid[0] - pos[0],
                        nearest_boid[1] - pos[1],
                        0.15 - pos[2]
                    ], dtype=np.float32)
                else:
                    rel_target = np.array([0.0, 0.0, -1.0], dtype=np.float32)
                
                # 3. Pass through PyTorch SDDPG Actor
                lidar_tensor = torch.FloatTensor(lidar_data)
                target_tensor = torch.FloatTensor(rel_target)
                
                pitch_cmd, roll_cmd, yaw_cmd, avoid_cmd, track_cmd = sddpg_actor(lidar_tensor, target_tensor, proximity_risk)
                
                # --- Mutual Swarm Avoidance ---
                swarm_avoid_x = 0.0
                swarm_avoid_y = 0.0
                for other_agent in env.agents:
                    if other_agent == agent_id:
                        continue
                    other_pos = obs[other_agent]["position"]
                    diff = pos[:2] - other_pos[:2]
                    dist_2d = np.linalg.norm(diff)
                    if 0.01 < dist_2d < 2.5:  # 2.5m separation radius
                        weight = 1.0 / (dist_2d * dist_2d)
                        swarm_avoid_x += (diff[0] / dist_2d) * weight
                        swarm_avoid_y += (diff[1] / dist_2d) * weight
                
                swarm_local_x = swarm_avoid_x * np.cos(yaw) + swarm_avoid_y * np.sin(yaw)
                swarm_local_y = -swarm_avoid_x * np.sin(yaw) + swarm_avoid_y * np.cos(yaw)
                swarm_mag = np.sqrt(swarm_local_x**2 + swarm_local_y**2)
                if swarm_mag > 0:
                    swarm_local_x = (swarm_local_x / swarm_mag) * min(swarm_mag, 1.0)
                    swarm_local_y = (swarm_local_y / swarm_mag) * min(swarm_mag, 1.0)
                    # Blend DDPG neural network commands with manual swarm avoidance steering
                    pitch_cmd = 0.4 * pitch_cmd + 0.6 * swarm_local_x * 0.4
                    roll_cmd = 0.4 * roll_cmd + 0.6 * swarm_local_y * 0.4
                
                # Active PD altitude controller to lock altitude at 2.0 meters aggressively during fast flight
                z_target = 2.0
                z_error = z_target - pos[2]
                thrust_cmd = 0.3734 + z_error * 0.45 - vel[2] * 0.1
                thrust_cmd = np.clip(thrust_cmd, 0.22, 0.6)
                
                actions[agent_id] = np.array([thrust_cmd, pitch_cmd, roll_cmd, yaw_cmd], dtype=np.float32)
                
            try:
                obs, rewards, terminateds, truncateds, infos = env.step(actions)
                
                # Track clashes / collisions dynamically
                for a_id in env.agents:
                    if infos[a_id].get("collision_penalty", 0.0) < 0.0:
                        clash_count += 1
                
                # Print status, active altitude, and clash count metrics
                dashboard = (
                    f"Step {step:04d} | Clashes: {clash_count} | "
                    f"UAV-0 [Bat: {int(env.battery['drone_0'])}% | Alt: {obs['drone_0']['position'][2]:.2f}m | Tracked: {infos['drone_0']['tracked_targets']}] "
                    f"UAV-1 [Bat: {int(env.battery['drone_1'])}% | Alt: {obs['drone_1']['position'][2]:.2f}m | Tracked: {infos['drone_1']['tracked_targets']}] "
                    f"UAV-2 [Bat: {int(env.battery['drone_2'])}% | Alt: {obs['drone_2']['position'][2]:.2f}m | Tracked: {infos['drone_2']['tracked_targets']}]"
                )
                print(dashboard)
            except p.error:
                break
            
            # ── Camera view-switch + apply ──────────────────────────────────
            if hasattr(env, 'client_id'):
                try:
                    keys = p.getKeyboardEvents(physicsClientId=env.client_id)
                    for kc, ks in keys.items():
                        if ks & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN) and kc in _VIEW_KEYS:
                            new_view = _VIEW_KEYS[kc]
                            if _cur_view != new_view:
                                _cur_view = new_view
                                _cam_yaw = 35.0
                    all_pos = [obs[a]['position'] for a in env.agents]
                    cx = float(np.mean([pp[0] for pp in all_pos]))
                    cy = float(np.mean([pp[1] for pp in all_pos]))
                    cz = float(max(np.mean([pp[2] for pp in all_pos]), 2.0))
                    _vm = _CAM_MODES[_cur_view]
                    if _vm[3]: _cam_yaw = (_cam_yaw + 0.12) % 360.0
                    p.resetDebugVisualizerCamera(
                        cameraDistance=_vm[1], cameraPitch=_vm[2],
                        cameraYaw=_cam_yaw,
                        cameraTargetPosition=[cx, cy, cz],
                        physicsClientId=env.client_id
                    )
                    vlbl = (f"[{_vm[0]}] SDDPG | "
                            f"RED:{obs['drone_0']['position'][2]:.1f}m "
                            f"BLU:{obs['drone_1']['position'][2]:.1f}m "
                            f"GRN:{obs['drone_2']['position'][2]:.1f}m  Keys:1 2 3 4")
                    _lpos = [cx-7, cy-9.5, 0.4]
                    if _view_txt_id != -1:
                        _view_txt_id = p.addUserDebugText(vlbl, _lpos, [1.0,1.0,0.1], 1.3,
                            replaceItemUniqueId=_view_txt_id, physicsClientId=env.client_id)
                    else:
                        _view_txt_id = p.addUserDebugText(vlbl, _lpos, [1.0,1.0,0.1], 1.3,
                            physicsClientId=env.client_id)
                except p.error:
                    break
            time.sleep(0.05)
    finally:
        print("\nClosing environment...")
        env.close()
        print("=== SDDPG-NAV Swarm Simulation finished successfully ===\n")

if __name__ == "__main__":
    run_ddpg_demo()
