import os
import sys
import numpy as np
import time
import math

# Add project root to sys.path to enable absolute imports when run directly
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

# Import DroneSurveillanceEnv from envs package
from envs.drone_env import DroneSurveillanceEnv


def run_distill_demo():
    print("\n" + "="*80)
    print("      STIRS-2025: ATTENTION-BASED POLICY DISTILLATION CONTROL LOOP")
    print("="*80)
    print("\n[Behavior Distillation Setup] Initializing real-time behavioral blending...")
    print("  - Defensive Clearance Policy: Pitch/Roll opposite of the closest obstacle")
    print("  - Target Tracking Policy   : Pitch/Roll aligned with nearest ground boid")
    print("  - Attention Weighting      : Dynamic blending based on active LiDAR proximity risk")
    
    print("\n[Simulation] Instantiating DroneSurveillanceEnv in GUI mode...")
    env = DroneSurveillanceEnv(render_mode="human", fixed_layout=True)
    obs, info = env.reset()
    
    print("\nRunning Policy Distillation simulation...")
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
        hover_thrust = 0.3734
        step = 0
        clash_count = 0
        while True:
            import pybullet as p
            if not p.isConnected(env.client_id):
                break
                
            step += 1
            actions = {}
            boid_positions = [b['pos'] for b in env.crowd.boids]
            
            try:
                for agent_id in env.agents:
                    agent_obs = obs[agent_id]
                    lidar_data = agent_obs["lidar"]
                    pos = agent_obs["position"]
                    vel = agent_obs["velocity"]
                    
                    # We need the current heading (yaw) of the drone
                    drone_id = env.drone_ids[agent_id]
                    _, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=env.client_id)
                    euler = p.getEulerFromQuaternion(orientation)
                    yaw = euler[2]
                    
                    # 1. DEFENSIVE CLEARANCE POLICY
                    # Find the angle of the closest obstacle from 36 horizontal LiDAR rays (10-deg spacing)
                    i_min = np.argmin(lidar_data)
                    min_dist = lidar_data[i_min]
                    
                    # Proximity risk scales from 0.0 to 1.0 when obstacles are within 3.0 meters
                    proximity_risk = np.clip((3.0 - min_dist) / 2.5, 0.0, 1.0)
                    
                    # Angle of closest obstacle in local frame
                    alpha_obs = i_min * 10 * math.pi / 180.0
                    
                    # Local avoidance vector: point directly opposite the obstacle
                    avoid_local_x = -math.cos(alpha_obs)
                    avoid_local_y = -math.sin(alpha_obs)
                    
                    # Avoidance actions: Pitch controls forward/backward (X), Roll controls left/right (Y)
                    pitch_avoid = avoid_local_x * 0.45
                    roll_avoid = avoid_local_y * 0.45
                    
                    # 2. TARGET TRACKING POLICY
                    # Locate closest ground target
                    nearest_boid = None
                    min_boid_dist = float('inf')
                    for b_pos in boid_positions:
                        d_2d = np.linalg.norm(pos[:2] - b_pos)
                        if d_2d < min_boid_dist:
                            min_boid_dist = d_2d
                            nearest_boid = b_pos
                    
                    if nearest_boid is not None:
                        # Global relative vector
                        dx = nearest_boid[0] - pos[0]
                        dy = nearest_boid[1] - pos[1]
                        
                        # Rotate into drone's local coordinate frame
                        track_local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
                        track_local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
                        
                        # Normalize local target vector
                        mag = math.sqrt(track_local_x**2 + track_local_y**2)
                        if mag > 0:
                            track_local_x /= mag
                            track_local_y /= mag
                        
                        pitch_track = track_local_x * 0.35
                        roll_track = track_local_y * 0.35
                    else:
                        pitch_track = 0.0
                        roll_track = 0.0
                    
                    # 2.5 MUTUAL SWARM AVOIDANCE (Avoid other drones)
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
                    
                    # Convert swarm avoidance vector to local coordinate frame
                    swarm_local_x = swarm_avoid_x * math.cos(yaw) + swarm_avoid_y * math.sin(yaw)
                    swarm_local_y = -swarm_avoid_x * math.sin(yaw) + swarm_avoid_y * math.cos(yaw)
                    
                    swarm_mag = math.sqrt(swarm_local_x**2 + swarm_local_y**2)
                    if swarm_mag > 0:
                        swarm_local_x = (swarm_local_x / swarm_mag) * min(swarm_mag, 1.0)
                        swarm_local_y = (swarm_local_y / swarm_mag) * min(swarm_mag, 1.0)
                        
                    # 3. ATTENTION FUSION / DISTILLATION WITH SWARM AVOIDANCE
                    # Blend the controllers based on proximity risk attention weight (alpha)
                    alpha = proximity_risk
                    pitch_blend = alpha * pitch_avoid + (1.0 - alpha) * pitch_track
                    roll_blend = alpha * roll_avoid + (1.0 - alpha) * roll_track
                    
                    # Apply swarm avoidance if active
                    if swarm_mag > 0:
                        pitch_blend = 0.4 * pitch_blend + 0.6 * swarm_local_x * 0.45
                        roll_blend = 0.4 * roll_blend + 0.6 * swarm_local_y * 0.45
                    
                    pitch_cmd = pitch_blend
                    roll_cmd = roll_blend
                    
                    # Vertical altitude control: target hover altitude of 2.0 meters
                    z_target = 2.0
                    z_error = z_target - pos[2]
                    thrust_cmd = hover_thrust + z_error * 0.45 - vel[2] * 0.1
                    thrust_cmd = np.clip(thrust_cmd, 0.2, 0.6)
                    
                    # Combine into full action command
                    actions[agent_id] = np.array([thrust_cmd, pitch_cmd, roll_cmd, 0.0], dtype=np.float32)
            except p.error:
                break
                
            try:
                obs, rewards, terminateds, truncateds, infos = env.step(actions)
                
                # Track clashes / collisions dynamically
                for a_id in env.agents:
                    if infos[a_id].get("collision_penalty", 0.0) < 0.0:
                        clash_count += 1
                
                # Print live flight blending weights and heights for all 3 UAVs
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
                    vlbl = (f"[{_vm[0]}] ATTN | "
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
        print("=== Policy Distillation Swarm Simulation finished successfully ===\n")

if __name__ == "__main__":
    run_distill_demo()
