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
                    
                    # 3. ATTENTION FUSION / DISTILLATION
                    # Blend the controllers based on proximity risk attention weight (alpha)
                    alpha = proximity_risk
                    pitch_cmd = alpha * pitch_avoid + (1.0 - alpha) * pitch_track
                    roll_cmd = alpha * roll_avoid + (1.0 - alpha) * roll_track
                    
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
            
            # Orbital camera tracking
            if hasattr(env, 'client_id'):
                try:
                    p.resetDebugVisualizerCamera(
                        cameraDistance=12.0,
                        cameraPitch=-35,
                        cameraYaw=(step * 2.0) % 360.0,
                        cameraTargetPosition=[0.0, 0.0, 1.0],
                        physicsClientId=env.client_id
                    )
                except p.error:
                    break
            time.sleep(0.1)
    finally:
        print("\nClosing environment...")
        env.close()
        print("=== Policy Distillation Swarm Simulation finished successfully ===\n")

if __name__ == "__main__":
    run_distill_demo()
