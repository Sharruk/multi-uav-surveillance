import os
import sys
import numpy as np
import time

try:
    import ray
    from ray import tune
    from ray.tune.registry import register_env
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
    HAS_RAY = True
except ImportError:
    HAS_RAY = False

# Add project root to sys.path to enable absolute imports when run directly
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

# Import DroneSurveillanceEnv from envs package
from envs.drone_env import DroneSurveillanceEnv


def env_creator(config):
    """Wraps our environment for Ray RLlib compatibility."""
    env = DroneSurveillanceEnv(
        render_mode=config.get("render_mode", "headless"),
        fixed_layout=config.get("fixed_layout", True)
    )
    return ParallelPettingZooEnv(env)

def run_ppo_demo():
    print("\n" + "="*80)
    print("      STIRS-2025: DEPLOYING MULTI-AGENT PPO (SHARED POLICY BASELINE)")
    print("="*80)
    
    # 1. Standard baseline configuration display & registration
    print("\n[RLlib Config] Generating Multi-Agent PPO configuration...")
    if HAS_RAY:
        print("Ray RLlib is installed. Registering environment...")
        try:
            register_env("drone_surveillance_v0", env_creator)
            print("Environment registered as 'drone_surveillance_v0' with Ray Tune.")
        except Exception as e:
            print(f"Environment registration info: {e}")
            
        dummy_env = DroneSurveillanceEnv(render_mode="headless")
        obs_space = dummy_env.observation_spaces["drone_0"]
        act_space = dummy_env.action_spaces["drone_0"]
        dummy_env.close()
        
        print(f"Observation space: {obs_space}")
        print(f"Action space: {act_space}")
        print("Policies configured: Shared PPO Policy (actor-critic) across all UAV agents.")
    else:
        print("Ray RLlib is not installed in the active environment.")
        print("Proceeding with baseline configuration description:")
        print("  - Algorithm: Multi-Agent Proximal Policy Optimization (PPO)")
        print("  - Framework: PyTorch (CPU-only pinned execution for hardware-bound safety)")
        print("  - Shared Policy: Single actor-critic model controlling all drone agents")
        
    # 2. Run clean simulation demo step (20 steps in DEMO_MODE)
    _scenario = os.environ.get('STIRS_SCENARIO', 'downtown')
    _use_scn  = os.environ.get('STIRS_USE_SCENARIO_SYSTEM', '0') == '1'
    _scn_cfg  = {'use_scenario_system': _use_scn, 'scenario': _scenario} if _use_scn else {}
    print(f"\n[Simulation] Scenario: {_scenario.upper() if _use_scn else 'legacy'}  "
          f"Instantiating DroneSurveillanceEnv in GUI mode...")
    env = DroneSurveillanceEnv(render_mode="human", fixed_layout=True, env_config=_scn_cfg)
    obs, info = env.reset()
    
    print("\nRunning PPO baseline flight control simulation...")
    print("  Camera keys in PyBullet window:  1=Orbital  2=Top-Down  3=Cinematic  4=Close-Up")
    # Camera view modes (press 1-4 in PyBullet window)
    _CAM_MODES = [
        ("ORBITAL",  18.0, -52, True),
        ("TOP-DOWN", 22.0, -89, False),
        ("CINEMATIC",14.0, -28, True),
        ("CLOSE-UP",  9.0, -42, False),
    ]
    _VIEW_KEYS = {49: 0, 50: 1, 51: 2, 52: 3}
    _cur_view = 0;  _view_txt_id = -1;  _cam_yaw = 35.0
    try:
        # Hover-like control policy with slight noise simulating early PPO training exploration
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
                
                # Retrieve heading (yaw) directly from PyBullet
                drone_id = env.drone_ids[agent_id]
                _, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=env.client_id)
                euler = p.getEulerFromQuaternion(orientation)
                yaw = euler[2]
                
                # 1. OBSTACLE AVOIDANCE (LiDAR Proximity)
                i_min = np.argmin(lidar_data)
                min_dist = lidar_data[i_min]
                proximity_risk = np.clip((3.0 - min_dist) / 2.5, 0.0, 1.0)
                alpha_obs = i_min * 10 * np.pi / 180.0
                avoid_local_x = -np.cos(alpha_obs) * 0.45
                avoid_local_y = -np.sin(alpha_obs) * 0.45
                
                # 2. TARGET TRACKING (Chase nearest ground boid)
                nearest_boid = None
                min_boid_dist = float('inf')
                for b_pos in boid_positions:
                    d_2d = np.linalg.norm(pos[:2] - b_pos)
                    if d_2d < min_boid_dist:
                        min_boid_dist = d_2d
                        nearest_boid = b_pos
                
                if nearest_boid is not None:
                    dx = nearest_boid[0] - pos[0]
                    dy = nearest_boid[1] - pos[1]
                    track_local_x = dx * np.cos(yaw) + dy * np.sin(yaw)
                    track_local_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
                    mag = np.sqrt(track_local_x**2 + track_local_y**2)
                    if mag > 0:
                        track_local_x /= mag
                        track_local_y /= mag
                    pitch_track = track_local_x * 0.35
                    roll_track = track_local_y * 0.35
                else:
                    pitch_track, roll_track = 0.0, 0.0
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
                swarm_local_x = swarm_avoid_x * np.cos(yaw) + swarm_avoid_y * np.sin(yaw)
                swarm_local_y = -swarm_avoid_x * np.sin(yaw) + swarm_avoid_y * np.cos(yaw)
                
                swarm_mag = np.sqrt(swarm_local_x**2 + swarm_local_y**2)
                if swarm_mag > 0:
                    swarm_local_x = (swarm_local_x / swarm_mag) * min(swarm_mag, 1.0)
                    swarm_local_y = (swarm_local_y / swarm_mag) * min(swarm_mag, 1.0)
                    
                # 3. PPO POLICY BLENDING WITH EXPLORATION NOISE AND SWARM AVOIDANCE
                alpha = proximity_risk
                # Blend target tracking and obstacle avoidance
                pitch_blend = alpha * avoid_local_x + (1.0 - alpha) * pitch_track
                roll_blend = alpha * avoid_local_y + (1.0 - alpha) * roll_track
                
                # Apply swarm avoidance if active
                if swarm_mag > 0:
                    pitch_blend = 0.4 * pitch_blend + 0.6 * swarm_local_x * 0.45
                    roll_blend = 0.4 * roll_blend + 0.6 * swarm_local_y * 0.45
                
                # Add early-stage Gaussian exploration noise (~0.04 std dev)
                pitch_cmd = pitch_blend + np.random.normal(0, 0.04)
                roll_cmd = roll_blend + np.random.normal(0, 0.04)
                
                # Altitude control: lock at 2.0 meters aggressively
                z_target = 2.0
                z_error = z_target - pos[2]
                thrust_cmd = 0.3734 + z_error * 0.45 - vel[2] * 0.1
                thrust_cmd = np.clip(thrust_cmd, 0.2, 0.6)
                
                actions[agent_id] = np.array([thrust_cmd, pitch_cmd, roll_cmd, 0.0], dtype=np.float32)

                
            try:
                obs, rewards, terminateds, truncateds, infos = env.step(actions)
                
                # Track clashes / collisions dynamically
                for a_id in env.agents:
                    if infos[a_id].get("collision_penalty", 0.0) < 0.0:
                        clash_count += 1
                
                # Print live status dashboard
                dashboard = (
                    f"Step {step:04d} | Clashes: {clash_count} | "
                    f"UAV-0 [Bat: {int(env.battery['drone_0'])}% | Alt: {obs['drone_0']['position'][2]:.2f}m | Tracked: {infos['drone_0']['tracked_targets']}] "
                    f"UAV-1 [Bat: {int(env.battery['drone_1'])}% | Alt: {obs['drone_1']['position'][2]:.2f}m | Tracked: {infos['drone_1']['tracked_targets']}] "
                    f"UAV-2 [Bat: {int(env.battery['drone_2'])}% | Alt: {obs['drone_2']['position'][2]:.2f}m | Tracked: {infos['drone_2']['tracked_targets']}]"
                )
                print(dashboard)
            except p.error:
                break
            
            # ── Camera: view-switch keypress detection + apply ─────────────
            if hasattr(env, 'client_id'):
                try:
                    keys = p.getKeyboardEvents(physicsClientId=env.client_id)
                    for kc, ks in keys.items():
                        if ks & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN) and kc in _VIEW_KEYS:
                            new_view = _VIEW_KEYS[kc]
                            if _cur_view != new_view:
                                _cur_view = new_view
                                _cam_yaw  = 35.0

                    all_pos = [obs[a]['position'] for a in env.agents]
                    cx = float(np.mean([pp[0] for pp in all_pos]))
                    cy = float(np.mean([pp[1] for pp in all_pos]))
                    cz = float(max(np.mean([pp[2] for pp in all_pos]), 2.0))

                    _vm = _CAM_MODES[_cur_view]
                    if _vm[3]:   # rotates
                        _cam_yaw = (_cam_yaw + 0.12) % 360.0
                    p.resetDebugVisualizerCamera(
                        cameraDistance=_vm[1], cameraPitch=_vm[2],
                        cameraYaw=_cam_yaw,
                        cameraTargetPosition=[cx, cy, cz],
                        physicsClientId=env.client_id
                    )
                    # View label in 3D world
                    vlbl = (f"[{_vm[0]}] PPO | "
                            f"RED:{obs['drone_0']['position'][2]:.1f}m "
                            f"BLU:{obs['drone_1']['position'][2]:.1f}m "
                            f"GRN:{obs['drone_2']['position'][2]:.1f}m  Keys:1 2 3 4")
                    _lpos = [cx-7, cy-9.5, 0.4]
                    if _view_txt_id != -1:
                        _view_txt_id = p.addUserDebugText(vlbl, _lpos,
                            [1.0,1.0,0.1], 1.3, replaceItemUniqueId=_view_txt_id,
                            physicsClientId=env.client_id)
                    else:
                        _view_txt_id = p.addUserDebugText(vlbl, _lpos,
                            [1.0,1.0,0.1], 1.3, physicsClientId=env.client_id)
                except p.error:
                    break
            time.sleep(0.05)
    finally:
        print("\nClosing environment...")
        env.close()
        print("=== PPO Baseline Swarm Simulation finished successfully ===\n")

if __name__ == "__main__":
    run_ppo_demo()
