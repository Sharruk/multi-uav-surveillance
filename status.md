# Project Status Tracker

## Phase 1: Environment & SLAM Foundation (Completed)

- [x] Initialize `gym-pybullet-drones` multi-agent environment.
- [x] Generate 3D static obstacles (buildings).
- [x] Implement linear-walking `RandomCrowd` targets with procedural building collision detection and auto-rerouting (replacing the flocking `BoidsCrowd`).
- [x] Build Raycast-to-Occupancy-Grid pipeline with adjustable noise (Simulated SLAM).
- [x] STIRS-2025 Environment Visual Overhaul & Premium Polish:
  - **Cinematic Orbital Camera**: Clean visualizer configuration (disabled sidebar/panels, enabled shadows) and a smooth, hands-free 360° rotating camera tracking the arena center.
  - **Premium Visual Fidelity**: Solid light-grey ground floor plane with textures cleared, concrete medium-light grey procedural buildings, and bright blue UAVs.
  - **RandomCrowd Walkers**: 12 bright yellow ground dots moving towards random goals with active collision checks against building footprints.
  - **Total Line Drawings Cleanup**: Removed all cyan ADGAT network lines, white trajectory trails, and the green/red LiDAR raycast sweeps to provide a pristine, cinematic visual view.
  - **CPU-Safe Performance Bypass**: Global `DEMO_MODE` master switch for instantly reverting to a headless DIRECT pipeline during training.
- [x] **Final Visual Polish**: Completely removed all `p.addUserDebugLine` calls and tracking dictionaries/removal code for LiDAR raycasting inside `_get_drone_sensors` to eliminate `User debug draw failed` warnings and enhance execution speed, while retaining 100% of the underlying raycast physics and local occupancy grid generation mathematics.
- [x] **Predictable Layout Toggle**: Implemented a `fixed_layout` parameter (default `False`) for `DroneSurveillanceEnv` to conditionally lock building and crowd generation seeds (`42`). This ensures consistent and reproducible layouts and target starting positions for cinematic demo sequences while preserving procedural generation for future RL training.

## Phase 2: The Brain & Network (Completed)

- [x] Map Occupancy Grid arrays to PettingZoo observation space.
- [x] Scaffold standard MADDPG baseline model (Ray RLlib).
- [x] Scaffold proposed custom policy (ARReSVG + LSTM memory for POMDP).

## Phase 3: UAV Swarm Dynamics & Swarm Dashboard (Completed)

- [x] **Realistic Multi-Body Spawning**: Assemble a high-fidelity 3D quadcopter model using `p.createMultiBody` (dark-grey base box [0.15, 0.15, 0.05] and 4 bright-blue cylindrical rotors [radius 0.08, height 0.02] offset to corners as links with FIXED joints).
- [x] **Torque Kinematics Control**: Replace the velocity controller in `_apply_flight_control` with physical forces and torques:
  - Vertical thrust in local LINK frame: `thrust_force = (Thrust + 1.0) * 10.0` N.
  - Directional torques in local LINK frame: `roll_torque = Roll * 0.5`, `pitch_torque = Pitch * 0.5`, `yaw_torque = Yaw * 0.5`.
  - Battery safety shutdown: Force thrust and torques to `0.0` if battery drops to `0.0%`.
- [x] **Battery & Wind Constraints**:
  - Drain battery by `0.01 + 0.03 * abs(thrust_action)` per step.
  - Apply random lateral wind forces `[-0.15, 0.15]` in global X/Y.
- [x] **Camera FOV Downward Cone**: Calculate a downward 45-degree cone camera view (distance `d <= Z`) for tracking ground crowd targets, replacing the old 5m 2D tracking logic in the reward.
- [x] **Holographic HUD Labels**: Draw a bright Cyan (`[0, 1, 1]`) floating debug label `UAV-i | Bat: {bat}% | Alt: {alt}m` hovering `0.5m` above each drone in real-time.
- [x] **Live Swarm Terminal Dashboard**: Print live UAV swarm status dynamically in-place on a single terminal line using `\r` and `end=""` inside the interactive demo loop.

## Phase 4: Testbed Integration & Swarm Control GUI Selector (Completed)

- [x] **Package Structuring**:
  - Relocate custom environment file to `envs/drone_env.py`.
  - Add empty `__init__.py` module setups for package imports.
  - Establish algorithms folder structures (`algorithms/obstacle_avoidance/`, `algorithms/swarm_coordination/`, `algorithms/collision_deconfliction/`, `algorithms/target_tracking/`).
- [x] **Algorithm Scaffolding**:
  - Implement Ray RLlib PPO Baseline config and 20-step demo simulator (`algorithms/obstacle_avoidance/ppo_baseline.py`).
  - Implement SDDPG-NAV split-network actor design and 20-step navigation demo (`algorithms/obstacle_avoidance/state_decomp_ddpg.py`).
  - Implement Attention-based Policy Distillation behavioral-blending controller and 20-step clearance demo (`algorithms/obstacle_avoidance/attention_distill.py`).
- [x] **Unified Control Panel (main_selector.py)**:
  - Create a premium dark-themed native Tkinter GUI panel centered on screen.
  - Provide Combobox dropdown selecting from the 3 configurations.
  - Code robust teardown sequence to prevent deadlock before executing direct PyBullet module imports.
- [x] **Dynamic Flight & Clash Telemetry Refinements**:
  - Implement $10\times$ physical sub-stepping inside the environment to align simulation time and crowd boids speed.
  - Scale up control tilt commands to `0.35` (tracking) and `0.45` (avoidance) to physically tilt, bank, and steer the drones dynamically.
  - Track cumulative clashes dynamically across all three scripts and output Unified Dashboards reporting battery, altitude, targets, and clashes for all 3 active UAVs.
