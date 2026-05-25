import os
from collections import deque
import math
import random
import time
import numpy as np
import pybullet as p
import pybullet_data

# ── Enhanced environment assets (trees, birds, wind, roads, houses, poles) ──
try:
    from envs.environment_assets import (
        WindSystem, TreeSpawner, BuildingSpawner,
        HouseSpawner, ElectricPoleSpawner, BirdFlock, RoadAndParkSpawner
    )
except ImportError:
    # Fallback for direct script execution (python drone_env.py)
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..")))
    from envs.environment_assets import (
        WindSystem, TreeSpawner, BuildingSpawner,
        HouseSpawner, ElectricPoleSpawner, BirdFlock, RoadAndParkSpawner
    )

# Global Demo Mode Toggle: Set to True to enable premium GUI mode, camera tracking, and 3D LiDAR rendering.
# In Docker, we set HEADLESS=1 to force headless mode and avoid X server errors.
DEMO_MODE = os.environ.get("HEADLESS", "0") != "1"

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces


def inject_slam_noise(grid, flip_prob=0.05, gaussian_std=0.05, drift_offset=(0, 0)):
    """
    Injects simulated SLAM noise into a 2D Local Occupancy Grid.
    
    Args:
        grid (np.ndarray): 16x16 grid of float values in [0.0, 1.0].
        flip_prob (float): Probability of randomly flipping any given cell.
        gaussian_std (float): Standard deviation of Gaussian noise added to cell values.
        drift_offset (tuple): (x_shift, y_shift) cell shift to simulate SLAM mapping drift.
        
    Returns:
        np.ndarray: The noisy grid, clipped to [0.0, 1.0].
    """
    noisy_grid = np.copy(grid)
    
    # 1. Inject Drift Offset (spatial displacement)
    if drift_offset != (0, 0):
        noisy_grid = np.roll(noisy_grid, shift=drift_offset, axis=(0, 1))
        
    # 2. Add Gaussian noise
    if gaussian_std > 0:
        noise = np.random.normal(0.0, gaussian_std, size=noisy_grid.shape)
        noisy_grid = np.clip(noisy_grid + noise, 0.0, 1.0)
        
    # 3. Apply Random Flips (sensor/communication interference)
    if flip_prob > 0:
        flip_mask = np.random.random(size=noisy_grid.shape) < flip_prob
        # A soft flip to preserve continuous bounds
        noisy_grid[flip_mask] = 1.0 - noisy_grid[flip_mask]
        
    return np.array(noisy_grid, dtype=np.float32)


class RandomCrowd:
    """
    Simulates a crowd of moving targets on the ground plane (z=0) walking linearly
    towards random destinations. Avoids procedurally generated static buildings.
    """
    def __init__(self, num_boids=12, boundary=8.0, physics_client_id=0, building_specs=None):
        self.num_boids = num_boids
        self.boundary = boundary
        self.physics_client_id = physics_client_id
        self.building_specs = building_specs if building_specs is not None else []
        self.boids = []
        
        # Create visual shapes for dots in PyBullet (bright yellow spheres)
        self.col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.15, physicsClientId=physics_client_id)
        self.vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[1.0, 0.85, 0.0, 1.0], physicsClientId=physics_client_id)
        
        for i in range(num_boids):
            pos = self.get_random_valid_pos()
            dest = self.get_random_valid_pos()
            
            # Spawn in PyBullet as a static/kinematic body
            boid_body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=self.col_id,
                baseVisualShapeIndex=self.vis_id,
                basePosition=[pos[0], pos[1], 0.15],
                physicsClientId=physics_client_id
            )
            
            if DEMO_MODE:
                p.changeVisualShape(boid_body_id, -1, rgbaColor=[1.0, 0.85, 0.0, 1.0], physicsClientId=physics_client_id)
                
            self.boids.append({
                'id': boid_body_id,
                'pos': pos,
                'dest': dest
            })
            
    def is_in_building(self, pos, margin=0.2):
        x, y = pos[0], pos[1]
        for spec in self.building_specs:
            cx, cy = spec["center"]
            hx, hy = spec["half_extents"][0], spec["half_extents"][1]
            if (cx - hx - margin) <= x <= (cx + hx + margin) and (cy - hy - margin) <= y <= (cy + hy + margin):
                return True
        return False

    def get_random_valid_pos(self):
        for _ in range(200):
            x = random.uniform(-self.boundary, self.boundary)
            y = random.uniform(-self.boundary, self.boundary)
            if not self.is_in_building([x, y]):
                return np.array([x, y])
        return np.array([0.0, 0.0])

    def update(self):
        speed = 0.05
        for b in self.boids:
            pos = b['pos']
            dest = b['dest']
            
            direction = dest - pos
            dist = np.linalg.norm(direction)
            
            if dist < speed:
                dest = self.get_random_valid_pos()
                b['dest'] = dest
                direction = dest - pos
                dist = np.linalg.norm(direction)
                
            if dist > 0:
                direction = direction / dist
                next_pos = pos + direction * speed
                
                # Collision check
                if self.is_in_building(next_pos, margin=0.2):
                    dest = self.get_random_valid_pos()
                    b['dest'] = dest
                    direction = dest - pos
                    dist = np.linalg.norm(direction)
                    if dist > 0:
                        direction = direction / dist
                        next_pos = pos + direction * speed
                        if not self.is_in_building(next_pos, margin=0.2):
                            pos = next_pos
                else:
                    pos = next_pos
                    
            b['pos'] = pos
            
            # Reset visual body in PyBullet (z=0.15 keeps it sitting perfectly on the plane)
            p.resetBasePositionAndOrientation(
                b['id'],
                posObj=[pos[0], pos[1], 0.15],
                ornObj=[0, 0, 0, 1],
                physicsClientId=self.physics_client_id
            )



class DroneSurveillanceEnv(gym.Env):
    """
    Custom Gymnasium Multi-Agent environment for decentralized surveillance.
    Follows PettingZoo's ParallelEnv API format.
    
    Includes 3 UAVs, 4 static urban buildings, and a crowd of 12 boids moving on the ground.
    Utilizes 360-degree horizontal 1D raycasting for local occupancy grids with custom SLAM noise.
    """
    metadata = {"render_modes": ["human", "headless"], "name": "drone_surveillance_v0"}

    # ── Default configuration for enhanced environmental features ──────────
    DEFAULT_ENV_CONFIG = {
        "enable_trees":        True,   # Spawn tree clusters (affects LiDAR)
        "enable_houses":       True,   # Spawn small residential houses
        "enable_poles":        True,   # Spawn utility poles along roads
        "enable_birds":        True,   # Spawn flying bird flock
        "enable_wind_physics": True,   # Realistic wind system
        "enable_roads":        True,   # Spawn road/park planes (GUI only)
        "num_trees":           12,     # Reduced: cleaner scene for research
        "num_birds":           8,      # Bird flock size
        "wind_base_speed":     0.4,    # SAFE: ~0.4N force (was 2.0 = drone-crashing)
        "num_houses":          6,      # Reduced: less clutter
        "num_tall_buildings":  3,      # Reduced: was 6, blocked entire camera view
        # ── Scenario system (Phase 1) ────────────────────────────────────────
        "use_scenario_system": False,  # Set True to use procedural city scenarios
        "scenario":            "downtown",  # downtown|residential|event|mixed|industrial
        "arena_xy_bound":      13.0,   # Overridden per scenario when system is active
    }

    def __init__(self, render_mode="headless", noise_params=None, fixed_layout=False,
                 env_config=None):
        super().__init__()
        self.render_mode = render_mode
        self.fixed_layout = fixed_layout
        self.noise_params = noise_params if noise_params is not None else {
            "flip_prob": 0.05,
            "gaussian_std": 0.05,
            "drift_offset": (0, 0)
        }

        # ── Merge user config with safe defaults (backward compatible) ────────
        self.env_config = dict(self.DEFAULT_ENV_CONFIG)
        if env_config is not None:
            self.env_config.update(env_config)

        # Start PyBullet client (GUI if DEMO_MODE is True or render_mode is human)
        if DEMO_MODE or self.render_mode == "human":
            self.client_id = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
        else:
            self.client_id = p.connect(p.DIRECT)

        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)

        # PettingZoo interface agents setup
        self.agents = ["drone_0", "drone_1", "drone_2"]
        self.possible_agents = self.agents[:]

        # No lidar line drawing tracking in clean visual mode

        # Define 16x16 local occupancy grid parameters
        self.grid_size = 16
        self.max_lidar_range = 8.0  # meters (covers grid size of 16m total)

        # Multi-Agent observation space
        self.observation_spaces = {
            agent_id: spaces.Dict({
                "position":      spaces.Box(low=-50.0, high=50.0, shape=(3,), dtype=np.float32),
                "velocity":      spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
                "lidar":         spaces.Box(low=0.0, high=self.max_lidar_range, shape=(36,), dtype=np.float32),
                "occupancy_grid": spaces.Box(low=0.0, high=1.0, shape=(self.grid_size, self.grid_size), dtype=np.float32)
            }) for agent_id in self.agents
        }

        # Multi-Agent action space (Continuous: Thrust, Pitch, Roll, Yaw)
        self.action_spaces = {
            agent_id: spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
            for agent_id in self.agents
        }

        # Drones standard initial positions
        self.drone_init_positions = [
            [-3.0, 0.0, 2.0],
            [0.0, -3.0, 2.0],
            [3.0, 0.0, 2.0]
        ]

        # ── Environment episode variables (original) ──────────────────────────
        self.max_steps = 500
        self.current_step = 0
        self.drone_ids = {}
        self.building_ids = []
        self.crowd = None
        self.plane_id = None

        # ── Enhanced asset tracking lists ─────────────────────────────────────
        self.tree_ids      = []   # All tree body IDs (trunks + foliage)
        self.house_ids     = []   # All house body IDs
        self.pole_ids      = []   # All electric pole body IDs
        self.bird_ids      = []   # All bird body IDs
        self.road_ids      = []   # Road/park plane body IDs (GUI only)
        self.tall_bld_ids  = []   # Extra tall building body IDs
        self.furniture_ids = []   # Phase-2 street furniture body IDs

        # ── Wind system (initialised in reset() with optional seed) ───────────
        self.wind_system   = None
        self.bird_flock    = None  # BirdFlock instance (recreated each reset)

        # ── Arena boundary (may be overridden by scenario system) ─────────────
        self._arena_xy_bound = self.env_config.get("arena_xy_bound", 13.0)
        self.building_specs  = []   # [{center, half_extents}] for crowd avoidance

    def _spawn_scenario_environment(self):
        """
        Phase-1 scenario system: replace legacy random building spawning with a
        procedurally generated urban environment (roads, sidewalks, architectural
        buildings, scenario-specific extras).

        Populates self.building_ids, self.tall_bld_ids, self.house_ids, and
        self.building_specs in formats fully compatible with the existing
        collision-detection and crowd-avoidance code.
        Also updates self._arena_xy_bound so the out-of-bounds check expands
        to match the new city footprint.
        """
        try:
            from envs.scenarios import load_scenario
        except ImportError:
            import sys as _sys, os as _os
            _sys.path.insert(0, _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..")))
            from envs.scenarios import load_scenario

        scenario_name = self.env_config.get("scenario", "downtown")
        seed = 42 if self.fixed_layout else random.randint(0, 9999)
        scn  = load_scenario(scenario_name, self.client_id, self.env_config, seed=seed)
        result = scn.spawn()

        self.building_ids   = result.get("building_ids", [])
        self.tall_bld_ids   = result.get("tall_bld_ids",  [])
        self.house_ids      = result.get("house_ids",     [])
        self.building_specs = result.get("building_specs", [])
        self.furniture_ids  = result.get("furniture_ids", [])
        self._arena_xy_bound = result.get("arena_bound", 14.0)

        if DEMO_MODE:
            # Show scenario name as debug label
            label = f"Scenario: {scenario_name.upper()}"
            p.addUserDebugText(label, [-11.0, -11.0, 0.5],
                               textColorRGB=[0.9, 0.8, 0.3], textSize=1.2,
                               physicsClientId=self.client_id)

    def _spawn_buildings(self):
        """Spawns 8-15 randomized static 3D buildings as boxes forming an urban canyon, checking collisions."""
        self.building_ids = []
        if self.fixed_layout:
            np.random.seed(42)
            random.seed(42)
        num_buildings = random.randint(8, 15)
        
        # Keep track of building specs for logging/reference if needed
        self.building_specs = []
        
        for _ in range(num_buildings):
            # Attempt coordinates generation up to 100 times to avoid spawning on drones
            for attempt in range(100):
                # Randomized dimensions
                # widths (1m to 3m total) => half-extents in [0.5, 1.5]
                half_x = random.uniform(0.5, 1.5)
                half_y = random.uniform(0.5, 1.5)
                # heights (2m to 8m total) => half-extents in [1.0, 4.0]
                half_z = random.uniform(1.0, 4.0)
                
                # Randomized coordinates in arena boundary [-9.0, 9.0]
                bx = random.uniform(-9.0, 9.0)
                by = random.uniform(-9.0, 9.0)
                
                # Collision Check: Do not spawn directly on top of the drone initial positions
                collision = False
                for drone_pos in self.drone_init_positions:
                    dist_2d = math.sqrt((bx - drone_pos[0])**2 + (by - drone_pos[1])**2)
                    min_dist = math.sqrt(half_x**2 + half_y**2) + 1.2
                    if dist_2d < min_dist:
                        collision = True
                        break
                        
                if not collision:
                    break
            
            # Use the generated building parameters
            half_extents = [half_x, half_y, half_z]
            pos = [bx, by, half_z] # Center of the box stands on z=0
            
            self.building_specs.append({
                "center": [bx, by],
                "half_extents": half_extents
            })
            
            col_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                physicsClientId=self.client_id
            )
            # Generate concrete medium-light grey building colors
            grey = random.uniform(0.55, 0.65)
            vis_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=[grey, grey, grey, 1.0],
                physicsClientId=self.client_id
            )
            
            building_id = p.createMultiBody(
                baseMass=0, # static
                baseCollisionShapeIndex=col_shape,
                baseVisualShapeIndex=vis_shape,
                basePosition=pos,
                physicsClientId=self.client_id
            )
            if DEMO_MODE:
                p.changeVisualShape(building_id, -1, rgbaColor=[grey, grey, grey, 1.0], physicsClientId=self.client_id)
            self.building_ids.append(building_id)

    # Per-drone identity colours — used for body, HUD, and trajectory trails
    DRONE_COLORS = {
        "drone_0": {"body": [0.95, 0.15, 0.15, 1.0],  "name": "UAV-RED",   "hud": [1.0, 0.3, 0.3]},
        "drone_1": {"body": [0.10, 0.55, 1.00, 1.0],  "name": "UAV-BLU",   "hud": [0.3, 0.7, 1.0]},
        "drone_2": {"body": [0.10, 0.90, 0.30, 1.0],  "name": "UAV-GRN",   "hud": [0.2, 1.0, 0.4]},
    }

    def _spawn_drones(self):
        """
        Assembles large, colour-coded quadcopter models for clear research visibility.

        Each drone has:
        - A large flat body box (0.35m) in a unique colour (Red / Blue / Green)
        - 4 bright YELLOW rotor discs placed wide apart for visual span
        - A glowing white BEACON sphere on top (easy to spot from any angle)
        - Body mass set so thrust stays balanced against gravity
        """
        self.drone_ids = {}
        self._drone_trail_pos = {}   # last known pos for trajectory lines

        for i, spawn_pos in enumerate(self.drone_init_positions):
            agent_id = f"drone_{i}"
            dc = self.DRONE_COLORS[agent_id]
            body_rgba = dc["body"]

            # ── Body: large flat box (0.35 x 0.35 x 0.07 m) ───────────────────
            base_col_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[0.35, 0.35, 0.07],
                physicsClientId=self.client_id
            )
            base_vis_id = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.35, 0.35, 0.07],
                rgbaColor=body_rgba,
                physicsClientId=self.client_id
            )

            # ── Rotors: bright YELLOW discs, placed wide (0.32m offset) ─────────
            rotor_col_id = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=0.16,
                height=0.025,
                physicsClientId=self.client_id
            )
            rotor_vis_id = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.16,
                length=0.025,
                rgbaColor=[1.0, 0.90, 0.0, 1.0],   # bright yellow
                physicsClientId=self.client_id
            )

            # ── Beacon: small glowing white sphere on top ───────────────────
            beacon_col_id = p.createCollisionShape(
                p.GEOM_SPHERE, radius=0.10,
                physicsClientId=self.client_id
            )
            beacon_vis_id = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.10,
                rgbaColor=[1.0, 1.0, 1.0, 1.0],   # white beacon
                physicsClientId=self.client_id
            )

            # Rotor positions: 4 corners at ±0.32m in XY, +0.04m above body centre
            link_positions = [
                [ 0.32,  0.32, 0.04],
                [-0.32,  0.32, 0.04],
                [-0.32, -0.32, 0.04],
                [ 0.32, -0.32, 0.04],
                [ 0.00,  0.00, 0.14],  # beacon sits on top
            ]
            link_masses       = [0.05, 0.05, 0.05, 0.05, 0.01]
            link_col_ids      = [rotor_col_id] * 4 + [beacon_col_id]
            link_vis_ids      = [rotor_vis_id] * 4 + [beacon_vis_id]
            link_orientations = [[0, 0, 0, 1]] * 5
            link_parent_indices = [0] * 5
            link_joint_types  = [p.JOINT_FIXED] * 5
            link_joint_axes   = [[0, 0, 1]] * 5

            drone_id = p.createMultiBody(
                baseMass=1.0,
                baseCollisionShapeIndex=base_col_id,
                baseVisualShapeIndex=base_vis_id,
                basePosition=spawn_pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                linkMasses=link_masses,
                linkCollisionShapeIndices=link_col_ids,
                linkVisualShapeIndices=link_vis_ids,
                linkPositions=link_positions,
                linkOrientations=link_orientations,
                linkInertialFramePositions=[[0, 0, 0]] * 5,
                linkInertialFrameOrientations=[[0, 0, 0, 1]] * 5,
                linkParentIndices=link_parent_indices,
                linkJointTypes=link_joint_types,
                linkJointAxis=link_joint_axes,
                physicsClientId=self.client_id
            )

            self.drone_ids[agent_id] = drone_id
            self._drone_trail_pos[agent_id] = list(spawn_pos)

            # Label each drone above its spawn position
            if DEMO_MODE:
                p.addUserDebugText(
                    text=dc["name"],
                    textPosition=[spawn_pos[0], spawn_pos[1], spawn_pos[2] + 1.2],
                    textColorRGB=dc["hud"][:3],
                    textSize=1.8,
                    physicsClientId=self.client_id
                )


    def _apply_flight_control(self, agent_id, action):
        """Applies torque kinematics and thrust force based on the (4,) action input."""
        drone_id = self.drone_ids[agent_id]
        
        # Get current state from PyBullet
        pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
        euler = p.getEulerFromQuaternion(orientation)
        current_roll, current_pitch = euler[0], euler[1]
        
        # Passive stabilizing torques (attitude controller) to prevent instant flipping
        stabilizing_roll = -2.0 * current_roll
        stabilizing_pitch = -2.0 * current_pitch
        
        Thrust, Pitch, Roll, Yaw = action[0], action[1], action[2], action[3]
        
        if self.battery[agent_id] <= 0.0:
            thrust_force = 0.0
            roll_torque = 0.0
            pitch_torque = 0.0
            yaw_torque = 0.0
        else:
            thrust_force = (Thrust + 1.0) * 10.0 # 0 to 20 N force
            roll_torque = Roll * 0.5 + stabilizing_roll
            pitch_torque = Pitch * 0.5 + stabilizing_pitch
            yaw_torque = Yaw * 0.5
            
        # Apply vertical thrust force in local LINK frame of drone
        p.applyExternalForce(
            drone_id,
            -1,
            [0.0, 0.0, thrust_force],
            [0.0, 0.0, 0.0],
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id
        )
        
        # Apply roll, pitch, and yaw torques in local LINK frame of drone
        p.applyExternalTorque(
            drone_id,
            -1,
            [roll_torque, pitch_torque, yaw_torque],
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id
        )

    def _get_drone_sensors(self, agent_id):
        """Performs 360-degree horizontal 1D raycast and generates local occupancy grid with SLAM noise."""
        drone_id = self.drone_ids[agent_id]
        pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
        euler = p.getEulerFromQuaternion(orientation)
        yaw = euler[2]
        
        num_rays = 36
        max_range = self.max_lidar_range
        start_offset = 0.25 # Offset ray starts outside drone physics body to avoid self-hits
        
        ray_starts = []
        ray_ends = []
        
        for i in range(num_rays):
            angle = yaw + math.radians(i * 10)
            start = [
                pos[0] + start_offset * math.cos(angle),
                pos[1] + start_offset * math.sin(angle),
                pos[2]
            ]
            end = [
                pos[0] + max_range * math.cos(angle),
                pos[1] + max_range * math.sin(angle),
                pos[2]
            ]
            ray_starts.append(start)
            ray_ends.append(end)
            
        # Efficient batch raycasting in PyBullet
        results = p.rayTestBatch(ray_starts, ray_ends, physicsClientId=self.client_id)
        
        distances = []
        for i, res in enumerate(results):
            hit_id = res[0]
            hit_fraction = res[2]
            
            # If hit is an obstacle or building (and not the drone itself)
            if hit_id >= 0 and hit_id != drone_id and hit_fraction < 1.0:
                dist = start_offset + hit_fraction * (max_range - start_offset)
            else:
                dist = max_range
            distances.append(dist)
            
        # Build local occupancy grid (16x16) centered on drone in drone's local coordinate frame
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        L = max_range
        
        for i, dist in enumerate(distances):
            # Angles relative to drone body coordinate frame
            local_angle = math.radians(i * 10)
            x_local = dist * math.cos(local_angle)
            y_local = dist * math.sin(local_angle)
            
            # Calculate grid coordinate indices [0, 15]
            col = int(np.clip((x_local + L) / (2.0 * L) * self.grid_size, 0, self.grid_size - 1))
            row = int(np.clip((y_local + L) / (2.0 * L) * self.grid_size, 0, self.grid_size - 1))
            
            grid[row, col] = 1.0
            
        # Apply simulated SLAM noise (adjustable)
        noisy_grid = inject_slam_noise(
            grid,
            flip_prob=self.noise_params.get("flip_prob", 0.05),
            gaussian_std=self.noise_params.get("gaussian_std", 0.05),
            drift_offset=self.noise_params.get("drift_offset", (0, 0))
        )
        
        return np.array(distances, dtype=np.float32), noisy_grid

    def reset(self, seed=None, options=None):
        """Resets the environment and spawns all assets (original + enhanced)."""
        super().reset(seed=seed)
        self.current_step = 0

        # Initialize battery state to 100.0% for all agents
        self.battery = {agent_id: 100.0 for agent_id in self.agents}

        # Initialize HUD debug text item IDs
        self.hud_text_ids = {agent_id: -1 for agent_id in self.agents}

        # Clear enhanced asset ID lists from previous episode
        self.tree_ids.clear()
        self.house_ids.clear()
        self.pole_ids.clear()
        self.bird_ids.clear()
        self.road_ids.clear()
        self.tall_bld_ids.clear()

        p.resetSimulation(physicsClientId=self.client_id)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client_id)

        # Enforce no GPU rendering by disabling shadow maps and loading plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        try:
            self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        except Exception:
            # Planar geometry fallback
            plane_col = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client_id)
            self.plane_id = p.createMultiBody(0, plane_col, physicsClientId=self.client_id)

        if DEMO_MODE:
            # Richer ground: warm sandy/earthy tone instead of plain grey
            p.changeVisualShape(
                self.plane_id, -1,
                rgbaColor=[0.76, 0.72, 0.62, 1.0],  # sandy earth tone
                textureUniqueId=-1,
                physicsClientId=self.client_id
            )

        # ── Seeding for reproducible layouts ─────────────────────────────────
        if self.fixed_layout:
            np.random.seed(42)
            random.seed(42)

        # ── Building / terrain spawning ───────────────────────────────────────
        use_scenario = self.env_config.get("use_scenario_system", False)

        if use_scenario:
            # ── Phase-1 procedural scenario system ────────────────────────────
            self._spawn_scenario_environment()
            # arena_xy_bound set inside _spawn_scenario_environment()

            # Build exclusion list from drone positions + scenario buildings
            exclusion_zones = [[dpos[0], dpos[1]] for dpos in self.drone_init_positions]
            for spec in self.building_specs:
                exclusion_zones.append(list(spec["center"]))

        else:
            # ── Legacy random building spawning (unchanged) ───────────────────
            self._arena_xy_bound = self.env_config.get("arena_xy_bound", 13.0)
            self._spawn_buildings()

            # Build exclusion list
            exclusion_zones = [[dpos[0], dpos[1]] for dpos in self.drone_init_positions]
            for spec in self.building_specs:
                exclusion_zones.append(list(spec["center"]))

            # Legacy roads (GUI decorative)
            if DEMO_MODE and self.env_config["enable_roads"]:
                road_spawner = RoadAndParkSpawner(physics_client_id=self.client_id)
                self.road_ids = road_spawner.spawn(arena=10.0)

            # Legacy tall landmark buildings
            if self.env_config["num_tall_buildings"] > 0:
                tall_spawner = BuildingSpawner(
                    physics_client_id=self.client_id,
                    exclusion_zones=exclusion_zones[:]
                )
                self.tall_bld_ids = tall_spawner.spawn(
                    num_buildings=self.env_config["num_tall_buildings"],
                    arena=9.0
                )
                for bid in self.tall_bld_ids:
                    try:
                        pos_t, _ = p.getBasePositionAndOrientation(bid, physicsClientId=self.client_id)
                        exclusion_zones.append([pos_t[0], pos_t[1]])
                    except Exception:
                        pass

            # Legacy small houses
            if self.env_config["enable_houses"]:
                house_spawner = HouseSpawner(
                    physics_client_id=self.client_id,
                    exclusion_zones=exclusion_zones[:]
                )
                self.house_ids = house_spawner.spawn(
                    num_houses=self.env_config["num_houses"],
                    arena=9.0
                )
                for bid in self.house_ids:
                    try:
                        pos_h, _ = p.getBasePositionAndOrientation(bid, physicsClientId=self.client_id)
                        exclusion_zones.append([pos_h[0], pos_h[1]])
                    except Exception:
                        pass

        # 4. Trees — affects LiDAR (trunks are solid collision cylinders)
        _tree_arena = min(self._arena_xy_bound * 0.75, 11.0)
        if self.env_config["enable_trees"]:
            tree_spawner = TreeSpawner(
                physics_client_id=self.client_id,
                exclusion_zones=exclusion_zones[:]
            )
            self.tree_ids = tree_spawner.spawn(
                num_trees=self.env_config["num_trees"],
                arena=_tree_arena
            )

        # 5. Electric poles (GUI enrichment + minor LiDAR hits from thin trunks)
        if DEMO_MODE and self.env_config["enable_poles"]:
            pole_spawner = ElectricPoleSpawner(
                physics_client_id=self.client_id,
                exclusion_zones=exclusion_zones[:]
            )
            self.pole_ids = pole_spawner.spawn(arena=_tree_arena)

        # 6. Spawn ground crowd (original moving boids)
        _crowd_boundary = min(self._arena_xy_bound * 0.70, 10.0)
        self.crowd = RandomCrowd(
            num_boids=12, boundary=_crowd_boundary,
            physics_client_id=self.client_id,
            building_specs=self.building_specs
        )

        # 7. Flying bird flock (real-world random altitude behaviour)
        if self.env_config["enable_birds"]:
            self.bird_flock = BirdFlock(
                physics_client_id=self.client_id,
                num_birds=self.env_config["num_birds"]
            )
            self.bird_ids = self.bird_flock.spawn(arena=8.5)
        else:
            self.bird_flock = None
            self.bird_ids = []

        # 8. Wind system (fresh instance per episode — new wind direction)
        if self.env_config["enable_wind_physics"]:
            wind_seed = 42 if self.fixed_layout else None
            self.wind_system = WindSystem(
                base_speed=self.env_config["wind_base_speed"],
                seed=wind_seed
            )
        else:
            self.wind_system = None

        # ── Spawn UAVs (original) ─────────────────────────────────────────────
        self._spawn_drones()

        # Let PyBullet settle physics
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.client_id)

        # Re-initialize/clear debug lines for all agents on reset
        if DEMO_MODE:
            p.removeAllUserDebugItems(physicsClientId=self.client_id)

        # ── Improved lighting setup (GUI mode) ───────────────────────────────
        if DEMO_MODE:
            # Directional light from upper-right (sun-like)
            p.configureDebugVisualizer(
                p.COV_ENABLE_SHADOWS, 1,
                physicsClientId=self.client_id
            )
            # Wind direction indicator text (compass rose style)
            if self.wind_system is not None:
                wind_dir = self.wind_system.get_direction_degrees()
                _lbl_pos = -self._arena_xy_bound * 0.72
                p.addUserDebugText(
                    text=f"Wind: {wind_dir:.0f}° | {self.env_config['wind_base_speed']:.1f} m/s",
                    textPosition=[_lbl_pos, _lbl_pos, 1.5],
                    textColorRGB=[0.8, 0.9, 1.0],
                    textSize=1.0,
                    physicsClientId=self.client_id
                )

        # Set initial camera — nice 3/4 orbital view matching the reference image
        if DEMO_MODE:
            _cam_dist = max(18.0, self._arena_xy_bound * 1.4)
            p.resetDebugVisualizerCamera(
                cameraDistance=_cam_dist,
                cameraPitch=-52,
                cameraYaw=35,
                cameraTargetPosition=[0.0, 0.0, 2.5],
                physicsClientId=self.client_id
            )
            # Key-hint banner shown in 3D world space
            _hint_x = -self._arena_xy_bound * 0.65
            _hint_y = -self._arena_xy_bound * 0.75
            p.addUserDebugText(
                text="CAMERA: [1] Orbital  [2] Top-Down  [3] Cinematic  [4] Close-Up",
                textPosition=[_hint_x, _hint_y, 0.3],
                textColorRGB=[1.0, 1.0, 0.2],
                textSize=1.2,
                physicsClientId=self.client_id
            )

        obs = {}
        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
            vel, _ = p.getBaseVelocity(drone_id, physicsClientId=self.client_id)

            lidar, grid = self._get_drone_sensors(agent_id)
            obs[agent_id] = {
                "position":      np.array(pos, dtype=np.float32),
                "velocity":      np.array(vel, dtype=np.float32),
                "lidar":         lidar,
                "occupancy_grid": grid
            }

        info = {agent_id: {} for agent_id in self.agents}
        return obs, info

    def step(self, actions):
        """Steps the simulation environment with 10x sub-stepping to align control rate with physical time."""
        self.current_step += 1

        # 1. Update ground boid crowd positions (once per control step — original)
        self.crowd.update()

        # 2. Update flying bird flock positions (real-world altitude cycling)
        if self.bird_flock is not None:
            self.bird_flock.update(dt=0.02, arena=9.5)

        # 3. Pre-compute per-drone wind forces ONCE per control step (avoids repeated Gaussian samples
        #    in the inner sub-step loop while still being physically varied per drone position)
        wind_forces = {}
        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
            if self.wind_system is not None:
                # Realistic directional wind: height-dependent, slow direction drift, turbulence
                fx, fy, fz = self.wind_system.get_force(pos, self.current_step)
                wind_forces[agent_id] = [fx, fy, fz]
            else:
                # Legacy fallback: original random lateral wind
                wind_forces[agent_id] = [
                    random.uniform(-0.15, 0.15),
                    random.uniform(-0.15, 0.15),
                    0.0
                ]

        # 4. Run physical simulation sub-steps (10 sub-steps of 1/240s each, ~0.042s per control step)
        sub_steps = 10
        for _ in range(sub_steps):
            for agent_id in self.agents:
                drone_id = self.drone_ids[agent_id]

                # Apply flight control if action is provided
                # (PyBullet clears forces after each stepSimulation — must re-apply each sub-step)
                if agent_id in actions:
                    action = actions[agent_id]
                    self._apply_flight_control(agent_id, action)

                # Apply wind force in world frame
                wf = wind_forces[agent_id]
                p.applyExternalForce(
                    drone_id, -1,
                    wf,
                    [0.0, 0.0, 0.0],
                    flags=p.WORLD_FRAME,
                    physicsClientId=self.client_id
                )

            p.stepSimulation(physicsClientId=self.client_id)

        # 5. Drain battery once per control step (original logic unchanged)
        for agent_id in self.agents:
            if agent_id in actions:
                thrust_action = actions[agent_id][0]
                drain = 0.01 + 0.03 * abs(thrust_action)
            else:
                drain = 0.01
            self.battery[agent_id] = float(np.clip(self.battery[agent_id] - drain, 0.0, 100.0))

        # 6. Generate outputs
        obs        = {}
        rewards    = {}
        terminateds = {}
        truncateds  = {}
        infos       = {}

        boid_positions = [b['pos'] for b in self.crowd.boids]

        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
            vel, _ = p.getBaseVelocity(drone_id, physicsClientId=self.client_id)
            pos_np = np.array(pos)

            # ── Holographic HUD labels (DEMO_MODE only) ───────────────────────
            if DEMO_MODE:
                wf = wind_forces[agent_id]
                wind_mag = math.sqrt(wf[0]**2 + wf[1]**2)
                dc = self.DRONE_COLORS[agent_id]
                label_text = (
                    f"{dc['name']} | "
                    f"Bat:{int(self.battery[agent_id])}% | "
                    f"Alt:{pos_np[2]:.1f}m | "
                    f"W:{wind_mag:.1f}N"
                )
                # Float label 1.0m above drone so it's visible from high camera
                text_pos = [pos_np[0], pos_np[1], pos_np[2] + 1.0]
                hud_color = dc["hud"]
                if self.hud_text_ids.get(agent_id, -1) != -1:
                    self.hud_text_ids[agent_id] = p.addUserDebugText(
                        text=label_text,
                        textPosition=text_pos,
                        textColorRGB=hud_color,
                        textSize=1.5,
                        replaceItemUniqueId=self.hud_text_ids[agent_id],
                        physicsClientId=self.client_id
                    )
                else:
                    self.hud_text_ids[agent_id] = p.addUserDebugText(
                        text=label_text,
                        textPosition=text_pos,
                        textColorRGB=[0.0, 1.0, 1.0],
                        textSize=1.2,
                        physicsClientId=self.client_id
                    )

            lidar, grid = self._get_drone_sensors(agent_id)
            obs[agent_id] = {
                "position":      np.array(pos, dtype=np.float32),
                "velocity":      np.array(vel, dtype=np.float32),
                "lidar":         lidar,
                "occupancy_grid": grid
            }

            # ── A. Tracking Reward (downward 45° cone camera FOV: d <= Z) ─────
            tracked_count = 0
            alt = pos_np[2]
            for b_pos in boid_positions:
                dist_2d = np.linalg.norm(pos_np[:2] - b_pos)
                if dist_2d <= alt:
                    tracked_count += 1
            tracking_reward = 0.6 * tracked_count

            # ── B. Collision Detection ────────────────────────────────────────
            collision_penalty = 0.0
            bird_collisions   = 0

            # Collision with original buildings
            for b_id in self.building_ids:
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id, bodyB=b_id,
                    distance=1.0,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 6.0

            # Collision with tall landmark buildings
            for b_id in self.tall_bld_ids:
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id, bodyB=b_id,
                    distance=1.0,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 6.0

            # Collision with houses
            for h_id in self.house_ids:
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id, bodyB=h_id,
                    distance=0.8,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 4.0

            # Collision with flying birds (new: real-world hazard)
            for bird_id in self.bird_ids:
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id, bodyB=bird_id,
                    distance=1.0,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 3.0   # Bird strike penalty
                    bird_collisions   += 1

            # Collision with other drones
            for other_id in self.agents:
                if other_id == agent_id:
                    continue
                other_drone_id = self.drone_ids[other_id]
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id, bodyB=other_drone_id,
                    distance=1.0,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 4.0

            # ── C. Out-of-Bounds checking ────────────────────────────────────
            out_of_bounds = False
            xy_lim = self._arena_xy_bound
            z_max  = self.env_config.get("arena", {}).get("z_max", 18.0)
            if (abs(pos_np[0]) > xy_lim or abs(pos_np[1]) > xy_lim
                    or pos_np[2] < 0.4 or pos_np[2] > z_max):
                out_of_bounds = True
                collision_penalty -= 10.0

            # Control effort regularization penalty
            act_effort = np.sum(np.square(actions.get(agent_id, 0.0)))
            rewards[agent_id] = tracking_reward + collision_penalty - 0.01 * act_effort

            # Episode Termination conditions (out of bounds)
            terminateds[agent_id] = out_of_bounds

            # Truncation conditions (episode time limit)
            truncateds[agent_id] = (self.current_step >= self.max_steps)

            infos[agent_id] = {
                "tracked_targets":  tracked_count,
                "out_of_bounds":    out_of_bounds,
                "collision_penalty": collision_penalty,
                "bird_collisions":  bird_collisions,   # NEW: bird strike count
                "wind_force_xy":    math.sqrt(wind_forces[agent_id][0]**2
                                              + wind_forces[agent_id][1]**2),  # NEW
            }

        return obs, rewards, terminateds, truncateds, infos

    def close(self):
        """Clean up PyBullet client session."""
        if p.isConnected(self.client_id):
            p.disconnect(self.client_id)


def print_ascii_grid(grid):
    """Utility to print a 16x16 occupancy grid as readable ASCII in console."""
    print("+" + "-" * 32 + "+")
    for row in grid:
        row_str = ""
        for val in row:
            if val > 0.8:
                row_str += "# " # Occupied / obstacle / boundary
            elif val > 0.1:
                row_str += "? " # Sensor noise
            else:
                row_str += ". " # Empty space
        print("|" + row_str + "|")
    print("+" + "-" * 32 + "+")


if __name__ == "__main__":
    # Configure noise parameters
    noise = {
        "flip_prob": 0.04,
        "gaussian_std": 0.05,
        "drift_offset": (0, 0)
    }

    if DEMO_MODE:
        # ── Camera view definitions ────────────────────────────────────────────
        # Press 1/2/3/4 inside the PyBullet window to switch views live.
        _CAM_MODES = [
            # (label,       distance, pitch,  rotates, description)
            ("ORBITAL",      18.0,   -52,    True,   "3/4 Orbital — default, slow rotation"),
            ("TOP-DOWN",     22.0,   -89,    False,  "Overhead top-down bird's-eye view"),
            ("CINEMATIC",    14.0,   -28,    True,   "Low dramatic angle, fast rotation"),
            ("CLOSE-UP",      9.0,   -42,    False,  "Close chase — follows swarm tightly"),
        ]
        _cur_view     = 0
        _view_txt_id  = -1
        # Key codes in PyBullet for '1','2','3','4'
        _VIEW_KEYS    = {49: 0, 50: 1, 51: 2, 52: 3}

        print("=== Starting Infinite Interactive Flight-Stable Swarm Demo Loop ===")
        print("  Camera keys in PyBullet window:  1=Orbital  2=Top-Down  3=Cinematic  4=Close-Up")
        env = DroneSurveillanceEnv(render_mode="human", noise_params=noise, fixed_layout=True)
        obs, info = env.reset()

        print("Press Ctrl+C in terminal to stop.")

        t        = 0.0
        cam_yaw  = 35.0   # start from the same angle as reset()

        demo_thrusts = {agent_id: 0.3734 for agent_id in env.agents}

        try:
            while True:
                if not p.isConnected(env.client_id):
                    print("\nPyBullet visualizer window was closed by user.")
                    break

                # ── Generate hover actions ─────────────────────────────────────
                actions = {}
                for agent_id in env.agents:
                    demo_thrusts[agent_id] += random.uniform(-0.01, 0.01)
                    demo_thrusts[agent_id]  = np.clip(demo_thrusts[agent_id], 0.34, 0.42)
                    thrust = demo_thrusts[agent_id]
                    pitch  = random.uniform(-0.05, 0.05)
                    roll   = random.uniform(-0.05, 0.05)
                    yaw    = random.uniform(-0.02, 0.02)
                    actions[agent_id] = np.array([thrust, pitch, roll, yaw], dtype=np.float32)

                obs, rewards, terminateds, truncateds, infos = env.step(actions)

                # ── Drone centroid for camera target ───────────────────────────
                all_pos = [obs[a]['position'] for a in env.agents]
                cx = float(np.mean([pp[0] for pp in all_pos]))
                cy = float(np.mean([pp[1] for pp in all_pos]))
                cz = float(max(np.mean([pp[2] for pp in all_pos]), 2.0))

                # ── Keyboard: detect view-switch keypresses ────────────────────
                try:
                    keys = p.getKeyboardEvents(physicsClientId=env.client_id)
                    for kc, ks in keys.items():
                        if ks & p.KEY_WAS_TRIGGERED and kc in _VIEW_KEYS:
                            _cur_view = _VIEW_KEYS[kc]
                            cam_yaw   = 35.0
                            print(f"\n  [Camera] Switched to: {_CAM_MODES[_cur_view][0]}")
                except Exception:
                    pass

                # ── Apply camera for active view ───────────────────────────────
                _vm = _CAM_MODES[_cur_view]
                if _vm[3]:   # rotates flag
                    cam_yaw = (cam_yaw + 0.12) % 360.0
                p.resetDebugVisualizerCamera(
                    cameraDistance=_vm[1],
                    cameraPitch=_vm[2],
                    cameraYaw=cam_yaw,
                    cameraTargetPosition=[cx, cy, cz],
                    physicsClientId=env.client_id
                )

                # ── Update view-label text in 3D world ─────────────────────────
                view_label = (
                    f"[{_vm[0]}]  "
                    f"UAV-RED:{obs['drone_0']['position'][2]:.1f}m  "
                    f"UAV-BLU:{obs['drone_1']['position'][2]:.1f}m  "
                    f"UAV-GRN:{obs['drone_2']['position'][2]:.1f}m  "
                    f"W:{infos['drone_0']['wind_force_xy']:.1f}N  "
                    f"Keys: 1 2 3 4"
                )
                _lpos = [cx - 7.0, cy - 9.5, 0.4]
                if _view_txt_id != -1:
                    _view_txt_id = p.addUserDebugText(
                        text=view_label, textPosition=_lpos,
                        textColorRGB=[1.0, 1.0, 0.1], textSize=1.3,
                        replaceItemUniqueId=_view_txt_id,
                        physicsClientId=env.client_id
                    )
                else:
                    _view_txt_id = p.addUserDebugText(
                        text=view_label, textPosition=_lpos,
                        textColorRGB=[1.0, 1.0, 0.1], textSize=1.3,
                        physicsClientId=env.client_id
                    )

                # ── Terminal dashboard ─────────────────────────────────────────
                total_tracked = sum(infos[a]['tracked_targets'] for a in env.agents)
                print(
                    f"\r[{_vm[0]:9s}] "
                    f"RED:{obs['drone_0']['position'][2]:.1f}m "
                    f"BLU:{obs['drone_1']['position'][2]:.1f}m "
                    f"GRN:{obs['drone_2']['position'][2]:.1f}m "
                    f"| Wind:{infos['drone_0']['wind_force_xy']:.1f}N "
                    f"| Tracked:{total_tracked}",
                    end="", flush=True
                )

                t += 0.02
                time.sleep(0.02)

                # ── Reset on termination/truncation ───────────────────────────
                if any(terminateds.values()) or any(truncateds.values()):
                    obs, info = env.reset()
                    demo_thrusts = {a: 0.3734 for a in env.agents}
                    t = 0.0;  cam_yaw = 35.0
                    print()

        except p.error as pe:
            print(f"\nPyBullet connection ended: {pe}")
        except KeyboardInterrupt:
            print("\nDemo loop interrupted by user.")
        finally:
            try:
                env.close()
            except Exception:
                pass
            print("\n=== Swarm Demo finished successfully ===")
    else:
        # Run standard 10-step headless validation test
        print("=== Running Quick Headless Validation (DEMO_MODE=False) ===")
        env = DroneSurveillanceEnv(render_mode="headless", noise_params=noise)
        obs, info = env.reset()
        print("Environment successfully connected in DIRECT mode!")
        print(f"Number of agents: {len(env.agents)}")
        print(f"Building obstacle body count: {len(env.building_ids)}")
        print(f"Moving ground crowd boids count: {len(env.crowd.boids)}")
        
        # Showcase standard shapes of observations
        print("\nObservation Space shapes for 'drone_0':")
        for k, v in obs["drone_0"].items():
            print(f" - {k}: Shape={v.shape}, Dtype={v.dtype}")
            
        print("\nRunning a short 10-step simulation trial...")
        for step_num in range(1, 11):
            actions = {
                "drone_0": np.array([0.38, 0.05, 0.02, 0.01], dtype=np.float32),
                "drone_1": np.array([0.36, -0.02, 0.04, -0.01], dtype=np.float32),
                "drone_2": np.array([0.39, 0.01, -0.03, 0.02], dtype=np.float32)
            }
            obs, rewards, terminateds, truncateds, infos = env.step(actions)
            print(f"Step {step_num:02d} -> "
                  f"drone_0 Pos: {obs['drone_0']['position'].round(2)}, "
                  f"Reward: {rewards['drone_0']:.2f}, "
                  f"Tracked Boids: {infos['drone_0']['tracked_targets']}")
                  
        print("\nRendering local simulated SLAM 16x16 Occupancy Grid of drone_0 (with noise):")
        print_ascii_grid(obs["drone_0"]["occupancy_grid"])
        env.close()
        print("=== Headless validation successful and completed! ===")
