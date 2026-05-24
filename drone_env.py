import os
from collections import deque
import math
import random
import time
import numpy as np
import pybullet as p
import pybullet_data

# Global Demo Mode Toggle: Set to True to enable premium GUI mode, camera tracking, and 3D LiDAR rendering.
DEMO_MODE = True

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

    def __init__(self, render_mode="headless", noise_params=None, fixed_layout=False):
        super().__init__()
        self.render_mode = render_mode
        self.fixed_layout = fixed_layout
        self.noise_params = noise_params if noise_params is not None else {
            "flip_prob": 0.05,
            "gaussian_std": 0.05,
            "drift_offset": (0, 0)
        }
        
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
        self.max_lidar_range = 8.0 # meters (covers grid size of 16m total)
        
        # Multi-Agent observation space
        self.observation_spaces = {
            agent_id: spaces.Dict({
                "position": spaces.Box(low=-50.0, high=50.0, shape=(3,), dtype=np.float32),
                "velocity": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
                "lidar": spaces.Box(low=0.0, high=self.max_lidar_range, shape=(36,), dtype=np.float32),
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
        
        # Environment episode variables
        self.max_steps = 500
        self.current_step = 0
        self.drone_ids = {}
        self.building_ids = []
        self.crowd = None
        self.plane_id = None

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

    def _spawn_drones(self):
        """Assembles a high-fidelity 3D quadcopter model using p.createMultiBody."""
        self.drone_ids = {}
        for i, pos in enumerate(self.drone_init_positions):
            agent_id = f"drone_{i}"
            
            # Base shape: dark-grey box (GEOM_BOX, [0.15, 0.15, 0.05])
            base_col_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[0.15, 0.15, 0.05],
                physicsClientId=self.client_id
            )
            base_vis_id = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.15, 0.15, 0.05],
                rgbaColor=[0.2, 0.2, 0.2, 1.0], # Dark-grey box
                physicsClientId=self.client_id
            )

            # Rotor shape: 4 bright-blue cylindrical rotors (GEOM_CYLINDER, radius=0.08, height=0.02)
            rotor_col_id = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=0.08,
                height=0.02,
                physicsClientId=self.client_id
            )
            rotor_vis_id = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.08,
                length=0.02,
                rgbaColor=[0.0, 0.6, 1.0, 1.0], # Bright-blue cylindrical
                physicsClientId=self.client_id
            )

            link_masses = [0.1, 0.1, 0.1, 0.1]
            link_col_ids = [rotor_col_id] * 4
            link_vis_ids = [rotor_vis_id] * 4
            # Offset rotors to the corners as links with FIXED joints
            link_positions = [
                [0.15, 0.15, 0.03],
                [-0.15, 0.15, 0.03],
                [-0.15, -0.15, 0.03],
                [0.15, -0.15, 0.03]
            ]
            link_orientations = [[0, 0, 0, 1]] * 4
            link_parent_indices = [0] * 4
            link_joint_types = [p.JOINT_FIXED] * 4
            link_joint_axes = [[0, 0, 1]] * 4
            
            drone_id = p.createMultiBody(
                baseMass=1.0,
                baseCollisionShapeIndex=base_col_id,
                baseVisualShapeIndex=base_vis_id,
                basePosition=pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                linkMasses=link_masses,
                linkCollisionShapeIndices=link_col_ids,
                linkVisualShapeIndices=link_vis_ids,
                linkPositions=link_positions,
                linkOrientations=link_orientations,
                linkInertialFramePositions=[[0, 0, 0]] * 4,
                linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
                linkParentIndices=link_parent_indices,
                linkJointTypes=link_joint_types,
                linkJointAxis=link_joint_axes,
                physicsClientId=self.client_id
            )
            
            self.drone_ids[agent_id] = drone_id

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
        """Resets the environment and spawns all assets."""
        super().reset(seed=seed)
        self.current_step = 0
        
        # Initialize battery state to 100.0% for all agents
        self.battery = {agent_id: 100.0 for agent_id in self.agents}
        
        # Initialize HUD debug text item IDs
        self.hud_text_ids = {agent_id: -1 for agent_id in self.agents}
        
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
            # Solid light-grey floor [0.9, 0.9, 0.9, 1.0] and clear default texture
            p.changeVisualShape(self.plane_id, -1, rgbaColor=[0.9, 0.9, 0.9, 1.0], textureUniqueId=-1, physicsClientId=self.client_id)

        # Spawn static urban buildings (Phase 1.1)
        self._spawn_buildings()
        
        # Spawn ground crowd (Phase 1.1 - moving dots)
        if self.fixed_layout:
            np.random.seed(42)
            random.seed(42)
        self.crowd = RandomCrowd(num_boids=12, boundary=8.0, physics_client_id=self.client_id, building_specs=self.building_specs)
        
        # Spawn UAVs (Phase 1.1)
        self._spawn_drones()
        
        # Let PyBullet settle
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.client_id)
            
        # Re-initialize/clear debug lines for all agents on reset
        if DEMO_MODE:
            p.removeAllUserDebugItems(physicsClientId=self.client_id)
        
        # Set camera target on reset
        if DEMO_MODE:
            p.resetDebugVisualizerCamera(
                cameraDistance=12.0,
                cameraPitch=-35,
                cameraYaw=45,
                cameraTargetPosition=[0.0, 0.0, 1.0],
                physicsClientId=self.client_id
            )
        
        obs = {}
        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
            vel, _ = p.getBaseVelocity(drone_id, physicsClientId=self.client_id)
            
            lidar, grid = self._get_drone_sensors(agent_id)
            obs[agent_id] = {
                "position": np.array(pos, dtype=np.float32),
                "velocity": np.array(vel, dtype=np.float32),
                "lidar": lidar,
                "occupancy_grid": grid
            }
            
        info = {agent_id: {} for agent_id in self.agents}
        return obs, info

    def step(self, actions):
        """Steps the simulation environment."""
        self.current_step += 1
        
        # 1. Apply multi-agent controls, drain battery, apply lateral wind
        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            
            if agent_id in actions:
                action = actions[agent_id]
                self._apply_flight_control(agent_id, action)
                # Battery Drain
                thrust_action = action[0]
                drain = 0.01 + 0.03 * abs(thrust_action)
                self.battery[agent_id] = float(np.clip(self.battery[agent_id] - drain, 0.0, 100.0))
            else:
                drain = 0.01
                self.battery[agent_id] = float(np.clip(self.battery[agent_id] - drain, 0.0, 100.0))
                
            # Apply random lateral wind force [-0.15, 0.15] in global X/Y
            wind_x = random.uniform(-0.15, 0.15)
            wind_y = random.uniform(-0.15, 0.15)
            p.applyExternalForce(
                drone_id,
                -1,
                [wind_x, wind_y, 0.0],
                [0.0, 0.0, 0.0],
                flags=p.WORLD_FRAME,
                physicsClientId=self.client_id
            )
                
        # 2. Update boids crowd positions
        self.crowd.update()
        
        # 3. Simulate step in PyBullet physics
        p.stepSimulation(physicsClientId=self.client_id)
        
        # 4. Generate outputs
        obs = {}
        rewards = {}
        terminateds = {}
        truncateds = {}
        infos = {}
        
        boid_positions = [b['pos'] for b in self.crowd.boids]
        
        for agent_id in self.agents:
            drone_id = self.drone_ids[agent_id]
            pos, orientation = p.getBasePositionAndOrientation(drone_id, physicsClientId=self.client_id)
            vel, _ = p.getBaseVelocity(drone_id, physicsClientId=self.client_id)
            pos_np = np.array(pos)
            
            # Holographic HUD labels
            if DEMO_MODE:
                label_text = f"UAV-{agent_id.split('_')[1]} | Bat: {int(self.battery[agent_id])}% | Alt: {pos_np[2]:.1f}m"
                text_pos = [pos_np[0], pos_np[1], pos_np[2] + 0.5]
                if self.hud_text_ids.get(agent_id, -1) != -1:
                    self.hud_text_ids[agent_id] = p.addUserDebugText(
                        text=label_text,
                        textPosition=text_pos,
                        textColorRGB=[0.0, 1.0, 1.0],
                        textSize=1.2,
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
                "position": np.array(pos, dtype=np.float32),
                "velocity": np.array(vel, dtype=np.float32),
                "lidar": lidar,
                "occupancy_grid": grid
            }
            
            # A. Tracking Reward (downward 45-degree cone camera FOV: d <= Z)
            tracked_count = 0
            alt = pos_np[2]
            for b_pos in boid_positions:
                dist_2d = np.linalg.norm(pos_np[:2] - b_pos)
                if dist_2d <= alt:
                    tracked_count += 1
            tracking_reward = 0.6 * tracked_count
            
            # B. Collision Detection (Minimum Distance Margin of 1 to 5 meters)
            collision_penalty = 0.0
            
            # Collision with buildings
            for b_id in self.building_ids:
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id,
                    bodyB=b_id,
                    distance=1.0, # 1.0 meter threshold
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 6.0
                    
            # Collision with other drones
            for other_id in self.agents:
                if other_id == agent_id:
                    continue
                other_drone_id = self.drone_ids[other_id]
                closest_pts = p.getClosestPoints(
                    bodyA=drone_id,
                    bodyB=other_drone_id,
                    distance=1.0,
                    physicsClientId=self.client_id
                )
                if len(closest_pts) > 0:
                    collision_penalty -= 4.0
                    
            # C. Out-of-Bounds checking
            out_of_bounds = False
            if (abs(pos_np[0]) > 12.0 or abs(pos_np[1]) > 12.0 or pos_np[2] < 0.5 or pos_np[2] > 6.0):
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
                "tracked_targets": tracked_count,
                "out_of_bounds": out_of_bounds,
                "collision_penalty": collision_penalty
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
        # Phase 3 Real-time Terminal Dashboard & Interactive Flight-Stable Demo
        print("=== Starting Infinite Interactive Flight-Stable Swarm Demo Loop ===")
        # Initialize the environment in GUI mode for the demo (render_mode="human")
        env = DroneSurveillanceEnv(render_mode="human", noise_params=noise, fixed_layout=True)
        obs, info = env.reset()
        
        print("Press Ctrl+C in terminal to stop.")
        
        t = 0.0
        cam_yaw = 0.0
        
        # Initialize random walk states for Thrust
        demo_thrusts = {agent_id: 0.3734 for agent_id in env.agents}
        
        try:
            while True:
                # Break loop if user closed PyBullet GUI window
                if not p.isConnected(env.client_id):
                    print("\nPyBullet visualizer window was closed by user.")
                    break

                # Generate flight-stable but erratic physical actions
                actions = {}
                for agent_id in env.agents:
                    # Random walk on Thrust around hover (0.3734)
                    demo_thrusts[agent_id] += random.uniform(-0.01, 0.01)
                    demo_thrusts[agent_id] = np.clip(demo_thrusts[agent_id], 0.34, 0.42)
                    
                    thrust = demo_thrusts[agent_id]
                    # Small random pitch, roll, and yaw perturbations
                    pitch = random.uniform(-0.05, 0.05)
                    roll = random.uniform(-0.05, 0.05)
                    yaw = random.uniform(-0.02, 0.02)
                    
                    actions[agent_id] = np.array([thrust, pitch, roll, yaw], dtype=np.float32)
                
                obs, rewards, terminateds, truncateds, infos = env.step(actions)
                
                # Print live status dashboard in-place using carriage return \r
                dashboard = (
                    f"\rUAV-0 [Bat: {int(env.battery['drone_0'])}% | Alt: {obs['drone_0']['position'][2]:.2f}m | Tracked: {infos['drone_0']['tracked_targets']}] "
                    f"UAV-1 [Bat: {int(env.battery['drone_1'])}% | Alt: {obs['drone_1']['position'][2]:.2f}m | Tracked: {infos['drone_1']['tracked_targets']}] "
                    f"UAV-2 [Bat: {int(env.battery['drone_2'])}% | Alt: {obs['drone_2']['position'][2]:.2f}m | Tracked: {infos['drone_2']['tracked_targets']}]"
                )
                print(dashboard, end="", flush=True)
                
                # Dynamic camera tracking from the infinite demo loop (hands-free orbital camera)
                p.resetDebugVisualizerCamera(
                    cameraDistance=12.0,
                    cameraPitch=-35,
                    cameraYaw=cam_yaw,
                    cameraTargetPosition=[0.0, 0.0, 1.0],
                    physicsClientId=env.client_id
                )
                cam_yaw = (cam_yaw + 0.2) % 360.0
                
                # If any drone is terminated (e.g. out of bounds) or truncated, reset
                any_terminated = any(terminateds.values())
                any_truncated = any(truncateds.values())
                if any_terminated or any_truncated:
                    obs, info = env.reset()
                    demo_thrusts = {agent_id: 0.3734 for agent_id in env.agents}
                    t = 0.0
                    cam_yaw = 0.0
                    print()  # print newline on reset
                
                t += 0.02
                time.sleep(0.02)  # Control frame rate
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
