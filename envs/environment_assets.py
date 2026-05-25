"""
environment_assets.py — Modular Environmental Asset System
===========================================================
Provides reusable asset-spawning classes for the Multi-UAV Surveillance
simulation (STIRS-2025). All classes are pure PyBullet (no external deps).

Classes:
    WindSystem          — Realistic directional wind with turbulence and height scaling
    TreeSpawner         — Cylindrical trunks + spherical foliage canopies
    BuildingSpawner     — Tall urban office/commercial buildings (5–20m)
    HouseSpawner        — Small residential houses (3–5m)
    ElectricPoleSpawner — 6m utility poles with horizontal cross-arms
    BirdFlock           — Kinematic flying birds with real-world random altitude cycles
    RoadAndParkSpawner  — Gray road planes + green park zones (GUI-mode visual enrichment)

Usage in drone_env.py::

    from envs.environment_assets import (
        WindSystem, TreeSpawner, BuildingSpawner,
        HouseSpawner, ElectricPoleSpawner, BirdFlock, RoadAndParkSpawner
    )
"""

import math
import random
import numpy as np
import pybullet as p


# ---------------------------------------------------------------------------
# 1. WindSystem
# ---------------------------------------------------------------------------

class WindSystem:
    """
    Simulates realistic environmental wind forces for drone physics.

    Wind behaviour modelled:
    - A base wind direction that slowly rotates (like real-world weather patterns)
    - Height-dependent speed: wind is stronger at altitude (logarithmic profile)
    - Per-step turbulence: Gaussian gusts layered on the base flow
    - A slow sinusoidal variation in base speed (gusting cycles every ~5 seconds)

    Usage::

        wind = WindSystem(base_speed=2.0)
        fx, fy, fz = wind.get_force(drone_position, step_count)
        p.applyExternalForce(drone_id, -1, [fx, fy, fz], ...)
    """

    def __init__(self, base_speed: float = 2.0, seed: int = None):
        """
        Args:
            base_speed: Mean wind speed in m/s (translates to Newtons applied to 1 kg drone).
            seed: Optional random seed for reproducible wind fields.
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        self.base_speed = base_speed           # m/s (= N on 1 kg body)
        self.direction_angle = random.uniform(0, 2 * math.pi)  # radians, global XY
        self.turbulence_std = base_speed * 0.35  # 35% of base speed as turbulence std
        self.gust_phase = random.uniform(0, 2 * math.pi)       # phase offset for sinusoidal gusts

        # Slow drift rate of wind direction (rad per step)
        self._drift_rate = 0.003  # ~1.7° per step — barely perceptible rotation

    def get_force(self, position: list, step: int) -> tuple:
        """
        Computes the wind force vector at a given world position and time step.

        The logarithmic height profile: v(z) = v_ref * ln(z/z0) / ln(z_ref/z0)
        Here simplified to a smooth ramp: multiplier = 1.0 + 0.08 * clamp(z - 0.5, 0, 10)

        Args:
            position: [x, y, z] world position of the drone.
            step:     Current simulation step number (used for gust timing).

        Returns:
            (fx, fy, fz) force tuple in Newtons (world frame).
        """
        z = max(float(position[2]), 0.1)

        # 1. Height multiplier (stronger wind aloft)
        height_mult = 1.0 + 0.08 * min(max(z - 0.5, 0.0), 10.0)

        # 2. Slow sinusoidal gust cycle (period ~250 steps ≈ 5 seconds at 50 Hz)
        gust_mult = 1.0 + 0.25 * math.sin(step * 0.025 + self.gust_phase)

        # 3. Effective base speed at this altitude
        effective_speed = self.base_speed * height_mult * gust_mult

        # 4. Slowly rotate wind direction
        self.direction_angle += self._drift_rate + random.gauss(0, 0.001)

        # 5. Base force in XY plane
        base_fx = effective_speed * math.cos(self.direction_angle)
        base_fy = effective_speed * math.sin(self.direction_angle)

        # 6. Add Gaussian turbulence (random gusts)
        turb_x = random.gauss(0.0, self.turbulence_std * height_mult)
        turb_y = random.gauss(0.0, self.turbulence_std * height_mult)
        turb_z = random.gauss(0.0, self.turbulence_std * 0.15)  # Vertical gusts much smaller

        return (base_fx + turb_x, base_fy + turb_y, turb_z)

    def get_direction_degrees(self) -> float:
        """Returns current wind direction in compass degrees (0° = East, 90° = North)."""
        return math.degrees(self.direction_angle) % 360.0


# ---------------------------------------------------------------------------
# 2. TreeSpawner
# ---------------------------------------------------------------------------

class TreeSpawner:
    """
    Spawns realistic trees using PyBullet primitives:
      - Cylindrical brown trunk (GEOM_CYLINDER)
      - Spherical green foliage canopy (GEOM_SPHERE)

    Trees are scattered in clusters to mimic park/roadside planting patterns.
    In headless mode trees still spawn because their trunks block LiDAR rays.
    """

    # Brown trunk colours (bark tones)
    TRUNK_COLORS = [
        [0.42, 0.27, 0.14, 1.0],   # warm brown
        [0.35, 0.22, 0.10, 1.0],   # dark bark
        [0.50, 0.32, 0.18, 1.0],   # reddish bark
    ]

    # Green foliage colours (seasonal variation)
    FOLIAGE_COLORS = [
        [0.13, 0.55, 0.13, 1.0],   # forest green
        [0.20, 0.65, 0.20, 1.0],   # bright green
        [0.10, 0.42, 0.10, 1.0],   # deep green
        [0.30, 0.60, 0.10, 1.0],   # lime-ish
        [0.55, 0.50, 0.10, 1.0],   # autumn yellow-green
    ]

    def __init__(self, physics_client_id: int, exclusion_zones: list = None):
        """
        Args:
            physics_client_id: PyBullet physics client ID.
            exclusion_zones:   List of [x, y] positions where trees must NOT spawn
                               (e.g., drone initial positions, building centres).
        """
        self.client_id = physics_client_id
        self.exclusion_zones = exclusion_zones or []
        self.tree_ids = []         # (trunk_id, foliage_id) pairs
        self.all_body_ids = []     # flat list of all PyBullet body IDs created

    def _is_excluded(self, x: float, y: float, min_dist: float = 1.5) -> bool:
        """Returns True if (x,y) is too close to any exclusion zone."""
        for zone in self.exclusion_zones:
            if math.sqrt((x - zone[0])**2 + (y - zone[1])**2) < min_dist:
                return True
        return False

    def _try_place(self, arena: float = 9.0) -> tuple:
        """Attempts to find a valid (x, y) position with up to 50 retries."""
        for _ in range(50):
            x = random.uniform(-arena, arena)
            y = random.uniform(-arena, arena)
            if not self._is_excluded(x, y):
                return x, y
        return None, None

    def spawn(self, num_trees: int = 20, arena: float = 9.0) -> list:
        """
        Spawns ``num_trees`` trees scattered across the arena.

        Trees are placed in loose clusters (3–5 trees per cluster centre)
        to mimic natural planting patterns.

        Args:
            num_trees: Total number of trees to spawn.
            arena:     Half-width of the square arena boundary.

        Returns:
            List of all PyBullet body IDs created (trunks + foliage combined).
        """
        self.tree_ids.clear()
        self.all_body_ids.clear()

        # Generate cluster centres first
        num_clusters = max(1, num_trees // 4)
        cluster_centres = []
        for _ in range(num_clusters):
            cx, cy = self._try_place(arena)
            if cx is not None:
                cluster_centres.append((cx, cy))

        if not cluster_centres:
            return []

        trees_placed = 0
        while trees_placed < num_trees:
            # Pick a cluster centre with a small offset
            cx, cy = random.choice(cluster_centres)
            spread = random.uniform(0.8, 2.5)
            x = cx + random.gauss(0, spread)
            y = cy + random.gauss(0, spread)
            x = max(-arena, min(arena, x))
            y = max(-arena, min(arena, y))

            if self._is_excluded(x, y, min_dist=1.2):
                continue

            # Randomize tree dimensions
            trunk_radius = random.uniform(0.06, 0.14)
            trunk_height = random.uniform(1.2, 3.5)
            foliage_radius = random.uniform(0.6, 1.4)
            foliage_offset = trunk_height * 0.6  # foliage sits above mid-trunk

            trunk_col = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=trunk_radius,
                height=trunk_height,
                physicsClientId=self.client_id
            )
            trunk_vis = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=trunk_radius,
                length=trunk_height,
                rgbaColor=random.choice(self.TRUNK_COLORS),
                physicsClientId=self.client_id
            )
            trunk_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=trunk_col,
                baseVisualShapeIndex=trunk_vis,
                basePosition=[x, y, trunk_height / 2.0],
                physicsClientId=self.client_id
            )

            foliage_col = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=foliage_radius,
                physicsClientId=self.client_id
            )
            foliage_vis = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=foliage_radius,
                rgbaColor=random.choice(self.FOLIAGE_COLORS),
                physicsClientId=self.client_id
            )
            foliage_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=foliage_col,
                baseVisualShapeIndex=foliage_vis,
                basePosition=[x, y, trunk_height + foliage_offset],
                physicsClientId=self.client_id
            )

            self.tree_ids.append((trunk_id, foliage_id))
            self.all_body_ids.extend([trunk_id, foliage_id])

            # Register tree position as its own exclusion to avoid overlap
            self.exclusion_zones.append([x, y])
            trees_placed += 1

        return self.all_body_ids


# ---------------------------------------------------------------------------
# 3. BuildingSpawner
# ---------------------------------------------------------------------------

class BuildingSpawner:
    """
    Spawns tall urban office/commercial buildings (5–20m) in cluster formations,
    mimicking a downtown skyline. Heights and colors are varied for visual richness.

    Note: This SUPPLEMENTS the existing ``_spawn_buildings()`` in DroneSurveillanceEnv.
    The existing method spawns mid-size buildings (2–8m). This spawner adds taller
    landmark structures in distinct urban clusters.
    """

    # Urban building colour palette (glass towers, concrete, brick)
    COLORS = [
        [0.45, 0.55, 0.70, 1.0],   # steel blue (glass tower)
        [0.60, 0.60, 0.60, 1.0],   # concrete grey
        [0.70, 0.55, 0.40, 1.0],   # sandstone beige
        [0.35, 0.35, 0.45, 1.0],   # dark charcoal glass
        [0.65, 0.70, 0.65, 1.0],   # pale sage concrete
        [0.80, 0.75, 0.65, 1.0],   # warm limestone
    ]

    def __init__(self, physics_client_id: int, exclusion_zones: list = None):
        self.client_id = physics_client_id
        self.exclusion_zones = list(exclusion_zones or [])
        self.building_ids = []

    def _is_excluded(self, x: float, y: float, radius: float, min_gap: float = 1.0) -> bool:
        for zone in self.exclusion_zones:
            dist = math.sqrt((x - zone[0])**2 + (y - zone[1])**2)
            if dist < radius + min_gap:
                return True
        return False

    def spawn(self, num_buildings: int = 6, arena: float = 9.0) -> list:
        """
        Spawns tall urban buildings in 2–3 cluster zones.

        Args:
            num_buildings: Number of tall buildings to spawn (5–20m).
            arena:         Half-width of the square arena boundary.

        Returns:
            List of PyBullet body IDs created.
        """
        self.building_ids.clear()

        # Define 2 urban cluster centres
        cluster_centres = [
            (random.uniform(-7.0, -3.0), random.uniform(-7.0, -3.0)),
            (random.uniform(3.0, 7.0),  random.uniform(3.0, 7.0)),
        ]

        placed = 0
        max_attempts = num_buildings * 15

        for attempt in range(max_attempts):
            if placed >= num_buildings:
                break

            cx, cy = random.choice(cluster_centres)
            x = cx + random.gauss(0, 2.0)
            y = cy + random.gauss(0, 2.0)
            x = max(-arena, min(arena, x))
            y = max(-arena, min(arena, y))

            half_x = random.uniform(0.6, 1.8)
            half_y = random.uniform(0.6, 1.8)
            # Tall buildings: 5–10m total height → half_z in [2.5, 5.0]
            # (was up to 20m which trapped the camera inside a canyon)
            half_z = random.uniform(2.5, 5.0)

            if self._is_excluded(x, y, max(half_x, half_y)):
                continue

            col = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[half_x, half_y, half_z],
                physicsClientId=self.client_id
            )
            color = random.choice(self.COLORS)
            vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[half_x, half_y, half_z],
                rgbaColor=color,
                physicsClientId=self.client_id
            )
            bid = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=[x, y, half_z],
                physicsClientId=self.client_id
            )
            self.building_ids.append(bid)
            self.exclusion_zones.append([x, y])
            placed += 1

        return self.building_ids


# ---------------------------------------------------------------------------
# 4. HouseSpawner
# ---------------------------------------------------------------------------

class HouseSpawner:
    """
    Spawns small residential houses (3–5m) in a suburban layout.
    Houses use warm residential colours and are scattered in a sub-zone
    away from the tall building clusters.
    """

    # Residential colour palette (warm suburban tones)
    COLORS = [
        [0.85, 0.70, 0.55, 1.0],   # sandy cream
        [0.75, 0.55, 0.45, 1.0],   # terracotta
        [0.60, 0.75, 0.65, 1.0],   # mint green
        [0.80, 0.80, 0.75, 1.0],   # off-white
        [0.65, 0.55, 0.70, 1.0],   # dusty lavender
        [0.90, 0.80, 0.65, 1.0],   # warm beige
    ]

    # Darker roof shades
    ROOF_COLORS = [
        [0.35, 0.22, 0.15, 1.0],   # brown shingle
        [0.25, 0.25, 0.35, 1.0],   # slate blue
        [0.45, 0.35, 0.25, 1.0],   # cedar
        [0.20, 0.20, 0.20, 1.0],   # charcoal
    ]

    def __init__(self, physics_client_id: int, exclusion_zones: list = None):
        self.client_id = physics_client_id
        self.exclusion_zones = list(exclusion_zones or [])
        self.house_ids = []

    def _is_excluded(self, x: float, y: float, radius: float = 1.0) -> bool:
        for zone in self.exclusion_zones:
            if math.sqrt((x - zone[0])**2 + (y - zone[1])**2) < radius:
                return True
        return False

    def spawn(self, num_houses: int = 10, arena: float = 9.0) -> list:
        """
        Spawns small houses (body + triangular roof prism) in a residential cluster.

        Args:
            num_houses: Number of houses to spawn.
            arena:      Half-width of the arena boundary.

        Returns:
            List of PyBullet body IDs created.
        """
        self.house_ids.clear()

        placed = 0
        max_attempts = num_houses * 20

        for _ in range(max_attempts):
            if placed >= num_houses:
                break

            x = random.uniform(-arena * 0.7, arena * 0.7)
            y = random.uniform(-arena * 0.7, arena * 0.7)

            half_x = random.uniform(0.4, 0.9)
            half_y = random.uniform(0.4, 0.9)
            # Houses: 3–5m total height → half_z in [1.5, 2.5]
            half_z = random.uniform(1.5, 2.5)

            if self._is_excluded(x, y, radius=max(half_x, half_y) + 0.8):
                continue

            # Main body
            body_col = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[half_x, half_y, half_z],
                physicsClientId=self.client_id
            )
            body_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[half_x, half_y, half_z],
                rgbaColor=random.choice(self.COLORS),
                physicsClientId=self.client_id
            )
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=body_col,
                baseVisualShapeIndex=body_vis,
                basePosition=[x, y, half_z],
                physicsClientId=self.client_id
            )
            self.house_ids.append(body_id)

            # Roof — flattened box sitting on top of the house body
            roof_half_x = half_x * 1.05
            roof_half_y = half_y * 1.05
            roof_half_z = half_z * 0.30  # shallow pitched roof approximation
            roof_col = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[roof_half_x, roof_half_y, roof_half_z],
                physicsClientId=self.client_id
            )
            roof_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[roof_half_x, roof_half_y, roof_half_z],
                rgbaColor=random.choice(self.ROOF_COLORS),
                physicsClientId=self.client_id
            )
            roof_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=roof_col,
                baseVisualShapeIndex=roof_vis,
                basePosition=[x, y, half_z * 2.0 + roof_half_z],
                physicsClientId=self.client_id
            )
            self.house_ids.append(roof_id)

            self.exclusion_zones.append([x, y])
            placed += 1

        return self.house_ids


# ---------------------------------------------------------------------------
# 5. ElectricPoleSpawner
# ---------------------------------------------------------------------------

class ElectricPoleSpawner:
    """
    Spawns utility electric poles:
      - Thin 6m cylindrical shaft (grey)
      - Two horizontal cross-arm boxes near the top

    Poles are placed along virtual road edges at regular intervals.
    """

    POLE_COLOR = [0.50, 0.50, 0.52, 1.0]      # galvanized steel grey
    CROSSARM_COLOR = [0.40, 0.35, 0.30, 1.0]   # dark treated wood

    def __init__(self, physics_client_id: int, exclusion_zones: list = None):
        self.client_id = physics_client_id
        self.exclusion_zones = list(exclusion_zones or [])
        self.pole_ids = []

    def _is_excluded(self, x: float, y: float) -> bool:
        for zone in self.exclusion_zones:
            if math.sqrt((x - zone[0])**2 + (y - zone[1])**2) < 1.5:
                return True
        return False

    def _spawn_single_pole(self, x: float, y: float):
        """Spawns one complete pole (shaft + cross-arms) at (x, y)."""
        pole_height = 6.0
        shaft_radius = 0.05

        # Shaft
        shaft_col = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=shaft_radius,
            height=pole_height,
            physicsClientId=self.client_id
        )
        shaft_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=shaft_radius,
            length=pole_height,
            rgbaColor=self.POLE_COLOR,
            physicsClientId=self.client_id
        )
        shaft_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=shaft_col,
            baseVisualShapeIndex=shaft_vis,
            basePosition=[x, y, pole_height / 2.0],
            physicsClientId=self.client_id
        )
        self.pole_ids.append(shaft_id)

        # Cross-arm (oriented along Y axis — perpendicular to road)
        arm_half = [0.08, 0.70, 0.04]
        for arm_y_offset in [-0.3, 0.3]:
            arm_col = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=arm_half,
                physicsClientId=self.client_id
            )
            arm_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=arm_half,
                rgbaColor=self.CROSSARM_COLOR,
                physicsClientId=self.client_id
            )
            arm_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=arm_col,
                baseVisualShapeIndex=arm_vis,
                basePosition=[x, y + arm_y_offset, pole_height - 0.4],
                physicsClientId=self.client_id
            )
            self.pole_ids.append(arm_id)

    def spawn(self, arena: float = 9.0) -> list:
        """
        Places poles along two virtual road lines:
          - Road 1: runs along X axis at Y ≈ 0 (horizontal road)
          - Road 2: runs along Y axis at X ≈ 0 (vertical road)

        Poles spaced every 3.5m, skipped near exclusion zones.

        Args:
            arena: Half-width of the arena boundary.

        Returns:
            List of PyBullet body IDs created.
        """
        self.pole_ids.clear()

        spacing = 6.0    # wider spacing = fewer poles = cleaner view (was 3.5m)
        road_offset = 1.2  # distance from road centre to pole

        # Road 1 — poles on both sides of horizontal road (Y ≈ 0)
        x_positions = np.arange(-arena + 1.0, arena, spacing)
        for x in x_positions:
            for y_side in [-road_offset, road_offset]:
                if not self._is_excluded(float(x), float(y_side)):
                    self._spawn_single_pole(float(x), float(y_side))

        # Road 2 — poles on both sides of vertical road (X ≈ 0)
        y_positions = np.arange(-arena + 1.0, arena, spacing)
        for y in y_positions:
            for x_side in [-road_offset, road_offset]:
                if not self._is_excluded(float(x_side), float(y)):
                    self._spawn_single_pole(float(x_side), float(y))

        return self.pole_ids


# ---------------------------------------------------------------------------
# 6. BirdFlock
# ---------------------------------------------------------------------------

class BirdFlock:
    """
    Simulates a flock of 5–10 birds as kinematic PyBullet bodies.

    Real-world altitude behaviour:
    - Each bird has its own random altitude "target" that changes over time
    - Birds cycle through soaring (climbing), gliding (level), and diving phases
    - Phase transitions happen at random intervals (mimicking thermal soaring)
    - Flocking forces: separation (avoid crowding), alignment (match neighbours),
      cohesion (loosely stay together)
    - XY wandering: each bird has a slow-turning heading that curves lazily

    Collision bodies are enabled — drones can detect birds via getClosestPoints().
    """

    # Vivid bird colours — large enough to spot easily from 28m camera distance
    BIRD_COLORS = [
        [1.00, 0.45, 0.00, 1.0],   # vivid orange (most visible)
        [1.00, 0.85, 0.00, 1.0],   # bright yellow
        [1.00, 0.20, 0.20, 1.0],   # hot red
        [0.90, 0.00, 0.90, 1.0],   # magenta
    ]

    def __init__(self, physics_client_id: int, num_birds: int = 7):
        """
        Args:
            physics_client_id: PyBullet client ID.
            num_birds:         Number of birds to spawn (5–10 recommended).
        """
        self.client_id = physics_client_id
        self.num_birds = max(3, min(num_birds, 12))
        self.birds = []     # List of bird state dicts
        self.body_ids = []  # All PyBullet body IDs

        # Shared collision/visual shapes — 0.45m radius = clearly visible from 28m camera
        self._col_shape = p.createCollisionShape(
            p.GEOM_SPHERE, radius=0.45, physicsClientId=self.client_id
        )

    def spawn(self, arena: float = 8.0) -> list:
        """
        Creates bird bodies and initialises their state.

        Each bird state dict contains::

            {
                'id':          PyBullet body ID,
                'pos':         np.array([x, y, z]),
                'vel':         np.array([vx, vy, vz]),
                'heading':     float  (radians, XY plane),
                'speed':       float  (m/s, horizontal),
                'alt_target':  float  (m, current altitude goal),
                'alt_min':     float  (m, personal minimum altitude),
                'alt_max':     float  (m, personal maximum altitude),
                'phase':       str    ('soar' | 'glide' | 'dive'),
                'phase_timer': int    (steps remaining in current phase),
            }

        Args:
            arena: Half-width of the spawn area.

        Returns:
            List of PyBullet body IDs for all birds.
        """
        self.birds.clear()
        self.body_ids.clear()

        for i in range(self.num_birds):
            color = random.choice(self.BIRD_COLORS)
            vis_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.45,     # 2.5x larger than before — visible from above
                rgbaColor=color,
                physicsClientId=self.client_id
            )

            # Bird altitude band: 1.5–5m so they overlap with drone hover altitude of ~2m
            alt_min = random.uniform(1.5, 3.0)
            alt_max = alt_min + random.uniform(1.5, 3.5)
            start_z = random.uniform(alt_min, alt_max)

            x = random.uniform(-arena, arena)
            y = random.uniform(-arena, arena)

            body_id = p.createMultiBody(
                baseMass=0,  # kinematic — we move it manually
                baseCollisionShapeIndex=self._col_shape,
                baseVisualShapeIndex=vis_shape,
                basePosition=[x, y, start_z],
                physicsClientId=self.client_id
            )

            self.body_ids.append(body_id)
            self.birds.append({
                'id':          body_id,
                'pos':         np.array([x, y, start_z], dtype=np.float64),
                'vel':         np.zeros(3, dtype=np.float64),
                'heading':     random.uniform(0, 2 * math.pi),
                'speed':       random.uniform(1.2, 3.5),       # m/s horizontal
                'alt_target':  start_z,
                'alt_min':     alt_min,
                'alt_max':     alt_max,
                'phase':       random.choice(['soar', 'glide', 'dive']),
                'phase_timer': random.randint(30, 150),         # steps in this phase
            })

        return self.body_ids

    def _next_phase(self, bird: dict):
        """Transitions a bird to a new altitude phase randomly."""
        current = bird['phase']
        # Natural transitions: soar → glide → dive → soar (with some randomness)
        transitions = {
            'soar':  ['glide', 'glide', 'soar'],     # usually transitions to glide
            'glide': ['soar', 'dive', 'glide'],       # can go either way
            'dive':  ['glide', 'soar', 'glide'],      # usually recovers to glide
        }
        bird['phase'] = random.choice(transitions.get(current, ['glide']))
        bird['phase_timer'] = random.randint(40, 200)

        # Set new altitude target based on phase
        if bird['phase'] == 'soar':
            bird['alt_target'] = random.uniform(
                (bird['alt_min'] + bird['alt_max']) / 2.0, bird['alt_max']
            )
        elif bird['phase'] == 'dive':
            bird['alt_target'] = random.uniform(
                bird['alt_min'], (bird['alt_min'] + bird['alt_max']) / 2.0
            )
        else:  # glide — hold somewhere in the middle band
            bird['alt_target'] = random.uniform(
                bird['alt_min'] + 0.3, bird['alt_max'] - 0.3
            )

    def update(self, dt: float = 0.02, arena: float = 9.5):
        """
        Advances all bird positions by one timestep.

        Applies:
        1. Phase-based vertical movement (soar/glide/dive)
        2. Slow XY heading drift (lazy curving flight path)
        3. Separation flocking force (avoid crowding between birds)
        4. Arena boundary reflection (birds stay inside arena)
        5. PyBullet kinematic body position update

        Args:
            dt:    Timestep in seconds (should match env step rate).
            arena: Half-width of the arena; birds reflect off the boundary.
        """
        positions = np.array([b['pos'] for b in self.birds])

        for i, bird in enumerate(self.birds):
            # --- Phase timer countdown ---
            bird['phase_timer'] -= 1
            if bird['phase_timer'] <= 0:
                self._next_phase(bird)

            # --- Vertical movement toward altitude target ---
            z_error = bird['alt_target'] - bird['pos'][2]
            # Vertical speed proportional to error, capped at ±0.8 m/s
            vz = np.clip(z_error * 0.15, -0.8, 0.8)

            # --- Heading drift (lazy turns, ±3° per step) ---
            bird['heading'] += random.gauss(0, 0.05)

            # --- Separation force: steer away from nearby birds ---
            sep_x, sep_y = 0.0, 0.0
            for j, other_pos in enumerate(positions):
                if i == j:
                    continue
                diff = bird['pos'][:2] - other_pos[:2]
                dist = np.linalg.norm(diff)
                if 0.01 < dist < 2.5:  # separation radius 2.5m
                    # Force inversely proportional to distance
                    sep_x += diff[0] / (dist * dist)
                    sep_y += diff[1] / (dist * dist)

            sep_mag = math.sqrt(sep_x**2 + sep_y**2)
            if sep_mag > 0:
                sep_x /= sep_mag
                sep_y /= sep_mag

            # --- Obstacle Avoidance (Buildings, Trees, Houses, etc.) ---
            obs_x, obs_y, obs_z = 0.0, 0.0, 0.0
            # Check a 4m bounding box around the bird for early warning
            x, y, z = bird['pos']
            aabbMin = [x - 4.0, y - 4.0, z - 4.0]
            aabbMax = [x + 4.0, y + 4.0, z + 4.0]
            overlaps = p.getOverlappingObjects(aabbMin, aabbMax, physicsClientId=self.client_id)
            if overlaps:
                for overlap in overlaps:
                    obj_id = overlap[0]
                    # Ignore other birds (already handled by separation) and ground plane (usually ID 0)
                    if obj_id not in self.body_ids and obj_id > 0:
                        try:
                            # Get exact AABB of the obstacle
                            o_min, o_max = p.getAABB(obj_id, physicsClientId=self.client_id)
                            # Find closest point on the obstacle AABB to the bird
                            cx = max(o_min[0], min(x, o_max[0]))
                            cy = max(o_min[1], min(y, o_max[1]))
                            cz = max(o_min[2], min(z, o_max[2]))
                            
                            diff = np.array([x - cx, y - cy, z - cz], dtype=np.float64)
                            dist = np.linalg.norm(diff)
                            
                            # Avoid division by zero if bird somehow overlaps/penetrates
                            if dist < 0.05:
                                obj_pos, _ = p.getBasePositionAndOrientation(obj_id, physicsClientId=self.client_id)
                                center_diff = bird['pos'] - np.array(obj_pos)
                                dist_horiz = np.linalg.norm(center_diff[:2])
                                if dist_horiz > 0.01:
                                    obs_x += (center_diff[0] / dist_horiz) * 10.0
                                    obs_y += (center_diff[1] / dist_horiz) * 10.0
                                else:
                                    obs_x += random.uniform(-1.0, 1.0) * 10.0
                                    obs_y += random.uniform(-1.0, 1.0) * 10.0
                                obs_z += 5.0
                                continue
                                
                            # Avoidance distance threshold (e.g. 3.5 meters)
                            if dist < 3.5:
                                weight = 1.0 / (dist * dist)
                                # Horizontal repulsion vector
                                horiz_dist = np.linalg.norm(diff[:2])
                                if horiz_dist > 0.05:
                                    obs_x += (diff[0] / horiz_dist) * weight
                                    obs_y += (diff[1] / horiz_dist) * weight
                                else:
                                    obj_pos, _ = p.getBasePositionAndOrientation(obj_id, physicsClientId=self.client_id)
                                    center_diff = bird['pos'] - np.array(obj_pos)
                                    dist_horiz = np.linalg.norm(center_diff[:2])
                                    if dist_horiz > 0.01:
                                        obs_x += (center_diff[0] / dist_horiz) * weight
                                        obs_y += (center_diff[1] / dist_horiz) * weight
                                
                                # Vertical climb decision:
                                # Can we clear this obstacle? If the top is less than 3.0m above us,
                                # we can try to fly over it. Otherwise, we must steer around it.
                                top_dist = o_max[2] - z
                                if 0 <= top_dist < 3.0:
                                    obs_z += (3.0 - top_dist) * weight * 1.5
                        except Exception:
                            pass

            obs_mag = math.sqrt(obs_x**2 + obs_y**2)
            if obs_mag > 0:
                obs_x /= obs_mag
                obs_y /= obs_mag
                # Stronger response to obstacles than flock separation
                sep_x = sep_x * 0.3 + obs_x * 0.7
                sep_y = sep_y * 0.3 + obs_y * 0.7
            
            if obs_z > 0:
                vz += min(obs_z * 0.5, 1.5)  # Add emergency climb

            # Blend heading direction + separation/avoidance
            fwd_x = math.cos(bird['heading'])
            fwd_y = math.sin(bird['heading'])
            # If avoiding obstacles, reduce forward momentum weight
            avoid_weight = 0.6 if obs_mag > 0 else 0.25
            blend_x = (1.0 - avoid_weight) * fwd_x + avoid_weight * sep_x
            blend_y = (1.0 - avoid_weight) * fwd_y + avoid_weight * sep_y

            blend_mag = math.sqrt(blend_x**2 + blend_y**2)
            if blend_mag > 0:
                blend_x /= blend_mag
                blend_y /= blend_mag

            # New position
            speed = bird['speed']
            new_x = bird['pos'][0] + blend_x * speed * dt
            new_y = bird['pos'][1] + blend_y * speed * dt
            new_z = bird['pos'][2] + vz * dt

            # --- Arena boundary reflection ---
            if new_x > arena or new_x < -arena:
                bird['heading'] = math.pi - bird['heading']
                new_x = np.clip(new_x, -arena, arena)
            if new_y > arena or new_y < -arena:
                bird['heading'] = -bird['heading']
                new_y = np.clip(new_y, -arena, arena)

            # Clamp Z to personal altitude band
            new_z = np.clip(new_z, max(0.8, bird['alt_min'] - 0.5), bird['alt_max'] + 0.5)

            bird['pos'] = np.array([new_x, new_y, new_z])

            # --- Update PyBullet kinematic body ---
            p.resetBasePositionAndOrientation(
                bird['id'],
                posObj=[new_x, new_y, new_z],
                ornObj=[0, 0, 0, 1],
                physicsClientId=self.client_id
            )


# ---------------------------------------------------------------------------
# 7. RoadAndParkSpawner
# ---------------------------------------------------------------------------

class RoadAndParkSpawner:
    """
    Spawns flat visual planes for roads and parks — purely for GUI visual enrichment.

    Roads: Gray thin boxes forming a cross (+) pattern through the arena centre.
    Parks: Green thin boxes in corner zones with slightly raised terrain feel.

    These are purely decorative in GUI mode. The class does nothing
    (returns empty list) when called — callers should check DEMO_MODE
    before instantiating.
    """

    ROAD_COLOR  = [0.30, 0.30, 0.32, 1.0]   # asphalt grey
    PARK_COLOR  = [0.22, 0.55, 0.22, 1.0]   # grass green
    CURB_COLOR  = [0.60, 0.60, 0.55, 1.0]   # kerb stone

    def __init__(self, physics_client_id: int):
        self.client_id = physics_client_id
        self.road_ids = []

    def _make_flat_box(self, x: float, y: float, z: float,
                       hx: float, hy: float, hz: float,
                       color: list) -> int:
        """Creates a static flat box at the given position."""
        col = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[hx, hy, hz],
            physicsClientId=self.client_id
        )
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[hx, hy, hz],
            rgbaColor=color,
            physicsClientId=self.client_id
        )
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[x, y, z],
            physicsClientId=self.client_id
        )
        return body_id

    def spawn(self, arena: float = 10.0) -> list:
        """
        Spawns road planes and park zones.

        Layout::

            ┌──────────────────────────────────┐
            │  PARK  │      Road (N/S)  │  PARK │
            │        │                  │       │
            │  ──────┼──── Road (E/W)──┼────── │
            │        │                  │       │
            │  PARK  │                  │  PARK │
            └──────────────────────────────────┘

        Args:
            arena: Half-width of the arena.

        Returns:
            List of PyBullet body IDs for all spawned objects.
        """
        self.road_ids.clear()
        road_thickness = 0.015   # very thin slab sitting on z=0
        road_width = 1.0         # half-width of road strip = 1m → 2m total road

        # Horizontal road (runs East–West along Y=0)
        body_id = self._make_flat_box(
            x=0.0, y=0.0, z=road_thickness,
            hx=arena, hy=road_width, hz=road_thickness,
            color=self.ROAD_COLOR
        )
        self.road_ids.append(body_id)

        # Vertical road (runs North–South along X=0)
        body_id = self._make_flat_box(
            x=0.0, y=0.0, z=road_thickness,
            hx=road_width, hy=arena, hz=road_thickness,
            color=self.ROAD_COLOR
        )
        self.road_ids.append(body_id)

        # Road lane markings — white dashed strips (decorative thin boxes)
        for lx in np.arange(-arena + 1.5, arena - 1.0, 2.5):
            mid = self._make_flat_box(
                x=float(lx), y=0.0, z=road_thickness * 2,
                hx=0.6, hy=0.06, hz=road_thickness * 0.5,
                color=[0.95, 0.95, 0.85, 1.0]
            )
            self.road_ids.append(mid)

        for ly in np.arange(-arena + 1.5, arena - 1.0, 2.5):
            mid = self._make_flat_box(
                x=0.0, y=float(ly), z=road_thickness * 2,
                hx=0.06, hy=0.6, hz=road_thickness * 0.5,
                color=[0.95, 0.95, 0.85, 1.0]
            )
            self.road_ids.append(mid)

        # Four corner park zones (green patches)
        park_half = arena * 0.38
        park_offset = arena * 0.60
        for px, py in [(-park_offset, -park_offset), (park_offset, -park_offset),
                       (-park_offset,  park_offset), (park_offset,  park_offset)]:
            park_id = self._make_flat_box(
                x=px, y=py, z=road_thickness * 0.5,
                hx=park_half, hy=park_half, hz=road_thickness * 0.5,
                color=self.PARK_COLOR
            )
            self.road_ids.append(park_id)

        # Kerb stones along road edges (small raised grey strips)
        for kx in np.arange(-arena + 0.5, arena - 0.5, 3.0):
            for ky in [road_width + 0.06, -(road_width + 0.06)]:
                k_id = self._make_flat_box(
                    x=float(kx), y=float(ky), z=0.04,
                    hx=1.2, hy=0.06, hz=0.04,
                    color=self.CURB_COLOR
                )
                self.road_ids.append(k_id)

        return self.road_ids
