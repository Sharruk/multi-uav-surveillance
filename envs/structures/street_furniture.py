"""
Phase 2: Street furniture spawner for STIRS-2025 urban scenarios.
All objects use baseCollisionShapeIndex=-1 (visual-only, zero physics overhead).

Body-count budget per scenario: ~250 (down-tuned to keep downtown ≥100 FPS).
"""
import numpy as np
import pybullet as p


class StreetFurnitureSpawner:
    """
    Spawns Phase-2 street furniture onto a CityLayout:
      - Street lights   (LED poles, warm lamp head, ~10 m spacing)
      - Traffic signals (3-light, red-active, 4 bodies each)
      - Utility poles   (cross-arms + single power line per span)
      - Parked cars     (realistic colour palette)
      - Benches + trash bins  (parks / plazas, ≤2 per block)
      - Bus stops       (glass-panel shelters, 3 locations)
      - Varied sidewalk trees (randomised height + green shade, ~12 m spacing)
    """

    CAR_COLORS = [
        [0.93, 0.93, 0.93, 1.0],  # white
        [0.70, 0.72, 0.72, 1.0],  # silver
        [0.12, 0.12, 0.12, 1.0],  # black
        [0.10, 0.20, 0.50, 1.0],  # dark blue
        [0.75, 0.10, 0.10, 1.0],  # red
        [0.30, 0.32, 0.30, 1.0],  # charcoal
        [0.45, 0.28, 0.10, 1.0],  # bronze
        [0.12, 0.38, 0.18, 1.0],  # forest green
    ]

    FOLIAGE_COLORS = [
        [0.15, 0.65, 0.15, 1.0],  # bright green
        [0.08, 0.48, 0.08, 1.0],  # medium green
        [0.04, 0.32, 0.06, 1.0],  # dark forest
        [0.38, 0.48, 0.08, 1.0],  # olive
        [0.18, 0.52, 0.10, 1.0],  # mid forest
    ]

    def __init__(self, client_id: int, rng=None):
        self.client_id = client_id
        self.rng = rng or np.random.default_rng(42)

    # ── PyBullet visual-only helpers ──────────────────────────────────────────

    def _vis_box(self, pos, half_extents, color, ori=(0, 0, 0, 1)):
        vs = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[float(v) for v in half_extents],
                                  rgbaColor=[float(v) for v in color],
                                  physicsClientId=self.client_id)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vs,
                                  basePosition=[float(v) for v in pos],
                                  baseOrientation=[float(v) for v in ori],
                                  physicsClientId=self.client_id)

    def _vis_cylinder(self, pos, radius, height, color, ori=(0, 0, 0, 1)):
        vs = p.createVisualShape(p.GEOM_CYLINDER,
                                  radius=float(radius), length=float(height),
                                  rgbaColor=[float(v) for v in color],
                                  physicsClientId=self.client_id)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vs,
                                  basePosition=[float(v) for v in pos],
                                  baseOrientation=[float(v) for v in ori],
                                  physicsClientId=self.client_id)

    def _vis_sphere(self, pos, radius, color):
        vs = p.createVisualShape(p.GEOM_SPHERE,
                                  radius=float(radius),
                                  rgbaColor=[float(v) for v in color],
                                  physicsClientId=self.client_id)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vs,
                                  basePosition=[float(v) for v in pos],
                                  physicsClientId=self.client_id)

    # ── Street lights — 3 bodies each, 10 m spacing ───────────────────────────

    def spawn_street_lights(self, layout):
        """LED street lights on both sides of every road (~10 m spacing)."""
        ids = []
        rh = layout.road_width / 2
        cx, cy = layout.city_size[0] / 2, layout.city_size[1] / 2
        spacing = 10.0          # wider spacing keeps body count manageable

        for road in layout.roads:
            if road.orientation == 'horizontal':
                for x in np.arange(-cx + spacing, cx, spacing):
                    for side in (1, -1):
                        ids += self._street_light(
                            float(x), road.y + side * (rh + 0.70), side,
                            vertical=False)
            else:
                for y in np.arange(-cy + spacing, cy, spacing):
                    for side in (1, -1):
                        ids += self._street_light(
                            road.x + side * (rh + 0.70), float(y), side,
                            vertical=True)
        return ids

    def _street_light(self, x, y, side, vertical=False):
        """3-body street light: pole + combined arm/head box + warm lamp color."""
        ids = []
        ph = 7.5

        # Pole
        ids.append(self._vis_cylinder([x, y, ph / 2], 0.04, ph,
                                       [0.50, 0.50, 0.52, 1.0]))
        # Arm direction toward road (inward)
        arm = 0.40
        if not vertical:
            tip_x, tip_y = x, y - side * arm
        else:
            tip_x, tip_y = x - side * arm, y

        # Arm + lamp head combined into one box (saves a body vs separate)
        half_x = abs(tip_x - x) / 2 + 0.22
        half_y = abs(tip_y - y) / 2 + 0.12
        ids.append(self._vis_box(
            [(x + tip_x) / 2, (y + tip_y) / 2, ph - 0.06],
            [half_x, half_y, 0.07],
            [0.22, 0.22, 0.22, 1.0]))

        # Warm LED glow (bright colour, single sphere — high visual payoff)
        ids.append(self._vis_sphere([tip_x, tip_y, ph - 0.04], 0.11,
                                     [1.0, 0.87, 0.38, 0.95]))
        return ids

    # ── Traffic signals — 4 bodies each ──────────────────────────────────────

    def spawn_traffic_lights(self, layout):
        """One traffic signal at each intersection (NE corner)."""
        ids = []
        corner = layout.road_width / 2 + 0.45
        for ix, iy in layout.intersections:
            ids += self._traffic_signal(ix + corner, iy + corner)
        return ids

    def _traffic_signal(self, x, y):
        """4-body signal: pole-with-housing + red + amber + green spheres."""
        ids = []
        ph = 3.8
        # Pole merged with housing (one tall cylinder, dark)
        ids.append(self._vis_cylinder([x, y, (ph + 0.88) / 2], 0.10, ph + 0.88,
                                       [0.14, 0.14, 0.14, 1.0]))
        face_y = y + 0.11
        ids.append(self._vis_sphere([x, face_y, ph + 0.77], 0.085,
                                     [1.00, 0.04, 0.04, 1.0]))   # red   ON
        ids.append(self._vis_sphere([x, face_y, ph + 0.44], 0.085,
                                     [0.28, 0.22, 0.03, 1.0]))   # amber dim
        ids.append(self._vis_sphere([x, face_y, ph + 0.11], 0.085,
                                     [0.04, 0.18, 0.04, 1.0]))   # green dim
        return ids

    # ── Utility poles + single power line per span ────────────────────────────

    def spawn_electric_poles(self, layout):
        """Brown utility poles with cross-arms and one overhead wire per span."""
        ids = []
        rh = layout.road_width / 2
        cx, cy = layout.city_size[0] / 2, layout.city_size[1] / 2
        spacing = 8.0
        ph = 6.0
        wire_col = [0.10, 0.10, 0.10, 1.0]

        for road in layout.roads:
            poles = []
            if road.orientation == 'horizontal':
                py = road.y - rh - 1.15
                for x in np.arange(-cx + 3.0, cx, spacing):
                    poles.append((float(x), py))
                    ids += self._utility_pole(float(x), py, ph, 'horizontal')
                # Single centre power line per span (runs in X)
                for i in range(len(poles) - 1):
                    x1, _ = poles[i];  x2, _ = poles[i + 1]
                    mx = (x1 + x2) / 2;  span = abs(x2 - x1) / 2 - 0.05
                    ids.append(self._vis_box([mx, py, ph - 0.35],
                                             [span, 0.012, 0.012], wire_col))
            else:
                px = road.x + rh + 1.15
                for y in np.arange(-cy + 3.0, cy, spacing):
                    poles.append((px, float(y)))
                    ids += self._utility_pole(px, float(y), ph, 'vertical')
                # Single centre power line per span (runs in Y)
                for i in range(len(poles) - 1):
                    _, y1 = poles[i];  _, y2 = poles[i + 1]
                    my = (y1 + y2) / 2;  span = abs(y2 - y1) / 2 - 0.05
                    ids.append(self._vis_box([px, my, ph - 0.35],
                                             [0.012, span, 0.012], wire_col))
        return ids

    def _utility_pole(self, x, y, ph, road_orientation):
        ids = []
        ids.append(self._vis_cylinder([x, y, ph / 2], 0.055, ph,
                                       [0.40, 0.27, 0.13, 1.0]))
        # Cross-arm perpendicular to road
        if road_orientation == 'horizontal':
            arm_half = [0.04, 0.90, 0.04]
        else:
            arm_half = [0.90, 0.04, 0.04]
        ids.append(self._vis_box([x, y, ph - 0.35], arm_half,
                                   [0.42, 0.29, 0.14, 1.0]))
        return ids

    # ── Parked cars ───────────────────────────────────────────────────────────

    def spawn_parked_cars(self, layout):
        """Fill parking blocks and add sparse roadside cars."""
        ids = []
        for block in layout.blocks:
            if block.block_type == 'parking':
                ids += self._fill_parking_lot(block)
        ids += self._roadside_parking(layout)
        return ids

    def _fill_parking_lot(self, block):
        ids = []
        slot_l, slot_w = 4.6, 2.25
        n_col = max(1, int((block.width - 1.2) / slot_l))
        n_row = max(1, int((block.depth - 1.2) / slot_w))
        start_x = block.x - (n_col * slot_l) / 2 + slot_l / 2
        start_y = block.y - (n_row * slot_w) / 2 + slot_w / 2

        for row in range(n_row):
            for col in range(n_col):
                if self.rng.random() < 0.25:
                    continue
                cx = start_x + col * slot_l
                cy = start_y + row * slot_w
                color = self.CAR_COLORS[int(self.rng.integers(0, len(self.CAR_COLORS)))]
                ids += self._car(float(cx), float(cy), color, yaw=0.0)
        return ids

    def _roadside_parking(self, layout):
        ids = []
        rh = layout.road_width / 2
        cx = layout.city_size[0] / 2
        for road in layout.roads:
            if road.orientation != 'horizontal':
                continue
            for x in np.arange(-cx + 5.0, cx - 4.0, 11.0):
                if self.rng.random() < 0.40:
                    continue
                color = self.CAR_COLORS[int(self.rng.integers(0, len(self.CAR_COLORS)))]
                ids += self._car(float(x), road.y + rh + 1.15, color, yaw=0.0)
        return ids

    def _car(self, x, y, color, yaw=0.0):
        """2-body car: lower body + narrower cabin."""
        ids = []
        q = p.getQuaternionFromEuler([0, 0, float(yaw)])
        ids.append(self._vis_box([x, y, 0.52], [1.85, 0.88, 0.52], color, q))
        cabin = [max(0.0, color[0] - 0.07), max(0.0, color[1] - 0.07),
                 max(0.0, color[2] - 0.07), 1.0]
        ids.append(self._vis_box([x, y, 1.24], [1.08, 0.78, 0.35], cabin, q))
        return ids

    # ── Benches — ≤2 per block, 3 bodies ─────────────────────────────────────

    def spawn_benches(self, layout):
        """Up to 2 benches per park/plaza block."""
        ids = []
        for block in layout.blocks:
            if block.block_type not in ('park', 'plaza'):
                continue
            for _ in range(2):          # exactly 2 — predictable body count
                bx = block.x + float(self.rng.uniform(
                    -block.width * 0.30, block.width * 0.30))
                by = block.y + float(self.rng.uniform(
                    -block.depth * 0.30, block.depth * 0.30))
                ids += self._bench(bx, by, float(self.rng.uniform(0, np.pi)))
        return ids

    def _bench(self, x, y, yaw=0.0):
        """3-body bench: seat + backrest + single end-support box."""
        ids = []
        q = p.getQuaternionFromEuler([0, 0, yaw])
        wood  = [0.55, 0.34, 0.13, 1.0]
        metal = [0.32, 0.32, 0.34, 1.0]
        ids.append(self._vis_box([x, y, 0.44], [0.70, 0.19, 0.042], wood, q))
        ids.append(self._vis_box([x, y, 0.73], [0.70, 0.042, 0.24], wood, q))
        # Single support spanning full bench width (1 body instead of 2 legs)
        ids.append(self._vis_box([x, y, 0.22], [0.68, 0.06, 0.22], metal, q))
        return ids

    # ── Bus stops — 4 bodies each ─────────────────────────────────────────────

    def spawn_bus_stops(self, layout):
        """3 glass-panel bus shelters at road-edge positions."""
        ids = []
        rh = layout.road_width / 2
        candidates = []
        for road in layout.roads:
            if road.orientation == 'horizontal':
                for x_off in (-4.5, 0.0, 4.5):
                    candidates.append((x_off, road.y + rh + 0.65))
                    candidates.append((x_off, road.y - rh - 0.65))

        if not candidates:
            return ids
        n = min(3, len(candidates))
        chosen = self.rng.choice(len(candidates), size=n, replace=False)
        for idx in chosen:
            bx, by = candidates[int(idx)]
            ids += self._bus_stop(bx, by)
        return ids

    def _bus_stop(self, x, y):
        ids = []
        ids.append(self._vis_box([x, y, 1.10], [0.55, 0.04, 1.10],
                                   [0.55, 0.70, 0.88, 0.55]))
        ids.append(self._vis_box([x, y + 0.22, 2.24], [0.65, 0.44, 0.045],
                                   [0.52, 0.56, 0.60, 1.0]))
        for dx in (-0.50, 0.50):
            ids.append(self._vis_cylinder([x + dx, y + 0.44, 1.10], 0.028, 2.20,
                                           [0.38, 0.40, 0.43, 1.0]))
        return ids

    # ── Trash bins — 1 per park block ─────────────────────────────────────────

    def spawn_trash_bins(self, layout):
        """One dark-green bin per park/plaza block."""
        ids = []
        bin_col = [0.17, 0.22, 0.17, 1.0]
        lid_col = [0.22, 0.28, 0.22, 1.0]
        for block in layout.blocks:
            if block.block_type not in ('park', 'plaza'):
                continue
            bx = block.x + float(self.rng.uniform(
                -block.width * 0.35, block.width * 0.35))
            by = block.y + float(self.rng.uniform(
                -block.depth * 0.35, block.depth * 0.35))
            ids.append(self._vis_cylinder([bx, by, 0.37], 0.17, 0.74, bin_col))
            ids.append(self._vis_cylinder([bx, by, 0.77], 0.19, 0.058, lid_col))
        return ids

    # ── Varied sidewalk trees — ~12 m spacing ────────────────────────────────

    def spawn_street_trees(self, layout):
        """Sidewalk trees with randomised trunk height and foliage colour."""
        ids = []
        rh = layout.road_width / 2
        cx, cy = layout.city_size[0] / 2, layout.city_size[1] / 2
        spacing = 12.0          # wider than legacy trees to avoid crowding
        trunk_col = [0.36, 0.24, 0.11, 1.0]

        for road in layout.roads:
            if road.orientation == 'horizontal':
                for i, x in enumerate(np.arange(-cx + 5.0, cx - 2.0, spacing)):
                    side = 1 if i % 2 == 0 else -1
                    ids += self._varied_tree(float(x),
                                             road.y + side * (rh + 1.40), trunk_col)
            else:
                for i, y in enumerate(np.arange(-cy + 5.0, cy - 2.0, spacing)):
                    side = 1 if i % 2 == 0 else -1
                    ids += self._varied_tree(road.x + side * (rh + 1.40),
                                             float(y), trunk_col)
        return ids

    def _varied_tree(self, x, y, trunk_col):
        ids = []
        trunk_h = float(self.rng.uniform(1.8, 4.2))
        foliage_r = float(self.rng.uniform(0.55, 1.22))
        f_col = self.FOLIAGE_COLORS[int(self.rng.integers(0, len(self.FOLIAGE_COLORS)))]
        ids.append(self._vis_cylinder([x, y, trunk_h / 2], 0.055, trunk_h, trunk_col))
        ids.append(self._vis_sphere([x, y, trunk_h + foliage_r * 0.65], foliage_r, f_col))
        return ids

    # ── Entry point ───────────────────────────────────────────────────────────

    def spawn_all(self, layout):
        """Spawn all Phase-2 street furniture. Returns flat list of body IDs."""
        ids = []
        ids += self.spawn_street_lights(layout)
        ids += self.spawn_traffic_lights(layout)
        ids += self.spawn_electric_poles(layout)
        ids += self.spawn_parked_cars(layout)
        ids += self.spawn_benches(layout)
        ids += self.spawn_bus_stops(layout)
        ids += self.spawn_trash_bins(layout)
        ids += self.spawn_street_trees(layout)
        return ids
