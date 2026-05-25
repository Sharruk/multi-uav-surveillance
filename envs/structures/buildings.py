"""
Enhanced building generator with realistic architectural detail.

Design principle:
  - Main building body   → collision + visual (registered in building_ids)
  - Detail elements      → visual-only (no physics overhead)

Spawner returns two lists per building:
  collision_ids  — main bodies only  (add to drone_env's building_ids)
  visual_ids     — detail bodies     (tracking only)
"""

import pybullet as p
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


# ── Colour palettes ──────────────────────────────────────────────────────────

WALL = {
    # High-rise / commercial
    'glass_dark':    (0.14, 0.22, 0.32, 1.0),
    'glass_teal':    (0.22, 0.42, 0.50, 1.0),
    'glass_silver':  (0.60, 0.65, 0.70, 1.0),
    'steel_grey':    (0.58, 0.60, 0.62, 1.0),
    'charcoal':      (0.26, 0.26, 0.28, 1.0),
    'slate_blue':    (0.36, 0.43, 0.54, 1.0),
    'concrete':      (0.55, 0.55, 0.55, 1.0),
    'concrete_lt':   (0.76, 0.74, 0.72, 1.0),
    # Residential
    'cream':         (0.92, 0.89, 0.82, 1.0),
    'off_white':     (0.93, 0.93, 0.90, 1.0),
    'sandstone':     (0.83, 0.71, 0.54, 1.0),
    'terracotta':    (0.74, 0.44, 0.30, 1.0),
    'brick_red':     (0.64, 0.31, 0.21, 1.0),
    'sage_green':    (0.54, 0.61, 0.51, 1.0),
    'warm_beige':    (0.88, 0.83, 0.72, 1.0),
    # Industrial
    'warehouse_lt':  (0.62, 0.57, 0.51, 1.0),
    'warehouse_dk':  (0.37, 0.34, 0.30, 1.0),
    'corrugated':    (0.68, 0.66, 0.62, 1.0),
}

ROOF = {
    'tar_flat':   (0.18, 0.18, 0.18, 1.0),
    'gravel_flat': (0.50, 0.48, 0.45, 1.0),
    'metal_grey': (0.58, 0.60, 0.63, 1.0),
    'tile_red':   (0.58, 0.26, 0.18, 1.0),
    'tile_brown': (0.40, 0.28, 0.20, 1.0),
    'dark_slate': (0.28, 0.30, 0.32, 1.0),
}

WIN = {
    'glass_blue':  (0.42, 0.62, 0.80, 0.88),
    'glass_grey':  (0.60, 0.65, 0.72, 0.88),
    'glass_dark':  (0.18, 0.26, 0.36, 0.92),
    'glass_green': (0.42, 0.60, 0.52, 0.88),
    'lit_warm':    (0.92, 0.84, 0.62, 0.92),
    'lit_white':   (0.88, 0.90, 0.94, 0.96),
}

# Commercial wall colour choices (weighted towards glass/steel look)
COMMERCIAL_WALLS = [
    WALL['glass_dark'], WALL['glass_teal'], WALL['glass_silver'],
    WALL['steel_grey'], WALL['charcoal'], WALL['slate_blue'], WALL['concrete'],
]
RESIDENTIAL_WALLS = [
    WALL['cream'], WALL['off_white'], WALL['sandstone'], WALL['terracotta'],
    WALL['brick_red'], WALL['sage_green'], WALL['warm_beige'], WALL['concrete_lt'],
]
INDUSTRIAL_WALLS = [
    WALL['warehouse_lt'], WALL['warehouse_dk'], WALL['corrugated'], WALL['concrete'],
]


@dataclass
class BuildingSpec:
    """Parameters for a single building."""
    x: float
    y: float
    width: float
    depth: float
    height: float
    building_type: str   # highrise | commercial | residential | shop | warehouse


class EnhancedBuildingSpawner:
    """
    Spawns realistic-looking buildings for each city block.

    Returns separate lists of:
      collision_ids — one body per building (main box, has physics collision)
      visual_ids    — architectural detail bodies (visual-only)
    """

    def __init__(self, physics_client: int,
                 rng: Optional[np.random.Generator] = None,
                 detail_level: str = 'high'):
        self.client  = physics_client
        self.rng     = rng or np.random.default_rng(42)
        self.detail  = detail_level   # 'low' | 'medium' | 'high'
        self._collision_ids: List[int] = []
        self._visual_ids:    List[int] = []
        self._specs:         List[dict] = []   # building_specs for crowd avoidance

    # ── Public ────────────────────────────────────────────────────────────────

    def spawn_for_block(self, block, config: dict) -> Tuple[List[int], List[int]]:
        """
        Spawn buildings appropriate for a city block.
        Returns (collision_ids, visual_only_ids).
        """
        if block.block_type in ('park', 'plaza', 'parking'):
            return [], []

        height_range = config.get('height_range', [4.0, 14.0])
        specs = self._plan_block(block, block.block_type, height_range)
        c_ids, v_ids = [], []
        for spec in specs:
            ci, vi = self._spawn(spec)
            c_ids.extend(ci)
            v_ids.extend(vi)
            self._collision_ids.extend(ci)
            self._visual_ids.extend(vi)
            # Record spec for crowd building-avoidance
            self._specs.append({
                'center':       [spec.x, spec.y],
                'half_extents': [spec.width / 2, spec.depth / 2, spec.height / 2],
            })
        return c_ids, v_ids

    @property
    def collision_ids(self) -> List[int]:
        return list(self._collision_ids)

    @property
    def visual_ids(self) -> List[int]:
        return list(self._visual_ids)

    @property
    def building_specs(self) -> List[dict]:
        """Compatible with drone_env.py building_specs format."""
        return list(self._specs)

    # ── Block layout planner ─────────────────────────────────────────────────

    def _plan_block(self, block, block_type: str,
                    height_range: list) -> List[BuildingSpec]:
        """Decide how many buildings and of what type to fill this block."""
        hw, hd = block.width / 2, block.depth / 2

        fill = {
            'commercial': 0.80, 'residential': 0.70,
            'mixed': 0.72, 'industrial': 0.85,
        }.get(block_type, 0.70)

        h_lo, h_hi = float(height_range[0]), float(height_range[1])

        if block_type == 'commercial':
            n  = self.rng.integers(1, 3)
            h  = lambda: float(self.rng.uniform(max(h_lo, 6.0), h_hi))
            bt = lambda: self.rng.choice(['highrise', 'commercial', 'commercial'],
                                          p=[0.40, 0.40, 0.20])
        elif block_type == 'residential':
            n  = self.rng.integers(2, 4)
            h  = lambda: float(self.rng.uniform(h_lo, min(h_hi, 9.0)))
            bt = lambda: self.rng.choice(['residential', 'residential', 'shop'],
                                          p=[0.60, 0.25, 0.15])
        elif block_type == 'mixed':
            n  = self.rng.integers(1, 3)
            h  = lambda: float(self.rng.uniform(h_lo, h_hi))
            bt = lambda: self.rng.choice(['commercial', 'residential', 'shop'],
                                          p=[0.40, 0.40, 0.20])
        elif block_type == 'industrial':
            n  = self.rng.integers(1, 2)
            h  = lambda: float(self.rng.uniform(h_lo, min(h_hi, 9.0)))
            bt = lambda: 'warehouse'
        else:
            return []

        sub_areas = self._subdivide(block.x, block.y, hw * fill, hd * fill, n)
        specs = []
        for (sx, sy, sw, sd) in sub_areas:
            specs.append(BuildingSpec(
                x=sx, y=sy,
                width=sw * 0.88, depth=sd * 0.88,
                height=h(),
                building_type=bt()
            ))
        return specs

    def _subdivide(self, cx, cy, hw, hd, n: int):
        """Split a block centre area into n sub-cells."""
        if n == 1:
            return [(cx, cy, hw * 2, hd * 2)]
        if n == 2:
            if hw >= hd:
                return [(cx - hw / 2, cy, hw, hd * 2),
                        (cx + hw / 2, cy, hw, hd * 2)]
            else:
                return [(cx, cy - hd / 2, hw * 2, hd),
                        (cx, cy + hd / 2, hw * 2, hd)]
        # n == 3 or 4: 2×2 grid, take n cells
        cells = [
            (cx - hw / 2, cy - hd / 2, hw, hd),
            (cx + hw / 2, cy - hd / 2, hw, hd),
            (cx - hw / 2, cy + hd / 2, hw, hd),
            (cx + hw / 2, cy + hd / 2, hw, hd),
        ]
        return cells[:n]

    # ── Building type spawners ────────────────────────────────────────────────

    def _spawn(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        t = spec.building_type
        if t == 'highrise':
            return self._spawn_highrise(spec)
        elif t == 'commercial':
            return self._spawn_commercial(spec)
        elif t == 'residential':
            return self._spawn_residential(spec)
        elif t == 'shop':
            return self._spawn_shop(spec)
        elif t == 'warehouse':
            return self._spawn_warehouse(spec)
        else:
            return self._spawn_generic(spec)

    def _spawn_highrise(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        hw, hd, hh = spec.width / 2, spec.depth / 2, spec.height / 2
        color = self.rng.choice(COMMERCIAL_WALLS)
        c_ids, v_ids = [], []

        # ── Main body (collision) ──────────────────────────────────────────
        c_ids.append(self._col_box(spec.x, spec.y, hh, hw, hd, hh, color))

        # ── Wide plinth (visual) ────────────────────────────────────────────
        plinth_h = min(1.2, spec.height * 0.07)
        pc = self._darken(color, 0.80)
        v_ids.append(self._vis_box(spec.x, spec.y, plinth_h / 2,
                                    hw + 0.25, hd + 0.25, plinth_h / 2, pc))

        if self.detail in ('medium', 'high'):
            # ── Window-band strips every floor ────────────────────────────
            win_c = self.rng.choice(list(WIN.values()))
            v_ids.extend(self._window_bands(spec, win_c))

            # ── Horizontal floor ledges ────────────────────────────────────
            v_ids.extend(self._floor_ledges(spec, self._darken(color, 0.78)))

        if self.detail == 'high':
            # ── Parapet wall ───────────────────────────────────────────────
            v_ids.append(self._vis_box(spec.x, spec.y, spec.height + 0.35,
                                        hw + 0.08, hd + 0.08, 0.35,
                                        self._darken(color, 0.70)))
            # ── Rooftop HVAC ───────────────────────────────────────────────
            v_ids.extend(self._rooftop_equipment(spec, color))

        return c_ids, v_ids

    def _spawn_commercial(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        """Mid-rise commercial — similar to highrise but shorter."""
        hw, hd, hh = spec.width / 2, spec.depth / 2, spec.height / 2
        color = self.rng.choice(COMMERCIAL_WALLS[:5])  # glass/steel range
        c_ids, v_ids = [], []

        c_ids.append(self._col_box(spec.x, spec.y, hh, hw, hd, hh, color))

        if self.detail != 'low':
            win_c = self.rng.choice([WIN['glass_blue'], WIN['glass_grey'], WIN['glass_dark']])
            v_ids.extend(self._window_bands(spec, win_c))
            v_ids.extend(self._floor_ledges(spec, self._darken(color, 0.80)))
            # Flat roof with border
            v_ids.append(self._vis_box(spec.x, spec.y, spec.height + 0.20,
                                        hw + 0.06, hd + 0.06, 0.20,
                                        ROOF['tar_flat']))
        return c_ids, v_ids

    def _spawn_residential(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        hw, hd, hh = spec.width / 2, spec.depth / 2, spec.height / 2
        color = self.rng.choice(RESIDENTIAL_WALLS)
        c_ids, v_ids = [], []

        c_ids.append(self._col_box(spec.x, spec.y, hh, hw, hd, hh, color))

        if self.detail != 'low':
            # Individual window squares on front face
            win_c = self.rng.choice([WIN['glass_grey'], WIN['lit_warm'], WIN['glass_blue']])
            v_ids.extend(self._residential_windows(spec, win_c))

            # Flat roof with slight overhang
            roof_c = self.rng.choice([ROOF['tile_red'], ROOF['tile_brown'], ROOF['metal_grey']])
            v_ids.append(self._vis_box(spec.x, spec.y, spec.height + 0.12,
                                        hw + 0.12, hd + 0.12, 0.12, roof_c))

        if self.detail == 'high':
            # Balconies on higher floors
            v_ids.extend(self._balconies(spec, color))

        return c_ids, v_ids

    def _spawn_shop(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        h = min(spec.height, 4.5)
        hw, hd = spec.width / 2, spec.depth / 2
        c_ids, v_ids = [], []

        # Bright facade colour for shops
        facade = self.rng.choice([
            (0.80, 0.25, 0.22, 1.0),  # red
            (0.22, 0.42, 0.72, 1.0),  # blue
            (0.25, 0.58, 0.32, 1.0),  # green
            (0.76, 0.56, 0.16, 1.0),  # amber
            WALL['cream'], WALL['sandstone'],
        ])
        c_ids.append(self._col_box(spec.x, spec.y, h / 2, hw, hd, h / 2, facade))

        if self.detail != 'low':
            # Large storefront glazing on front face
            win_w = hw * 0.70
            win_h = h * 0.38
            win_c = self.rng.choice([WIN['glass_blue'], WIN['glass_grey']])
            v_ids.append(self._vis_box(spec.x, spec.y + hd + 0.01, win_h * 0.5 + 0.45,
                                        win_w, 0.015, win_h / 2, win_c))
            # Coloured awning
            aw_c = self.rng.choice([
                (0.80, 0.18, 0.18, 1.0),
                (0.18, 0.40, 0.72, 1.0),
                (0.20, 0.56, 0.28, 1.0),
                (0.76, 0.54, 0.12, 1.0),
            ])
            v_ids.append(self._vis_box(spec.x, spec.y + hd + 0.28, h * 0.55,
                                        hw * 0.88, 0.28, 0.045, aw_c))
            # Roof tile
            roof_c = self.rng.choice([ROOF['tile_red'], ROOF['tile_brown']])
            v_ids.append(self._vis_box(spec.x, spec.y, h + 0.12, hw + 0.12, hd + 0.12, 0.12, roof_c))

        return c_ids, v_ids

    def _spawn_warehouse(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        hw, hd, hh = spec.width / 2, spec.depth / 2, spec.height / 2
        color = self.rng.choice(INDUSTRIAL_WALLS)
        c_ids, v_ids = [], []

        c_ids.append(self._col_box(spec.x, spec.y, hh, hw, hd, hh, color))

        if self.detail != 'low':
            # Metal roof
            v_ids.append(self._vis_box(spec.x, spec.y, spec.height + 0.22,
                                        hw + 0.18, hd + 0.18, 0.22, ROOF['metal_grey']))
            # Loading bay door outline
            bay_w = min(2.2, hw * 0.55)
            bay_h = min(3.2, spec.height * 0.65)
            v_ids.append(self._vis_box(spec.x, spec.y + hd + 0.01, bay_h / 2,
                                        bay_w / 2, 0.012, bay_h / 2,
                                        (0.16, 0.16, 0.16, 1.0)))

        return c_ids, v_ids

    def _spawn_generic(self, spec: BuildingSpec) -> Tuple[List[int], List[int]]:
        hw, hd, hh = spec.width / 2, spec.depth / 2, spec.height / 2
        bid = self._col_box(spec.x, spec.y, hh, hw, hd, hh, WALL['concrete'])
        return [bid], []

    # ── Architectural detail helpers ──────────────────────────────────────────

    def _window_bands(self, spec: BuildingSpec, win_color: tuple) -> List[int]:
        """Horizontal glazing strips at each floor level on all 4 faces."""
        ids    = []
        floor  = 3.2
        n_fl   = max(1, int(spec.height / floor))
        hw, hd = spec.width / 2, spec.depth / 2
        offs   = 0.008   # slight protrusion

        for fl in range(1, n_fl):
            z    = fl * floor + floor * 0.35
            bh   = floor * 0.38

            # Front / Back
            for dy, _ in [(hd + offs, 1), (-hd - offs, -1)]:
                ids.append(self._vis_box(spec.x, spec.y + dy, z,
                                          hw * 0.84, offs, bh / 2, win_color))
            # Left / Right
            for dx, _ in [(hw + offs, 1), (-hw - offs, -1)]:
                ids.append(self._vis_box(spec.x + dx, spec.y, z,
                                          offs, hd * 0.84, bh / 2, win_color))
        return ids

    def _residential_windows(self, spec: BuildingSpec, win_color: tuple) -> List[int]:
        """Individual square windows on front/back faces of residential buildings."""
        ids   = []
        floor = 2.8
        n_fl  = min(int(spec.height / floor) + 1, 8)
        n_wx  = max(1, int(spec.width / 2.2))
        hw, hd = spec.width / 2, spec.depth / 2
        offs  = 0.008

        for fl in range(1, n_fl):
            z = fl * floor - floor * 0.25
            for i in range(n_wx):
                if n_wx > 1:
                    wx = spec.x - hw * 0.70 + i * (hw * 1.40 / (n_wx - 1))
                else:
                    wx = spec.x
                wx = float(np.clip(wx, spec.x - hw + 0.30, spec.x + hw - 0.30))
                for dy in [hd + offs, -hd - offs]:
                    ids.append(self._vis_box(wx, spec.y + dy, z,
                                              0.28, offs, 0.38, win_color))
        return ids

    def _floor_ledges(self, spec: BuildingSpec, ledge_color: tuple) -> List[int]:
        """Thin horizontal ledge/cornice every ~3.5 floors on commercial buildings."""
        ids   = []
        period = 3.5
        n     = max(0, int(spec.height / period) - 1)
        hw    = spec.width / 2 + 0.06
        hd    = spec.depth / 2 + 0.06

        for i in range(1, n + 1):
            z = i * period
            ids.append(self._vis_box(spec.x, spec.y, z,
                                      hw + 0.05, hd + 0.05, 0.06, ledge_color))
        return ids

    def _balconies(self, spec: BuildingSpec, wall_color: tuple) -> List[int]:
        """Small balcony slabs + railings on residential buildings."""
        ids     = []
        floor_h = 2.8
        n_fl    = int(spec.height / floor_h)
        hd      = spec.depth / 2
        slab_c  = self._darken(wall_color, 0.88)
        rail_c  = (0.74, 0.74, 0.76, 1.0)

        for fl in range(2, min(n_fl + 1, 5)):
            z = fl * floor_h - floor_h * 0.45
            bw = min(spec.width * 0.42, 1.2)
            # Slab
            ids.append(self._vis_box(spec.x, spec.y + hd + 0.38, z,
                                      bw / 2, 0.38, 0.055, slab_c))
            # Rail
            ids.append(self._vis_box(spec.x, spec.y + hd + 0.76, z + 0.45,
                                      bw / 2, 0.025, 0.45, rail_c))
        return ids

    def _rooftop_equipment(self, spec: BuildingSpec, wall_color: tuple) -> List[int]:
        """HVAC unit + optional antenna on high-rise rooftop."""
        ids = []
        z   = spec.height
        hvac_c = (0.68, 0.70, 0.72, 1.0)

        if self.rng.random() > 0.25:
            ox = self.rng.uniform(-spec.width * 0.22, spec.width * 0.22)
            oy = self.rng.uniform(-spec.depth * 0.22, spec.depth * 0.22)
            ids.append(self._vis_box(spec.x + ox, spec.y + oy, z + 0.55,
                                      0.55, 0.75, 0.55, hvac_c))

        if spec.height > 10 and self.rng.random() > 0.45:
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.045, length=2.4,
                                       rgbaColor=(0.52, 0.54, 0.56, 1.0),
                                       physicsClientId=self.client)
            bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                     baseVisualShapeIndex=vis,
                                     basePosition=[spec.x, spec.y, z + 2.0],
                                     physicsClientId=self.client)
            ids.append(bid)

        return ids

    # ── Primitive helpers ────────────────────────────────────────────────────

    def _col_box(self, x, y, z, hx, hy, hz, color) -> int:
        """Box with both collision and visual shape (main building body)."""
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                      physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                   rgbaColor=color, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=col,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, z],
                                  physicsClientId=self.client)

    def _vis_box(self, x, y, z, hx, hy, hz, color) -> int:
        """Visual-only box (no collision shape — zero physics cost)."""
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                   rgbaColor=color, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, z],
                                  physicsClientId=self.client)

    @staticmethod
    def _darken(color: tuple, factor: float) -> tuple:
        return tuple(v * factor if i < 3 else v for i, v in enumerate(color))
