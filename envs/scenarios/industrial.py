"""
Industrial / Port Zone Scenario
Warehouses, open logistics yards, containers, sparse crowd.
Good for: Long-range tracking, wide-area coverage, perimeter surveillance.
"""

import numpy as np
from typing import List, Dict
from .scenario_base import BaseScenario


class IndustrialScenario(BaseScenario):
    SCENARIO_NAME       = 'industrial'
    SCENARIO_DESC       = 'Industrial / port zone — warehouses, containers, sparse crowd'
    DEFAULT_ARENA_BOUND = 15.0

    def __init__(self, physics_client, config, seed=42):
        cfg = dict(config)
        if not cfg.get('terrain'):
            cfg['terrain'] = {}
        t = cfg['terrain']
        t.setdefault('city_size',  [26.0, 26.0])
        t.setdefault('block_size', 8.0)
        t.setdefault('road_width', 3.0)
        if not cfg.get('buildings'):
            cfg['buildings'] = {}
        cfg['buildings'].setdefault('height_range', [4.0, 10.0])
        cfg['buildings'].setdefault('density', 'low')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        rng = self._rng

        # ── Shipping containers (standardised 6m × 2.4m × 2.6m) ───────────────
        container_colors = [
            (0.72, 0.18, 0.18, 1.0),   # red
            (0.18, 0.45, 0.72, 1.0),   # blue
            (0.22, 0.55, 0.28, 1.0),   # green
            (0.75, 0.60, 0.18, 1.0),   # yellow
            (0.55, 0.55, 0.55, 1.0),   # grey
        ]
        # Stack clusters in two yard areas
        for yard_cx, yard_cy in [(-7.0, -7.0), (6.0, -7.0)]:
            for row in range(3):
                for col in range(3):
                    cx = yard_cx + col * 2.55
                    cy = yard_cy + row * 6.2
                    stack_h = rng.integers(1, 3)
                    c = container_colors[rng.integers(len(container_colors))]
                    for layer in range(int(stack_h)):
                        ids.append(self._vis_box(cx, cy,
                                                  1.3 + layer * 2.6,
                                                  1.2, 3.0, 1.3, c))

        # ── Large industrial crane arm (decorative) ────────────────────────────
        # Vertical mast
        ids.append(self._vis_cylinder(-9.0, -5.0, 4.5, 0.18, 9.0,
                                       (0.60, 0.58, 0.55, 1.0)))
        # Horizontal jib
        ids.append(self._vis_box(-6.5, -5.0, 9.0, 2.5, 0.10, 0.10,
                                  (0.70, 0.65, 0.60, 1.0)))

        # ── Fuel / chemical tanks ─────────────────────────────────────────────
        tank_c = (0.70, 0.68, 0.65, 1.0)
        for tx, ty in [(8.5, 6.0), (10.0, 6.0)]:
            ids.append(self._vis_cylinder(tx, ty, 2.0, 1.0, 4.0, tank_c))
            # Dome cap
            ids.append(self._vis_sphere(tx, ty, 4.2, 1.02, tank_c))

        # ── Perimeter fence posts ─────────────────────────────────────────────
        fence_c = (0.50, 0.50, 0.52, 1.0)
        for i in range(-6, 7):
            for (fx, fy) in [(i * 2.0, 12.0), (i * 2.0, -12.0),
                              (12.0, i * 2.0), (-12.0, i * 2.0)]:
                ids.append(self._vis_cylinder(fx, fy, 1.2, 0.04, 2.4, fence_c))

        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [0.0, 0.0],    'radius': 4.0, 'density': 'low'},
            {'center': [5.0, 5.0],    'radius': 2.0, 'density': 'low'},
        ]
