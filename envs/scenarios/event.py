"""
Event Crowd Control Scenario
Large central plaza surrounded by commercial buildings.
Very dense crowd gathering — ideal for crowd tracking & density estimation research.
Good for: Crowd density estimation, target tracking under occlusion.
"""

import numpy as np
from typing import List, Dict
from .scenario_base import BaseScenario


class EventScenario(BaseScenario):
    SCENARIO_NAME       = 'event'
    SCENARIO_DESC       = 'Event crowd control — large plaza, dense gathering, surrounding buildings'
    DEFAULT_ARENA_BOUND = 15.0

    def __init__(self, physics_client, config, seed=42):
        cfg = dict(config)
        if not cfg.get('terrain'):
            cfg['terrain'] = {}
        t = cfg['terrain']
        t.setdefault('city_size',  [26.0, 26.0])
        t.setdefault('block_size', 7.0)
        t.setdefault('road_width', 2.0)
        if not cfg.get('buildings'):
            cfg['buildings'] = {}
        cfg['buildings'].setdefault('height_range', [4.0, 12.0])
        cfg['buildings'].setdefault('density', 'medium')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        # ── Grand plaza surface ────────────────────────────────────────────────
        ids.append(self._vis_box(0, 0, 0.02, 5.5, 5.5, 0.02,
                                  (0.80, 0.76, 0.72, 1.0)))

        # ── Stage structure (event stage) ─────────────────────────────────────
        # Stage platform
        ids.append(self._vis_box(-2.0, -4.0, 0.50, 3.0, 1.2, 0.50,
                                  (0.25, 0.25, 0.25, 1.0)))
        # Stage back-drop
        ids.append(self._vis_box(-2.0, -5.0, 2.0, 3.0, 0.15, 2.0,
                                  (0.18, 0.18, 0.20, 1.0)))
        # Stage lights (orange cones)
        for lx in [-4.5, -2.0, 0.5]:
            ids.append(self._vis_cylinder(lx, -4.6, 3.8, 0.08, 0.30,
                                           (1.0, 0.75, 0.10, 1.0)))

        # ── Event tent canopies ────────────────────────────────────────────────
        tent_color = (0.85, 0.85, 0.90, 0.82)
        for tx, ty in [(4.5, 3.5), (-4.5, 3.5), (4.5, -1.5)]:
            ids.append(self._vis_box(tx, ty, 2.2, 1.8, 1.8, 0.08, tent_color))
            for px, py in [(tx - 1.8, ty - 1.8), (tx + 1.8, ty - 1.8),
                            (tx - 1.8, ty + 1.8), (tx + 1.8, ty + 1.8)]:
                ids.append(self._vis_cylinder(px, py, 1.1, 0.035, 2.2,
                                               (0.50, 0.48, 0.46, 1.0)))

        # ── Crowd barriers (low metal fences) ─────────────────────────────────
        barrier_c = (0.62, 0.62, 0.65, 1.0)
        for bx, by, bw, bd in [
            (-2.0, -2.5, 5.5, 0.06),  # front row
            (-6.0,  0.0, 0.06, 5.0),   # left wing
            ( 2.0,  0.0, 0.06, 5.0),   # right wing
        ]:
            ids.append(self._vis_box(bx, by, 0.55, bw, bd, 0.55, barrier_c))

        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [-2.0,  0.0],  'radius': 5.5, 'density': 'very_high'},
            {'center': [ 4.0,  3.5],  'radius': 2.5, 'density': 'high'},
            {'center': [-4.5,  3.5],  'radius': 2.5, 'density': 'high'},
            {'center': [ 0.0, -6.5],  'radius': 2.0, 'density': 'medium'},
        ]
