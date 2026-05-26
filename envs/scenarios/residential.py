"""
Residential Monitoring Scenario
Lower-rise neighbourhood with parks, parking lots, and tree-lined streets.
Good for: Formation control, coverage optimisation, suburban search patterns.
"""

import numpy as np
from typing import List, Dict
from .scenario_base import BaseScenario


class ResidentialScenario(BaseScenario):
    SCENARIO_NAME       = 'residential'
    SCENARIO_DESC       = 'Suburban residential — low-rise buildings, parks, parking lots'
    DEFAULT_ARENA_BOUND = 14.0

    def __init__(self, physics_client, config, seed=42):
        cfg = dict(config)
        if not cfg.get('terrain'):
            cfg['terrain'] = {}
        t = cfg['terrain']
        t.setdefault('city_size',  [24.0, 24.0])
        t.setdefault('block_size', 6.0)
        t.setdefault('road_width', 2.0)
        if not cfg.get('buildings'):
            cfg['buildings'] = {}
        cfg['buildings'].setdefault('height_range', [3.0, 9.0])
        cfg['buildings'].setdefault('density', 'medium')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        # Playground equipment in a park zone (simple colourful shapes)
        park_cx, park_cy = 5.5, 5.5   # approximate park block centre
        # Slide
        ids.append(self._vis_box(park_cx + 0.5, park_cy, 0.8,
                                  0.12, 0.6, 0.8, (0.80, 0.25, 0.18, 1.0)))
        # Swing frame
        for dx in [-0.6, 0.6]:
            ids.append(self._vis_cylinder(park_cx + dx, park_cy + 0.8, 1.2,
                                           0.04, 2.4, (0.55, 0.42, 0.30, 1.0)))
        ids.append(self._vis_box(park_cx, park_cy + 0.8, 2.4,
                                  0.62, 0.04, 0.04, (0.55, 0.42, 0.30, 1.0)))
        # Park bench
        ids.append(self._vis_box(park_cx - 1.5, park_cy - 1.0, 0.22,
                                  0.55, 0.12, 0.06, (0.48, 0.36, 0.24, 1.0)))
        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [0.0, 0.0],   'radius': 5.0, 'density': 'low'},
            {'center': [5.5, 5.5],   'radius': 3.0, 'density': 'medium'},
            {'center': [-5.5, 5.5],  'radius': 2.5, 'density': 'low'},
        ]
