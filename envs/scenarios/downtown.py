"""
Downtown Surveillance Scenario
Dense urban core with high-rises, busy streets, and plazas.
Good for: Occlusion testing, complex navigation, crowd density research.
"""

import numpy as np
from typing import List, Dict
from .scenario_base import BaseScenario


class DowntownScenario(BaseScenario):
    SCENARIO_NAME       = 'downtown'
    SCENARIO_DESC       = 'Dense urban downtown — high-rises, busy streets, large crowds'
    DEFAULT_ARENA_BOUND = 15.0

    def __init__(self, physics_client, config, seed=42):
        # Inject downtown defaults if terrain config is bare
        cfg = dict(config)
        if not cfg.get('terrain'):
            cfg['terrain'] = {}
        t = cfg['terrain']
        t.setdefault('city_size',  [26.0, 26.0])
        t.setdefault('block_size', 6.0)
        t.setdefault('road_width', 2.0)
        if not cfg.get('buildings'):
            cfg['buildings'] = {}
        cfg['buildings'].setdefault('height_range', [6.0, 18.0])
        cfg['buildings'].setdefault('density', 'high')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        # Central landmark plaza feature: raised stone platform
        ids.append(self._vis_box(0, 0, 0.18, 4.0, 4.0, 0.18,
                                  (0.78, 0.74, 0.70, 1.0)))
        # Fountain base
        ids.append(self._vis_cylinder(0, 0, 0.40, 1.0, 0.22,
                                       (0.72, 0.72, 0.76, 1.0)))
        ids.append(self._vis_cylinder(0, 0, 0.56, 0.30, 0.18,
                                       (0.68, 0.78, 0.88, 0.85)))
        # Four corner planters
        for sx, sy in [(3.0, 3.0), (-3.0, 3.0), (3.0, -3.0), (-3.0, -3.0)]:
            ids.append(self._vis_box(sx, sy, 0.28, 0.30, 0.30, 0.28,
                                      (0.42, 0.38, 0.34, 1.0)))
            ids.append(self._vis_sphere(sx, sy, 0.70, 0.40,
                                         (0.26, 0.50, 0.22, 1.0)))
        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [0.0,  0.0],  'radius': 7.0, 'density': 'high'},
            {'center': [5.0,  4.0],  'radius': 3.5, 'density': 'medium'},
            {'center': [-4.0, 3.0],  'radius': 3.0, 'density': 'medium'},
            {'center': [3.0, -5.0],  'radius': 2.5, 'density': 'low'},
        ]
