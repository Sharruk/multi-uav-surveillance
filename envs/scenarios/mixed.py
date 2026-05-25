"""
Mixed Urban Area Scenario
Varied building heights, commercial + residential mix, streets + parks.
General-purpose scenario — best for realistic conditions and baseline benchmarking.
Good for: General algorithm testing, realistic urban conditions.
"""

from typing import List, Dict
from .scenario_base import BaseScenario


class MixedUrbanScenario(BaseScenario):
    SCENARIO_NAME       = 'mixed'
    SCENARIO_DESC       = 'Mixed urban — varied heights, commercial/residential mix, parks'
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
        cfg['buildings'].setdefault('height_range', [3.0, 14.0])
        cfg['buildings'].setdefault('density', 'medium')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        # Street market stalls in a mixed zone
        stall_c = (0.78, 0.65, 0.42, 1.0)
        for sx, sy in [(-1.5, 6.5), (0.0, 6.5), (1.5, 6.5)]:
            # Stall canopy
            ids.append(self._vis_box(sx, sy, 2.0, 0.65, 0.65, 0.05, stall_c))
            # Pole
            ids.append(self._vis_cylinder(sx, sy, 1.0, 0.03, 2.0,
                                           (0.48, 0.36, 0.24, 1.0)))
            # Goods display
            ids.append(self._vis_box(sx, sy - 0.3, 0.55, 0.50, 0.25, 0.55,
                                      (0.55, 0.55, 0.55, 1.0)))
        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [ 0.0,  0.0],  'radius': 5.0, 'density': 'medium'},
            {'center': [ 5.5, -5.5],  'radius': 3.0, 'density': 'low'},
            {'center': [-5.0,  5.0],  'radius': 3.0, 'density': 'medium'},
            {'center': [ 0.0,  6.5],  'radius': 2.0, 'density': 'high'},
        ]
