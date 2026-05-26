"""
BaseScenario — common spawning pipeline for all urban surveillance scenarios.

Subclasses override:
  SCENARIO_NAME        str
  SCENARIO_DESC        str
  DEFAULT_ARENA_BOUND  float
  _extra_spawn()       → List[int]  (scenario-specific visual elements)
  get_crowd_zones()    → List[dict]
"""

import numpy as np
import pybullet as p
from typing import List, Tuple, Dict, Optional

from ..terrain.city_layout       import CityLayoutGenerator
from ..terrain.terrain_generator import TerrainGenerator
from ..structures.buildings      import EnhancedBuildingSpawner
from ..structures.street_furniture import StreetFurnitureSpawner


class BaseScenario:
    """
    Spawns a complete urban scene:
      1. City layout grid (road + block positions)
      2. Terrain surface tiles (roads, sidewalks, parks, markings)
      3. Buildings with architectural detail
      4. Scenario-specific extras (plazas, event tents, containers, …)

    Returns a metadata dict compatible with drone_env.py's attribute names:
      building_ids    → List[int]  (main building collision bodies)
      tall_bld_ids    → List[int]  (always empty — buildings already in building_ids)
      house_ids       → List[int]  (always empty — replaced by new system)
      building_specs  → List[dict] compatible with RandomCrowd
      arena_bound     → float      (XY out-of-bounds threshold for drone_env)
      terrain_ids     → List[int]  (visual-only terrain bodies)
    """

    SCENARIO_NAME       = 'base'
    SCENARIO_DESC       = 'Base urban scenario'
    DEFAULT_ARENA_BOUND = 14.0   # override per scenario

    def __init__(self, physics_client: int, config: dict, seed: int = 42):
        self.client = physics_client
        self.config = config
        self.seed   = seed
        self._rng   = np.random.default_rng(seed)

        # Apply per-scenario config overrides
        self._terrain_cfg   = config.get('terrain',   {})
        self._building_cfg  = config.get('buildings', {})
        self._detail_level  = config.get('visual_quality', {}).get('detail_level', 'high')
        self._arena_bound   = config.get('arena', {}).get('xy_bound', self.DEFAULT_ARENA_BOUND)

    # ── Public API ────────────────────────────────────────────────────────────

    def spawn(self) -> Dict:
        """Run the full spawn pipeline and return the metadata dict."""
        building_ids: List[int] = []
        terrain_ids:  List[int] = []

        # 1. Generate city layout grid
        layout = CityLayoutGenerator(
            self._terrain_cfg,
            rng=np.random.default_rng(self.seed)
        ).generate(self.SCENARIO_NAME)
        self.layout = layout

        # 2. Spawn terrain surface (visual-only)
        tgen = TerrainGenerator(
            self.client,
            rng=np.random.default_rng(self.seed + 1),
            detail_level=self._detail_level
        )
        terrain_ids = tgen.spawn_from_layout(layout)

        # 3. Spawn buildings
        builder = EnhancedBuildingSpawner(
            self.client,
            rng=np.random.default_rng(self.seed + 2),
            detail_level=self._detail_level
        )
        for block in layout.blocks:
            c_ids, _v_ids = builder.spawn_for_block(block, self._building_cfg)
            building_ids.extend(c_ids)

        building_specs = builder.building_specs   # [{center, half_extents}, …]

        # 4. Scenario-specific extras
        extra_ids = self._extra_spawn()

        # 5. Phase-2 street furniture (all visual-only)
        furniture = StreetFurnitureSpawner(
            self.client,
            rng=np.random.default_rng(self.seed + 10)
        )
        furniture_ids = furniture.spawn_all(layout)

        return {
            'building_ids':   building_ids,
            'tall_bld_ids':   [],           # merged into building_ids
            'house_ids':      [],           # replaced by new system
            'building_specs': building_specs,
            'terrain_ids':    terrain_ids,
            'extra_ids':      extra_ids,
            'furniture_ids':  furniture_ids,
            'arena_bound':    self._arena_bound,
            'layout':         layout,
            'crowd_zones':    self.get_crowd_zones(),
        }

    # ── Override hooks ────────────────────────────────────────────────────────

    def _extra_spawn(self) -> List[int]:
        """Override to spawn scenario-specific elements (returns body IDs)."""
        return []

    def get_crowd_zones(self) -> List[Dict]:
        """Override to return crowd density zones for this scenario."""
        return [{'center': [0.0, 0.0], 'radius': 6.0, 'density': 'medium'}]

    # ── Shared primitive helpers (available to subclasses) ────────────────────

    def _col_box(self, x, y, z, hx, hy, hz, color) -> int:
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
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                   rgbaColor=color, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, z],
                                  physicsClientId=self.client)

    def _vis_cylinder(self, x, y, z, radius, height, color) -> int:
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height,
                                   rgbaColor=color, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, z],
                                  physicsClientId=self.client)

    def _vis_sphere(self, x, y, z, radius, color) -> int:
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                                   rgbaColor=color, physicsClientId=self.client)
        return p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=vis,
                                  basePosition=[x, y, z],
                                  physicsClientId=self.client)
