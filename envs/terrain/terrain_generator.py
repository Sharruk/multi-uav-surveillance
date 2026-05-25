"""
Terrain surface generator — spawns visually rich ground tiles
(roads, sidewalks, block surfaces, road markings, crosswalks)
using PyBullet primitive shapes.

All terrain bodies are VISUAL-ONLY (baseCollisionShapeIndex=-1) so
they do not interfere with drone physics or LiDAR raycasting.
The ground-collision surface comes from plane.urdf as usual.
"""

import pybullet as p
import numpy as np
from typing import List, Optional

from .city_layout import CityLayout, CityBlock, RoadSegment


# ── Realistic surface colour palette (RGBA tuples) ───────────────────────────
C = {
    'asphalt':        (0.20, 0.20, 0.20, 1.0),
    'asphalt_worn':   (0.27, 0.26, 0.25, 1.0),
    'sidewalk':       (0.72, 0.70, 0.66, 1.0),
    'curb':           (0.60, 0.58, 0.56, 1.0),
    'grass':          (0.26, 0.50, 0.22, 1.0),
    'grass_dry':      (0.42, 0.52, 0.24, 1.0),
    'plaza':          (0.76, 0.72, 0.68, 1.0),
    'gravel':         (0.52, 0.50, 0.47, 1.0),
    'parking':        (0.28, 0.28, 0.28, 1.0),
    'road_white':     (0.90, 0.90, 0.86, 1.0),
    'road_yellow':    (0.94, 0.82, 0.08, 1.0),
    'industrial_gnd': (0.45, 0.42, 0.38, 1.0),
}

BLOCK_SURFACE_COLOR = {
    'commercial':  C['plaza'],
    'residential': C['sidewalk'],
    'mixed':       C['plaza'],
    'park':        C['grass'],
    'plaza':       C['plaza'],
    'parking':     C['parking'],
    'industrial':  C['industrial_gnd'],
}


class TerrainGenerator:
    """
    Creates decorative ground-level terrain tiles from a CityLayout.
    Returns a flat list of visual-only PyBullet body IDs.
    """

    TILE_Z = 0.01   # ground tiles sit 1 cm above z=0 to avoid z-fighting with plane.urdf

    def __init__(self, physics_client: int,
                 rng: Optional[np.random.Generator] = None,
                 detail_level: str = 'medium'):
        self.client = physics_client
        self.rng    = rng or np.random.default_rng(42)
        self.detail = detail_level   # 'low' | 'medium' | 'high'
        self._ids: List[int] = []

    # ── Public API ────────────────────────────────────────────────────────────

    def spawn_from_layout(self, layout: CityLayout) -> List[int]:
        """Spawn all terrain tiles from a CityLayout. Returns body ID list."""
        self._ids.clear()

        # 1. Base ground tint (covers the entire city footprint + border)
        self._spawn_ground_base(layout)

        # 2. Road surfaces
        for road in layout.roads:
            self._spawn_road(road, layout.city_size)

        # 3. Block surfaces
        for block in layout.blocks:
            self._spawn_block_surface(block)

        # 4. Decorative details (skip on low detail)
        if self.detail != 'low':
            for road in layout.roads:
                self._spawn_center_line(road)

            if self.detail == 'high':
                for ix, iy in layout.intersections:
                    self._spawn_crosswalk(ix, iy, layout.road_width)

            for block in layout.blocks:
                if block.block_type == 'park':
                    self._spawn_park_patches(block)
                elif block.block_type == 'parking':
                    self._spawn_parking_lines(block)
                else:
                    self._spawn_sidewalk_strips(block)

        return list(self._ids)

    # ── Internal spawners ─────────────────────────────────────────────────────

    def _spawn_ground_base(self, layout: CityLayout) -> None:
        hw = layout.city_size[0] / 2 + 3.0
        hd = layout.city_size[1] / 2 + 3.0
        color = self._jitter(C['asphalt_worn'], 0.02)
        self._vis_box(0, 0, 0, hw, hd, 0.005, color)

    def _spawn_road(self, road: RoadSegment, city_size: tuple) -> None:
        color = self._jitter(C['asphalt'], 0.025)
        if road.orientation == 'horizontal':
            hw, hd = road.length / 2, road.width / 2
        else:
            hw, hd = road.width / 2, road.length / 2
        self._vis_box(road.x, road.y, self.TILE_Z, hw, hd, 0.005, color)

    def _spawn_block_surface(self, block: CityBlock) -> None:
        color = self._jitter(
            BLOCK_SURFACE_COLOR.get(block.block_type, C['sidewalk']), 0.03
        )
        hw, hd = block.width / 2 - 0.05, block.depth / 2 - 0.05
        self._vis_box(block.x, block.y, self.TILE_Z, hw, hd, 0.005, color)

    def _spawn_center_line(self, road: RoadSegment) -> None:
        """Dashed white center-line markings along a road."""
        dash_len = 0.70
        gap_len  = 0.55
        period   = dash_len + gap_len
        span     = road.length - 1.0   # leave 0.5m each end clear
        n_dashes = max(0, int(span / period))
        if n_dashes == 0:
            return
        start = -(n_dashes * period) / 2 + dash_len / 2

        for i in range(n_dashes):
            offset = start + i * period
            if road.orientation == 'horizontal':
                px, py = road.x + offset, road.y
                hw, hd = dash_len / 2, 0.04
            else:
                px, py = road.x, road.y + offset
                hw, hd = 0.04, dash_len / 2
            self._vis_box(px, py, self.TILE_Z + 0.005, hw, hd, 0.003, C['road_white'])

    def _spawn_crosswalk(self, ix: float, iy: float, road_w: float) -> None:
        """Zebra-crossing stripes at an intersection (all four arms)."""
        stripe_w  = 0.25
        stripe_d  = road_w * 0.38
        n_stripes = max(3, int(road_w / (stripe_w * 2.2)))
        z = self.TILE_Z + 0.006

        for arm_x, arm_y, horizontal in [
            (ix,         iy + road_w * 0.32,  True),   # north
            (ix,         iy - road_w * 0.32,  True),   # south
            (ix + road_w * 0.32, iy,          False),  # east
            (ix - road_w * 0.32, iy,          False),  # west
        ]:
            span   = road_w * 0.62
            start  = -(n_stripes * stripe_w * 2.2) / 2 + stripe_w / 2
            for i in range(n_stripes):
                off = start + i * stripe_w * 2.2
                if horizontal:
                    px, py = arm_x + off, arm_y
                    hw, hd = stripe_w / 2, stripe_d / 2
                else:
                    px, py = arm_x, arm_y + off
                    hw, hd = stripe_d / 2, stripe_w / 2
                self._vis_box(px, py, z, hw, hd, 0.003, C['road_white'])

    def _spawn_sidewalk_strips(self, block: CityBlock) -> None:
        """Raised concrete sidewalk border around a non-park block."""
        sw = 0.6    # sidewalk width
        ht = 0.025  # raised height
        hw, hd = block.width / 2, block.depth / 2
        c  = C['sidewalk']
        z  = self.TILE_Z + ht

        # N / S strips (full width)
        self._vis_box(block.x, block.y + hd - sw / 2, z, hw, sw / 2, ht, c)
        self._vis_box(block.x, block.y - hd + sw / 2, z, hw, sw / 2, ht, c)
        # E / W strips (between N and S strips)
        inner_hd = hd - sw
        self._vis_box(block.x + hw - sw / 2, block.y, z, sw / 2, inner_hd, ht, c)
        self._vis_box(block.x - hw + sw / 2, block.y, z, sw / 2, inner_hd, ht, c)

    def _spawn_park_patches(self, block: CityBlock) -> None:
        """Varied grass circles inside park blocks for texture."""
        hw, hd = block.width / 2 * 0.7, block.depth / 2 * 0.7
        n = self.rng.integers(3, 7)
        for _ in range(n):
            px  = block.x + self.rng.uniform(-hw, hw)
            py  = block.y + self.rng.uniform(-hd, hd)
            r   = self.rng.uniform(0.4, 1.1)
            col = self._jitter(C['grass_dry'], 0.06)
            c   = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=0.01,
                                      rgbaColor=col, physicsClientId=self.client)
            bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                    baseVisualShapeIndex=c,
                                    basePosition=[px, py, self.TILE_Z + 0.015],
                                    physicsClientId=self.client)
            self._ids.append(bid)

    def _spawn_parking_lines(self, block: CityBlock) -> None:
        """White parking-bay division lines in a parking lot."""
        hw, hd  = block.width / 2, block.depth / 2
        bay_w   = 1.3       # stall width
        bay_d   = min(2.8, hd * 0.85)
        n_bays  = max(1, int((hw * 2) / bay_w) - 1)
        z       = self.TILE_Z + 0.007

        for row_sign in [-1, 1]:
            row_y = block.y + row_sign * hd * 0.35
            start_x = block.x - n_bays * bay_w / 2
            for i in range(n_bays + 1):
                lx = start_x + i * bay_w
                self._vis_box(lx, row_y, z, 0.03, bay_d / 2, 0.004, C['road_white'])

    # ── Low-level helpers ──────────────────────────────────────────────────────

    def _vis_box(self, x: float, y: float, z: float,
                 hx: float, hy: float, hz: float, color: tuple) -> int:
        """Create a visual-only (no collision) coloured box body."""
        vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[hx, hy, hz],
            rgbaColor=color, physicsClientId=self.client
        )
        bid = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=vis,
            basePosition=[x, y, z],
            physicsClientId=self.client
        )
        self._ids.append(bid)
        return bid

    def _jitter(self, color: tuple, amount: float) -> tuple:
        """Add subtle random variation to a colour to break visual uniformity."""
        c = list(color[:3])
        c = [float(np.clip(v + self.rng.uniform(-amount, amount), 0.0, 1.0)) for v in c]
        return tuple(c) + (color[3],)
