"""
Grid-based city layout generator for urban surveillance environments.
Produces a CityLayout describing block positions, road segments, and
intersection coordinates for use by terrain and building spawners.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class CityBlock:
    """One city block cell in the grid."""
    x: float          # center X (meters)
    y: float          # center Y (meters)
    width: float      # total block width (X)
    depth: float      # total block depth (Y)
    block_type: str   # commercial | residential | mixed | park | plaza | parking | industrial


@dataclass
class RoadSegment:
    """One road segment spanning the full city in one direction."""
    x: float           # center X
    y: float           # center Y
    length: float      # full span length
    width: float       # road width
    orientation: str   # 'horizontal' (runs along X) | 'vertical' (runs along Y)


@dataclass
class CityLayout:
    """Complete city layout description."""
    blocks: List[CityBlock] = field(default_factory=list)
    roads: List[RoadSegment] = field(default_factory=list)
    intersections: List[Tuple[float, float]] = field(default_factory=list)
    city_size: Tuple[float, float] = (26.0, 26.0)
    block_size: float = 6.0
    road_width: float = 2.0


class CityLayoutGenerator:
    """
    Generates a regular grid city layout within a given city_size bounding box.

    Grid math (for city_size=26, block_size=6, road_width=2, step=8):
      Roads at axis positions: -11, -3, 5  (3 roads per axis)
      Block centers between consecutive roads + city edges → 3×3 = 9 blocks
    """

    def __init__(self, config: dict, rng: Optional[np.random.Generator] = None):
        self.city_size  = config.get('city_size',  [26.0, 26.0])
        self.block_size = config.get('block_size', 6.0)
        self.road_width = config.get('road_width', 2.0)
        self.rng = rng or np.random.default_rng(42)

    def generate(self, scenario_type: str = 'downtown') -> CityLayout:
        layout = CityLayout(
            city_size=tuple(self.city_size),
            block_size=self.block_size,
            road_width=self.road_width,
        )

        step      = self.block_size + self.road_width
        half_w    = self.city_size[0] / 2
        half_d    = self.city_size[1] / 2
        road_half = self.road_width / 2

        # ── Road centre positions ──────────────────────────────────────────────
        road_xs = self._grid_positions(-half_w, half_w, road_half, step)
        road_ys = self._grid_positions(-half_d, half_d, road_half, step)

        for rx in road_xs:
            layout.roads.append(RoadSegment(
                x=rx, y=0.0,
                length=self.city_size[1],
                width=self.road_width,
                orientation='vertical'
            ))
        for ry in road_ys:
            layout.roads.append(RoadSegment(
                x=0.0, y=ry,
                length=self.city_size[0],
                width=self.road_width,
                orientation='horizontal'
            ))

        # ── Intersections ─────────────────────────────────────────────────────
        for rx in road_xs:
            for ry in road_ys:
                layout.intersections.append((rx, ry))

        # ── City blocks (cells between adjacent road/edge boundaries) ─────────
        # Build sorted lists of cell boundaries along each axis
        x_bounds = sorted(set([-half_w] + [rx - road_half for rx in road_xs]
                               + [rx + road_half for rx in road_xs] + [half_w]))
        y_bounds = sorted(set([-half_d] + [ry - road_half for ry in road_ys]
                               + [ry + road_half for ry in road_ys] + [half_d]))

        # A cell is a genuine city block only if it's wider than a road strip.
        # Road strips have width == road_width; blocks have width == block_size.
        # Threshold: anything <= road_width is a road, skip it.
        block_threshold = self.road_width + 0.5

        for i in range(len(x_bounds) - 1):
            x_lo, x_hi = x_bounds[i], x_bounds[i + 1]
            cell_w = x_hi - x_lo
            if cell_w <= block_threshold:
                continue
            bx = (x_lo + x_hi) / 2

            for j in range(len(y_bounds) - 1):
                y_lo, y_hi = y_bounds[j], y_bounds[j + 1]
                cell_d = y_hi - y_lo
                if cell_d <= block_threshold:
                    continue
                by = (y_lo + y_hi) / 2

                block_type = self._assign_type(bx, by, scenario_type, half_w, half_d)
                layout.blocks.append(CityBlock(
                    x=bx, y=by,
                    width=cell_w, depth=cell_d,
                    block_type=block_type
                ))

        return layout

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _grid_positions(lo: float, hi: float, half_road: float, step: float) -> List[float]:
        """Return road centre positions from lo+half_road stepping by step."""
        positions = []
        x = lo + half_road
        while x <= hi - half_road + 1e-6:
            positions.append(round(x, 6))
            x += step
        return positions

    def _assign_type(self, x: float, y: float, scenario: str,
                     half_w: float, half_d: float) -> str:
        """Assign a land-use type based on position and scenario."""
        dist     = np.sqrt(x ** 2 + y ** 2)
        max_dist = np.sqrt(half_w ** 2 + half_d ** 2)
        rel      = dist / max_dist  # 0 = center, 1 = corner

        choice = self.rng.choice

        if scenario == 'downtown':
            if rel < 0.25:
                return choice(['commercial', 'commercial', 'plaza'], p=[0.5, 0.35, 0.15])
            elif rel < 0.6:
                return choice(['commercial', 'residential', 'mixed'], p=[0.45, 0.30, 0.25])
            else:
                return choice(['residential', 'park', 'parking'],    p=[0.45, 0.35, 0.20])

        elif scenario == 'residential':
            if rel < 0.35:
                return choice(['residential', 'mixed', 'park'],      p=[0.5, 0.3, 0.2])
            else:
                return choice(['residential', 'park', 'parking'],    p=[0.55, 0.30, 0.15])

        elif scenario == 'event':
            if rel < 0.20:
                return 'plaza'
            elif rel < 0.55:
                return choice(['commercial', 'mixed'],                p=[0.55, 0.45])
            else:
                return choice(['residential', 'parking'],             p=[0.65, 0.35])

        elif scenario == 'industrial':
            if rel > 0.30:
                return choice(['industrial', 'parking'],              p=[0.70, 0.30])
            else:
                return choice(['industrial', 'mixed', 'commercial'],  p=[0.55, 0.25, 0.20])

        else:  # mixed
            return choice(
                ['residential', 'commercial', 'mixed', 'park', 'parking'],
                p=[0.30, 0.25, 0.25, 0.12, 0.08]
            )
