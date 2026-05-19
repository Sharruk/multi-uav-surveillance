"""
simulation/environment/city_map.py
===================================
Static city map definition:
  - Block / road geometry helpers
  - Building rectangles with labels
  - Zone rectangles with colours
  - Utility functions: _in_building(), _rand_open_pos()

This module is Pygame-free for most of its data; Pygame Rects are built
lazily at import time (after pygame.init() has been called by the runner).
"""

import math
import random
import pygame

from simulation.config import (
    PAD, BLK, RD,
    TITLE_H, SIM_W, SIM_H,
    C_ROAD, C_ROAD_MARK,
    C_BLDG_FILL, C_BLDG_BORDER, C_BLDG_LABEL,
)

# ── Grid helper ────────────────────────────────────────────────────────────────

def blk(col: int, row: int) -> tuple[int, int]:
    """Return the top-left pixel coordinate of grid cell (col, row)."""
    return (PAD + col * (BLK + RD), TITLE_H + PAD + row * (BLK + RD))


# ── Road geometry ──────────────────────────────────────────────────────────────

V_ROADS = [
    (PAD + BLK,         PAD + BLK + RD),
    (PAD + 2*BLK + RD,  PAD + 2*BLK + 2*RD),
]
H_ROADS = [
    (TITLE_H + PAD + BLK,         TITLE_H + PAD + BLK + RD),
    (TITLE_H + PAD + 2*BLK + RD,  TITLE_H + PAD + 2*BLK + 2*RD),
]

# ── Buildings ──────────────────────────────────────────────────────────────────

def _make_bldg(col: int, row: int, short: str, label: str):
    bx, by = blk(col, row)
    return (pygame.Rect(bx+3, by+3, BLK-6, BLK-6), short, label)


BUILDINGS = [
    _make_bldg(0, 0, "COM", "Commercial\nDistrict"),
    _make_bldg(1, 0, "CTH", "City Hall"),
    _make_bldg(2, 0, "TEC", "Tech Hub"),
    _make_bldg(1, 1, "MAL", "Central\nMall"),
    _make_bldg(2, 2, "UNI", "University"),
]

BLDG_RECTS: list[pygame.Rect] = [b[0] for b in BUILDINGS]

# ── Zones ──────────────────────────────────────────────────────────────────────

_ZONE_COLORS = [
    ((60,  185,  90), (80,  220, 110)),
    ((220, 110,  40), (255, 140,  55)),
    ((80,  155, 255), (110, 185, 255)),
    ((200,  80,  80), (230, 110, 100)),
]
_ZONE_DEFS = [(0, 1, "Zone-A"), (2, 1, "Zone-B"), (0, 2, "Zone-C"), (1, 2, "Zone-D")]

ZONES: list[dict] = []
for _i, (_c, _r, _lbl) in enumerate(_ZONE_DEFS):
    _bx, _by = blk(_c, _r)
    ZONES.append({
        "rect":   pygame.Rect(_bx+3, _by+3, BLK-6, BLK-6),
        "label":  _lbl,
        "fill":   _ZONE_COLORS[_i][0],
        "border": _ZONE_COLORS[_i][1],
    })

# ── Geometry utilities ─────────────────────────────────────────────────────────

def in_building(x: float, y: float) -> bool:
    """Return True if (x, y) overlaps any building rectangle."""
    for r in BLDG_RECTS:
        if r.collidepoint(x, y):
            return True
    return False


def rand_open_pos() -> tuple[float, float]:
    """Return a random position that does not overlap any building."""
    for _ in range(200):
        x = random.uniform(PAD + 10, PAD + SIM_W - 10)
        y = random.uniform(TITLE_H + 10, TITLE_H + SIM_H - 10)
        if not in_building(x, y):
            return x, y
    return PAD + SIM_W // 2, TITLE_H + SIM_H // 2
