"""
simulation/environment/dynamic_obstacles.py
============================================
Dynamic obstacle agents: moving vehicles and construction zones.

MovingVehicle
  - Travels along city road centrelines (horizontal or vertical)
  - Bounces at the road ends
  - Drawn as a small coloured rectangle

ConstructionZone
  - Stationary warning zone; relocates after CONSTRUCTION_LIFE frames
  - Drawn as an amber hazard area

UAV collision avoidance picks these up via rule_based_avoidance().

Teammate note: environment/ owner can add new obstacle types here.
All obstacles expose: x, y, radius, is_active  so the avoidance
module treats them uniformly.
"""

import math
import random
import pygame

from simulation.config import (
    PAD, SIM_W, SIM_H, TITLE_H,
    VEHICLE_SPD, VEHICLE_R,
    CONSTRUCTION_R, CONSTRUCTION_LIFE,
    NUM_VEHICLES, NUM_CONSTRUCTION,
)
from simulation.environment.city_map import (
    V_ROADS, H_ROADS, in_building, rand_open_pos,
)


# ── Road centreline helpers ────────────────────────────────────────────────────

def _v_road_paths() -> list[dict]:
    """Return centre-x and y-extent for each vertical road."""
    paths = []
    for x0, x1 in V_ROADS:
        paths.append({
            "axis": "v",
            "cx":   (x0 + x1) // 2,
            "y_min": TITLE_H + 10,
            "y_max": TITLE_H + SIM_H - 10,
        })
    return paths


def _h_road_paths() -> list[dict]:
    """Return centre-y and x-extent for each horizontal road."""
    paths = []
    for y0, y1 in H_ROADS:
        paths.append({
            "axis": "h",
            "cy":   (y0 + y1) // 2,
            "x_min": PAD + 10,
            "x_max": PAD + SIM_W - 10,
        })
    return paths


ALL_ROAD_PATHS = _v_road_paths() + _h_road_paths()


# ── MovingVehicle ──────────────────────────────────────────────────────────────

class MovingVehicle:
    """A vehicle travelling along a road centreline."""

    is_active = True  # always active

    def __init__(self) -> None:
        self._path = random.choice(ALL_ROAD_PATHS)
        self._dir  = random.choice([-1, 1])
        self._color = random.choice([
            (255, 120,  50),   # orange
            (180,  60, 220),   # purple
            ( 60, 180, 240),   # cyan
        ])
        self._place_on_path(random.uniform(0, 1))
        self.radius = VEHICLE_R

    def _place_on_path(self, t: float) -> None:
        p = self._path
        if p["axis"] == "v":
            self.x = float(p["cx"])
            self.y = p["y_min"] + t * (p["y_max"] - p["y_min"])
        else:
            self.x = p["x_min"] + t * (p["x_max"] - p["x_min"])
            self.y = float(p["cy"])

    def reset(self) -> None:
        self._path = random.choice(ALL_ROAD_PATHS)
        self._dir  = random.choice([-1, 1])
        self._place_on_path(random.uniform(0, 1))

    def update(self) -> None:
        p = self._path
        if p["axis"] == "v":
            self.y += self._dir * VEHICLE_SPD
            if self.y >= p["y_max"]:
                self._dir = -1
            elif self.y <= p["y_min"]:
                self._dir = 1
        else:
            self.x += self._dir * VEHICLE_SPD
            if self.x >= p["x_max"]:
                self._dir = -1
            elif self.x <= p["x_min"]:
                self._dir = 1

    def draw(self, surf) -> None:
        ix, iy = int(self.x), int(self.y)
        w, h = 18, 10
        p = self._path
        if p["axis"] == "h":
            rect = pygame.Rect(ix - w // 2, iy - h // 2, w, h)
        else:
            rect = pygame.Rect(ix - h // 2, iy - w // 2, h, w)
        pygame.draw.rect(surf, self._color, rect, border_radius=3)
        pygame.draw.rect(surf, (255, 255, 255), rect, 1, border_radius=3)
        # Hazard indicator
        pygame.draw.circle(surf, (255, 255, 255), (ix, iy), 3)


# ── ConstructionZone ───────────────────────────────────────────────────────────

class ConstructionZone:
    """A temporary road-work zone that relocates after its lifetime expires."""

    is_active = True

    def __init__(self) -> None:
        self.radius = CONSTRUCTION_R
        self._timer = random.randint(0, CONSTRUCTION_LIFE)
        self._relocate()

    def _relocate(self) -> None:
        # Place on a random road position (not in a building)
        p = random.choice(ALL_ROAD_PATHS)
        if p["axis"] == "v":
            self.x = float(p["cx"])
            self.y = random.uniform(p["y_min"] + 20, p["y_max"] - 20)
        else:
            self.x = random.uniform(p["x_min"] + 20, p["x_max"] - 20)
            self.y = float(p["cy"])
        self._timer = CONSTRUCTION_LIFE

    def reset(self) -> None:
        self._relocate()

    def update(self) -> None:
        self._timer -= 1
        if self._timer <= 0:
            self._relocate()

    def draw(self, surf) -> None:
        ix, iy = int(self.x), int(self.y)
        r = self.radius
        # Glowing amber zone
        s = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 160, 0, 55), (r, r), r)
        surf.blit(s, (ix - r, iy - r))
        pygame.draw.circle(surf, (255, 180, 0), (ix, iy), r, 2)
        # "⚠" style cross
        pygame.draw.line(surf, (255, 220, 0), (ix - 10, iy), (ix + 10, iy), 2)
        pygame.draw.line(surf, (255, 220, 0), (ix, iy - 10), (ix, iy + 10), 2)


# ── Factory ────────────────────────────────────────────────────────────────────

def create_obstacles() -> list:
    """Create the full set of dynamic obstacles for a simulation run."""
    obs: list = []
    obs.extend(MovingVehicle() for _ in range(NUM_VEHICLES))
    obs.extend(ConstructionZone() for _ in range(NUM_CONSTRUCTION))
    return obs


def reset_obstacles(obstacles: list) -> None:
    """Reset all dynamic obstacles (called on simulation reset)."""
    for o in obstacles:
        o.reset()
