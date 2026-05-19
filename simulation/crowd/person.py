"""
simulation/crowd/person.py
===========================
Person: a single pedestrian modelled as a boid agent with hotspot attraction.

Behaviour stack (applied each frame in order):
  1. Random wander — periodic angle jitter
  2. Boid flocking  — cohesion, alignment, separation vs. neighbours
  3. Hotspot bias   — steer toward nearest hotspot proportional to its weight
  4. Boundary & building avoidance

Teammate note: crowd/ module owner can swap out step 2 or 3 with alternative
crowd models (social force, potential field) without affecting other modules.
"""

import math
import random

from simulation.config import (
    PAD, SIM_W, SIM_H, TITLE_H,
    PERSON_SPD, FLOCK_R, C_CROWD,
)
from simulation.environment.city_map import in_building, rand_open_pos


class Person:
    """A single pedestrian boid agent."""

    def __init__(self, hx: float | None = None, hy: float | None = None) -> None:
        """
        Args:
            hx, hy: optional spawn anchor (cluster spawn near a hotspot).
                     If None, spawns at a random open position.
        """
        if hx is not None:
            # Cluster-spawn: offset by Gaussian noise around the anchor
            for _ in range(40):
                x = max(PAD + 8, min(PAD + SIM_W - 8, hx + random.gauss(0, 22)))
                y = max(TITLE_H + 8, min(TITLE_H + SIM_H - 8, hy + random.gauss(0, 22)))
                if not in_building(x, y):
                    self.x, self.y = x, y
                    break
            else:
                self.x, self.y = rand_open_pos()
        else:
            self.x, self.y = rand_open_pos()

        self.angle        = random.uniform(0, math.tau)
        self.spd          = PERSON_SPD * random.uniform(0.6, 1.4)
        self.wander_timer = random.randint(0, 60)

    def update(self, others: list["Person"], hotspots: list) -> None:
        """Advance the agent by one simulation frame.

        Args:
            others:   list of all Person instances (for boid rules)
            hotspots: list of Hotspot instances (for attraction bias)
        """
        # 1. Wander: periodic random angle change
        self.wander_timer -= 1
        if self.wander_timer <= 0:
            self.angle += random.uniform(-0.9, 0.9)
            self.wander_timer = random.randint(30, 90)

        # 2. Boid flocking (cohesion + alignment + separation)
        cx = cy = sx = sy = ax = ay = 0.0
        n = 0
        for o in others:
            if o is self:
                continue
            d = math.hypot(self.x - o.x, self.y - o.y)
            if 0 < d < FLOCK_R:
                cx += o.x;  cy += o.y
                ax += math.cos(o.angle);  ay += math.sin(o.angle)
                n += 1
                if d < 18:   # separation zone
                    sx -= (o.x - self.x) / d
                    sy -= (o.y - self.y) / d
        if n:
            cx /= n;  cy /= n
            cohx = (cx - self.x) * 0.002
            cohy = (cy - self.y) * 0.002
            alx  = ax / n * 0.04;  aly = ay / n * 0.04
            self.angle += math.atan2(cohy + aly + sy * 0.05,
                                     cohx + alx + sx * 0.05) * 0.3

        # 3. Hotspot attraction bias
        if hotspots:
            hs = min(hotspots, key=lambda h: math.hypot(self.x - h.x, self.y - h.y))
            ha = math.atan2(hs.y - self.y, hs.x - self.x)
            self.angle += math.sin(ha - self.angle) * hs.weight * 0.06

        self.angle = self.angle % math.tau

        # 4. Move and bounce off buildings / boundaries
        nx = self.x + math.cos(self.angle) * self.spd
        ny = self.y + math.sin(self.angle) * self.spd

        if in_building(nx, ny):
            self.angle += math.pi * random.uniform(0.4, 0.6)
            nx = self.x + math.cos(self.angle) * self.spd
            ny = self.y + math.sin(self.angle) * self.spd

        self.x = max(PAD + 4, min(PAD + SIM_W - 4, nx))
        self.y = max(TITLE_H + 4, min(TITLE_H + SIM_H - 4, ny))

        if in_building(self.x, self.y):
            self.x, self.y = rand_open_pos()

    def draw(self, surf) -> None:
        """Draw the person as a small coloured circle."""
        import pygame
        ix, iy = int(self.x), int(self.y)
        pygame.draw.circle(surf, C_CROWD, (ix, iy), 3)
        pygame.draw.circle(surf, (200, 160, 40), (ix, iy), 3, 1)
