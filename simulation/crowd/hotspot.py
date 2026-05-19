"""
simulation/crowd/hotspot.py
============================
Hotspot: a drifting crowd-attraction centre with a pulsing weight.

Each hotspot slowly wanders around the open areas of the city map.
Its `weight` oscillates between 0 and 1 via a sine wave, causing
crowd agents to cluster loosely and then disperse over time.

Teammate note: crowd/ module owner can modify drift/pulse behaviour here
without touching drone or path-planning code.
"""

import math
import random

from simulation.config import (
    PAD, SIM_W, SIM_H, TITLE_H,
    HOTSPOT_DRIFT, HOTSPOT_PULSE,
)
from simulation.environment.city_map import in_building, rand_open_pos


class Hotspot:
    """A drifting crowd-attraction centre with a pulsing attraction weight."""

    def __init__(self) -> None:
        self.x, self.y = rand_open_pos()
        self.angle  = random.uniform(0, math.tau)   # current drift direction
        self.weight = random.uniform(0.5, 1.0)       # attraction strength [0, 1]
        self.phase  = random.uniform(0, math.tau)    # phase for weight oscillation

    def reset(self) -> None:
        """Re-initialise to a new random position (called on simulation reset)."""
        self.x, self.y = rand_open_pos()
        self.angle  = random.uniform(0, math.tau)
        self.weight = random.uniform(0.5, 1.0)
        self.phase  = random.uniform(0, math.tau)

    def update(self) -> None:
        """Advance hotspot position and pulse weight by one simulation frame."""
        # Pulse weight via sine wave
        self.phase  += HOTSPOT_PULSE
        self.weight  = 0.5 + 0.5 * math.sin(self.phase)

        # Occasional direction perturbation for organic drift
        if random.random() < 0.01:
            self.angle += random.uniform(-0.5, 0.5)

        nx = self.x + math.cos(self.angle) * HOTSPOT_DRIFT
        ny = self.y + math.sin(self.angle) * HOTSPOT_DRIFT

        # Bounce off boundary walls
        if nx < PAD + 20 or nx > PAD + SIM_W - 20:
            self.angle = math.pi - self.angle
        if ny < TITLE_H + 20 or ny > TITLE_H + SIM_H - 20:
            self.angle = -self.angle

        nx = max(PAD + 20, min(PAD + SIM_W - 20, nx))
        ny = max(TITLE_H + 20, min(TITLE_H + SIM_H - 20, ny))

        if not in_building(nx, ny):
            self.x, self.y = nx, ny
        else:
            # Deflect away from building
            self.angle += math.pi * random.uniform(0.4, 0.8)

    def draw(self, surf) -> None:
        """Draw glowing hotspot indicator on the pygame surface."""
        import pygame
        ix, iy = int(self.x), int(self.y)
        r = int(40 + 20 * self.weight)
        s = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 200, 60, int(30 + 25 * self.weight)), (r, r), r)
        surf.blit(s, (ix - r, iy - r))
        pygame.draw.circle(surf, (255, 200, 60), (ix, iy), 5, 1)
        pygame.draw.line(surf, (255, 200, 60), (ix - 12, iy), (ix + 12, iy), 1)
        pygame.draw.line(surf, (255, 200, 60), (ix, iy - 12), (ix, iy + 12), 1)
