"""
simulation/crowd/crowd_system.py
==================================
CrowdSystem: orchestrates all Person and Hotspot instances.

Responsibilities:
  - Cluster-spawn people near hotspots at init
  - Step hotspots and people each frame
  - Compute crowd statistics: density, monitored %, per-hotspot counts
  - Draw all crowd elements

Teammate note: analytics/monitoring logic is intentionally kept here
(tight coupling with people list). Heavier metric computation lives
in simulation/metrics/analytics.py.
"""

import math

from simulation.config import NUM_PEOPLE, NUM_HOTSPOTS, COVERAGE_R, SIM_W, SIM_H, TITLE_H
from simulation.crowd.hotspot import Hotspot
from simulation.crowd.person  import Person


class CrowdSystem:
    """Top-level crowd manager."""

    def __init__(self) -> None:
        self.hotspots: list[Hotspot] = [Hotspot() for _ in range(NUM_HOTSPOTS)]

        # Cluster-spawn: distribute people near each hotspot
        self.people: list[Person] = []
        per_hs = NUM_PEOPLE // NUM_HOTSPOTS
        for hs in self.hotspots:
            for _ in range(per_hs):
                self.people.append(Person(hs.x, hs.y))
        while len(self.people) < NUM_PEOPLE:
            self.people.append(Person())

        # Crowd statistics (updated every frame)
        self.center:         tuple[float, float] = (SIM_W // 2, TITLE_H + SIM_H // 2)
        self.density:        float = 0.0   # fraction of people near centroid
        self.monitored:      float = 0.0   # % of people inside any drone coverage
        self.hotspot_counts: list[int] = [0] * NUM_HOTSPOTS

    def update(self, drones: list | None = None) -> None:
        """Advance crowd by one frame.

        Args:
            drones: list of Drone objects (used to compute monitored %).
        """
        for hs in self.hotspots:
            hs.update()
        for p in self.people:
            p.update(self.people, self.hotspots)

        xs = [p.x for p in self.people]
        ys = [p.y for p in self.people]
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys))
        cx, cy = self.center

        # Density: fraction within 80 px of centroid
        self.density = sum(
            1 for p in self.people if math.hypot(p.x - cx, p.y - cy) < 80
        ) / NUM_PEOPLE

        # Per-hotspot person counts
        for i, hs in enumerate(self.hotspots):
            self.hotspot_counts[i] = sum(
                1 for p in self.people if math.hypot(p.x - hs.x, p.y - hs.y) < 70
            )

        # Monitored coverage
        if drones:
            self.monitored = (
                sum(
                    1 for p in self.people
                    if any(math.hypot(p.x - d.x, p.y - d.y) < COVERAGE_R for d in drones)
                )
                / NUM_PEOPLE * 100
            )
        else:
            self.monitored = 0.0

    def draw(self, surf) -> None:
        """Draw hotspots, crowd centroid indicator, and all people."""
        import pygame
        for hs in self.hotspots:
            hs.draw(surf)

        # Crowd centroid indicator
        cx, cy = int(self.center[0]), int(self.center[1])
        r = 28
        s = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 220, 100, 35), (r, r), r)
        surf.blit(s, (cx - r, cy - r))
        pygame.draw.circle(surf, (255, 200, 60), (cx, cy), 6, 1)
        pygame.draw.line(surf, (255, 200, 60), (cx - 10, cy), (cx + 10, cy), 1)
        pygame.draw.line(surf, (255, 200, 60), (cx, cy - 10), (cx, cy + 10), 1)

        for p in self.people:
            p.draw(surf)
