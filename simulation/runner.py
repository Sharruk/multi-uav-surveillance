"""
simulation/runner.py
=====================
SimulationRunner: owns the Pygame game loop, clock, reset logic, and rendering.

This is the only place where Pygame is initialised and where the main loop runs.
All subsystems (crowd, drones, metrics, renderer) are called from here.

Teammate note: runner.py is the integration point. If you add a new subsystem,
wire it in here — not inside individual agent classes.
"""

import time
import pygame

from simulation.config import (
    WIN_W, WIN_H, FPS, SIM_W, SIM_H, TITLE_H, PAD,
    C_BG, C_PANEL_LINE,
)
from simulation.crowd.crowd_system         import CrowdSystem
from simulation.drone.drone                import Drone
from simulation.metrics.analytics          import calc_metrics
from simulation.visualization.renderer    import (
    draw_roads, draw_buildings, draw_zones, draw_title,
)
from simulation.visualization.panel       import draw_panel


class SimulationRunner:
    """Top-level simulation controller."""

    NUM_DRONES = 4

    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Smart City UAV Surveillance — Research Prototype")
        self.clock = pygame.time.Clock()

        # Fonts
        self.ft  = pygame.font.SysFont("segoeui",  15, bold=True)
        self.fh  = pygame.font.SysFont("segoeui",  14, bold=True)
        self.fsm = pygame.font.SysFont("segoeui",  12)
        self.fxs = pygame.font.SysFont("consolas", 10)

        # Simulation state
        self.crowd   = CrowdSystem()
        self.drones  = [Drone(i, self.crowd) for i in range(self.NUM_DRONES)]
        self.col_ref = [0]   # mutable avoidance event counter (shared across drones)
        self.metrics = {
            "collisions": 0,
            "coverage":   0.0,
            "accuracy":   100.0,
            "total_dist": 0.0,
            "wind_spd":   0.0,
        }
        self.sim_start    = time.time()
        self.paused       = False
        self.metric_timer = 0

    def reset(self) -> None:
        """Full simulation reset (SPACE key)."""
        self.crowd = CrowdSystem()
        for d in self.drones:
            d.reset(self.crowd)
        self.col_ref[0] = 0
        self.metrics.update(
            collisions=0, coverage=0.0, accuracy=100.0,
            total_dist=0.0, wind_spd=0.0,
        )
        self.sim_start    = time.time()
        self.metric_timer = 0

    def _handle_events(self) -> bool:
        """Process pygame events. Returns False if the simulation should quit."""
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    return False
                if ev.key == pygame.K_SPACE:
                    self.reset()
                if ev.key == pygame.K_p:
                    self.paused = not self.paused
        return True

    def _update(self) -> None:
        """Advance all simulation subsystems by one frame."""
        self.crowd.update(self.drones)
        for d in self.drones:
            d.update(self.drones, self.col_ref)
        self.metrics["collisions"] = self.col_ref[0]
        self.metric_timer += 1
        if self.metric_timer >= 30:
            self.metrics.update(calc_metrics(self.drones, self.crowd))
            self.metric_timer = 0

    def _draw(self, elapsed: float) -> None:
        """Render the full frame."""
        self.screen.fill(C_BG)
        pygame.draw.rect(self.screen, C_PANEL_LINE, (PAD, TITLE_H + PAD, SIM_W, SIM_H), 1)

        draw_roads(self.screen)
        draw_zones(self.screen, self.fsm)
        self.crowd.draw(self.screen)
        draw_buildings(self.screen, self.fsm, self.fxs)

        for d in self.drones:
            d.draw(self.screen, self.fxs)

        draw_panel(self.screen, self.fh, self.fsm, self.fxs,
                   self.drones, self.crowd, self.metrics, elapsed, self.paused)
        draw_title(self.screen, self.ft, self.fxs)

        fps_t = self.fxs.render(f"FPS:{int(self.clock.get_fps())}", True, (55, 70, 100))
        self.screen.blit(fps_t, (PAD + 4, TITLE_H + SIM_H - 16))
        pygame.display.flip()

    def run(self) -> None:
        """Main loop — runs until ESC or window close."""
        running = True
        while running:
            running = self._handle_events()
            if not self.paused:
                self._update()
            elapsed = time.time() - self.sim_start
            self._draw(elapsed)
            self.clock.tick(FPS)
        pygame.quit()
