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
from simulation.crowd.crowd_system                  import CrowdSystem
from simulation.drone.drone                         import Drone
from simulation.drone.leader_follower               import LeaderFollowerSystem
from simulation.environment.dynamic_obstacles       import create_obstacles, reset_obstacles
from simulation.metrics.analytics                   import calc_metrics
from simulation.visualization.renderer             import (
    draw_roads, draw_buildings, draw_zones, draw_title,
    draw_obstacles, draw_comm_links, draw_paths
)
from simulation.visualization.panel                import draw_panel

from simulation.algorithms.collision_avoidance import (
    rule_based_avoidance, potential_field_avoidance, 
    velocity_obstacle_avoidance, rvo_avoidance
)
from simulation.algorithms.path_planning import (
    coverage_offset_planner, follower_planner, astar_planner, 
    dijkstra_planner, potential_field_planner, qlearning_planner, hybrid_planner
)


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
        self.crowd     = CrowdSystem()
        self.drones    = [Drone(i, self.crowd) for i in range(self.NUM_DRONES)]
        self.obstacles = create_obstacles()
        self.lf_system = LeaderFollowerSystem(self.drones)
        self.col_ref   = [0]
        self.metrics   = {
            "collisions":  0,
            "coverage":    0.0,
            "accuracy":    100.0,
            "total_dist":  0.0,
            "wind_spd":    0.0,
        }
        self.sim_start    = time.time()
        self.paused       = False
        self.metric_timer = 0
        self.event_log    = ["Simulation started"]
        self.active_avoidance = "Rule-based"
        self.active_planner   = "Follower Planner"

    def reset(self) -> None:
        """Full simulation reset (SPACE key)."""
        self.crowd = CrowdSystem()
        for d in self.drones:
            d.reset(self.crowd)
        reset_obstacles(self.obstacles)
        self.lf_system.reset(self.drones)
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
                
                # Collision Avoidance Hot-swaps
                if ev.key == pygame.K_1:
                    self._set_avoidance("Rule-based", rule_based_avoidance)
                elif ev.key == pygame.K_2:
                    self._set_avoidance("Potential Field", potential_field_avoidance)
                elif ev.key == pygame.K_3:
                    self._set_avoidance("Velocity Obstacle", velocity_obstacle_avoidance)
                elif ev.key == pygame.K_4:
                    self._set_avoidance("RVO", rvo_avoidance)
                
                # Path Planning Hot-swaps
                elif ev.key == pygame.K_5:
                    self._set_planner("Follower Planner", follower_planner)
                elif ev.key == pygame.K_6:
                    self._set_planner("A* Planner", astar_planner)
                elif ev.key == pygame.K_7:
                    self._set_planner("Dijkstra Planner", dijkstra_planner)
                elif ev.key == pygame.K_8:
                    self._set_planner("Potential Field Planner", potential_field_planner)
                elif ev.key == pygame.K_9:
                    self._set_planner("Hybrid Planner", hybrid_planner)
                elif ev.key == pygame.K_0:
                    self._set_planner("Q-Learning", qlearning_planner)
        return True

    def _set_avoidance(self, name: str, fn) -> None:
        if self.active_avoidance != name:
            self.active_avoidance = name
            for d in self.drones:
                d.avoidance_fn = fn
            self.event_log.append(f"Avoidance -> {name}")

    def _set_planner(self, name: str, fn) -> None:
        if self.active_planner != name:
            self.active_planner = name
            for d in self.drones:
                d.planner_fn = fn
            self.event_log.append(f"Planner -> {name}")

    def _update(self) -> None:
        """Advance all simulation subsystems by one frame."""
        # 1. Leader-follower: elect / update flags before drones move
        self.lf_system.update(self.drones)
        if self.lf_system.switch_flash == 240: # Just switched
            self.event_log.append(f"Leader switched to {self.drones[self.lf_system.leader_idx].name}")
            
        # 2. Crowd + environment
        self.crowd.update(self.drones)
        for obs in self.obstacles:
            obs.update()
        # 3. Drone physics + avoidance
        for d in self.drones:
            d.update(self.drones, self.col_ref, self.obstacles)
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
        draw_obstacles(self.screen, self.obstacles)       # vehicles + construction
        self.crowd.draw(self.screen)
        draw_buildings(self.screen, self.fsm, self.fxs)
        draw_comm_links(self.screen, self.drones, self.lf_system)   # comm network
        draw_paths(self.screen, self.drones)                        # planned paths

        for d in self.drones:
            d.draw(self.screen, self.fxs)

        draw_panel(self.screen, self.fh, self.fsm, self.fxs,
                   self.drones, self.crowd, self.metrics, elapsed,
                   self.paused, self.lf_system,
                   self.active_avoidance, self.active_planner,
                   self.event_log)
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
