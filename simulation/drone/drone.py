"""
simulation/drone/drone.py
==========================
Drone: a single UAV agent with realistic physics and sensor uncertainty.

The Drone class delegates:
  - Path planning     → algorithms/path_planning.py  (drone.planner_fn)
  - Collision avoidance → algorithms/collision_avoidance.py (drone.avoidance_fn)
  - Comms / GPS / Wind  → algorithms/communication.py

To swap algorithms at runtime:
    from simulation.algorithms.path_planning import astar_planner
    drone.planner_fn = astar_planner

Physics model (per frame):
    desired_v  = direction_to_target * top_speed
    acceleration = clamp(desired_v - current_v, MAX_ACCEL)
    velocity   = (velocity + acceleration) * FRICTION
    heading    = heading + clamp(angle_to_velocity, MAX_TURN_RATE)
"""

import math
import time

import pygame

from simulation.config import (
    DRONE_NAMES, DRONE_COLORS,
    DRONE_R, MAX_SPEED, MAX_ACCEL, FRICTION, MAX_TURN_RATE,
    ARRIVE_D, COVERAGE_R, BATT_DRAIN,
    PAD, SIM_W, SIM_H, TITLE_H,
)
from simulation.environment.city_map import ZONES
from simulation.algorithms.collision_avoidance import rule_based_avoidance
from simulation.algorithms.path_planning       import follower_planner
from simulation.algorithms.communication       import CommBuffer, GPSSensor, WindModel


class Drone:
    """Single UAV agent.

    Attributes:
        idx:              drone index 0-3
        name:             display label (e.g. "UAV-A")
        color:            RGB tuple
        x, y:             current position (px)
        vx, vy:           velocity vector (px/frame)
        ax, ay:           acceleration vector (px/frame²)
        heading:          visual heading angle (rad), smoothly tracked
        tx, ty:           current target position
        battery:          remaining battery [0, 100]
        distance:         total distance flown (px)
        status:           "Tracking" | "On-Station" | "Avoiding" | "RTB"
        collision_avoids: count of avoidance manoeuvres
        planner_fn:       callable — path planning algorithm
        avoidance_fn:     callable — collision avoidance algorithm
    """

    def __init__(self, idx: int, crowd) -> None:
        self.idx   = idx
        self.name  = DRONE_NAMES[idx]
        self.color = DRONE_COLORS[idx]
        self.crowd = crowd

        # Position / motion
        self.x = float(ZONES[idx]["rect"].centerx)
        self.y = float(ZONES[idx]["rect"].centery)
        self.vx = self.vy = 0.0
        self.ax = self.ay = 0.0
        self.heading = 0.0

        # Target
        self.tx = self.x
        self.ty = self.y

        # Pluggable algorithms
        self.planner_fn   = follower_planner
        self.avoidance_fn = rule_based_avoidance

        # Leader-follower state (managed by LeaderFollowerSystem)
        self.is_leader   = (idx == 0)
        self.comm_active = True
        self.leader_ref  = None

        # Sensor / comms sub-systems
        self._comm   = CommBuffer(crowd.center)
        self._gps    = GPSSensor()
        self._wind   = WindModel()

        # Mission state
        self.battery          = 100.0
        self.distance         = 0.0
        self.status           = "Tracking"
        self.collision_avoids = 0
        
        # Metrics & Path Caching
        self.energy           = 0.0
        self.ideal_dist       = 0.0
        self.delay_caused     = 0
        self._path            = []

    def reset(self, crowd) -> None:
        """Full reset to initial state (called on SPACE key)."""
        self.crowd = crowd
        self.x = float(ZONES[self.idx]["rect"].centerx)
        self.y = float(ZONES[self.idx]["rect"].centery)
        self.vx = self.vy = 0.0
        self.ax = self.ay = 0.0
        self.heading = 0.0
        self.tx = self.x;  self.ty = self.y
        self._comm.reset(crowd.center)
        self._gps.reset()
        self._wind.reset()
        self.battery          = 100.0
        self.distance         = 0.0
        self.status           = "Tracking"
        self.collision_avoids = 0
        self.is_leader        = (self.idx == 0)
        self.comm_active      = True
        self.leader_ref       = None

        # Metrics & Path Caching
        self.energy           = 0.0
        self.ideal_dist       = 0.0
        self.delay_caused     = 0
        self._path            = []

    # ── Sensor accessors ──────────────────────────────────────────────────────

    @property
    def perceived_pos(self) -> tuple[float, float]:
        """GPS-noisy position used for navigation decisions."""
        ox, oy = self._gps.ox, self._gps.oy
        return self.x + ox, self.y + oy

    @property
    def wind_speed(self) -> float:
        return self._wind.speed()

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self, all_drones: list, col_ref: list,
               obstacles: list | None = None) -> None:
        """Advance drone by one simulation frame.

        Args:
            all_drones: full fleet list (for avoidance)
            col_ref:    mutable [int] shared avoidance event counter
            obstacles:  optional list of DynamicObstacle instances
        """
        # 1. Battery drain
        self.battery = max(0.0, self.battery - BATT_DRAIN)

        # 2. Sensor updates
        self._wind.update()
        self._gps.update()

        # 3. Comms-delayed crowd center → path planner → target
        delayed_center = self._comm.push(self.crowd.center)
        self.tx, self.ty = self.planner_fn(self, delayed_center)

        # 4. Physics: steer toward target
        px, py = self.perceived_pos
        dx = self.tx - px
        dy = self.ty - py
        d  = math.hypot(dx, dy)

        top_spd = MAX_SPEED * (0.5 if self.battery < 15 else 1.0)
        if d < ARRIVE_D:
            self.vx *= FRICTION
            self.vy *= FRICTION
            self.status = "On-Station"
        else:
            desired_vx = (dx / d) * top_spd
            desired_vy = (dy / d) * top_spd
            self.ax = desired_vx - self.vx
            self.ay = desired_vy - self.vy
            a_mag = math.hypot(self.ax, self.ay)
            if a_mag > MAX_ACCEL:
                self.ax = self.ax / a_mag * MAX_ACCEL
                self.ay = self.ay / a_mag * MAX_ACCEL
            self.vx = (self.vx + self.ax) * FRICTION
            self.vy = (self.vy + self.ay) * FRICTION
            spd = math.hypot(self.vx, self.vy)
            if spd > top_spd:
                self.vx = self.vx / spd * top_spd
                self.vy = self.vy / spd * top_spd
            self.status = "Tracking"

        # 5. Collision avoidance (drones + buildings + dynamic obstacles)
        dvx, dvy, avoiding = self.avoidance_fn(self, all_drones, col_ref, obstacles)
        self.vx += dvx
        self.vy += dvy
        if avoiding:
            self.status = "Avoiding"
            self.delay_caused += 1

        # 6. Wind disturbance
        wx, wy = self._wind.wx, self._wind.wy
        self.vx += wx
        self.vy += wy

        # 7. Battery depletion → return to base
        if self.battery <= 0:
            self.status = "RTB"
            self.vx *= 0.3
            self.vy *= 0.3

        # 8. Smooth heading
        _spd = math.hypot(self.vx, self.vy)
        if _spd > 0.05:
            th = math.atan2(self.vy, self.vx)
            dh = (th - self.heading + math.pi) % math.tau - math.pi
            self.heading = (
                self.heading + max(-MAX_TURN_RATE, min(MAX_TURN_RATE, dh))
            ) % math.tau

        # 9. Integrate position & metrics
        prev_x, prev_y = self.x, self.y
        self.x += self.vx
        self.y += self.vy
        self.x = max(PAD + DRONE_R, min(PAD + SIM_W - DRONE_R, self.x))
        self.y = max(TITLE_H + PAD + DRONE_R, min(TITLE_H + PAD + SIM_H - DRONE_R, self.y))
        
        move_dist = math.hypot(self.x - prev_x, self.y - prev_y)
        self.distance += move_dist
        
        # Energy: proportional to acceleration and speed
        self.energy += move_dist * 0.1 + math.hypot(self.ax, self.ay) * 0.5
        
        # Ideal dist: progress made directly toward the target
        if d > 0:
            ideal_move = ((self.x - prev_x) * (self.tx - prev_x) + (self.y - prev_y) * (self.ty - prev_y)) / d
            if ideal_move > 0:
                self.ideal_dist += ideal_move

    # ── Draw ──────────────────────────────────────────────────────────────────

    def draw(self, surf, font_xs) -> None:
        """Render the drone on the pygame surface."""
        from simulation.visualization.renderer import draw_dash
        ix, iy = int(self.x), int(self.y)

        # Dashed line to target
        draw_dash(surf, self.color, (ix, iy), (int(self.tx), int(self.ty)))

        # Quadrotor arm silhouette (4 arms at ±45° / ±135° from heading)
        arm_len = DRONE_R + 5
        dark = tuple(max(0, c - 70) for c in self.color)
        for ang in (
            self.heading + math.pi / 4,
            self.heading - math.pi / 4,
            self.heading + 3 * math.pi / 4,
            self.heading - 3 * math.pi / 4,
        ):
            ex2 = ix + int(math.cos(ang) * arm_len)
            ey2 = iy + int(math.sin(ang) * arm_len)
            pygame.draw.line(surf, dark, (ix, iy), (ex2, ey2), 2)
            pygame.draw.circle(surf, dark, (ex2, ey2), 4)

        # Body core
        pygame.draw.circle(surf, self.color, (ix, iy), DRONE_R - 2)
        pygame.draw.circle(surf, (200, 215, 255), (ix, iy), DRONE_R - 2, 2)

        # Heading direction arrow
        if math.hypot(self.vx, self.vy) > 0.05:
            ex = ix + int(math.cos(self.heading) * (DRONE_R + 7))
            ey = iy + int(math.sin(self.heading) * (DRONE_R + 7))
            pygame.draw.line(surf, (255, 255, 255), (ix, iy), (ex, ey), 2)

        # Sensor coverage ring
        s2 = pygame.Surface((COVERAGE_R * 2, COVERAGE_R * 2), pygame.SRCALPHA)
        pygame.draw.circle(s2, (*self.color, 18), (COVERAGE_R, COVERAGE_R), COVERAGE_R)
        surf.blit(s2, (ix - COVERAGE_R, iy - COVERAGE_R))
        pygame.draw.circle(surf, (*self.color, 80), (ix, iy), COVERAGE_R, 1)

        # Name label
        lbl = font_xs.render(self.name, True, self.color)
        surf.blit(lbl, (ix - lbl.get_width() // 2, iy - DRONE_R - 18))
