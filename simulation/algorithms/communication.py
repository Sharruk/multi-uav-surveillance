"""
simulation/algorithms/communication.py
========================================
Communication and sensor uncertainty models.

Models:
  - Comms delay:  target positions are buffered and delivered N frames late
  - GPS noise:    Gaussian positional offset applied to perceived position
  - Wind drift:   random force applied to velocity each frame

These are intentionally separated from the Drone class so that
communication models can be swapped or disabled for algorithm comparison.
"""

import random
import math
from collections import deque

from simulation.config import COMM_DELAY, GPS_NOISE, WIND_STR


class CommBuffer:
    """Comms-delay buffer: stores crowd-center observations and delivers them
    after COMM_DELAY frames."""

    def __init__(self, initial_center: tuple[float, float]) -> None:
        self._buf: deque = deque(maxlen=COMM_DELAY)
        for _ in range(COMM_DELAY):
            self._buf.append(initial_center)

    def push(self, center: tuple[float, float]) -> tuple[float, float]:
        """Push new observation; return the oldest (delayed) observation."""
        self._buf.append(center)
        return self._buf[0]

    def reset(self, initial_center: tuple[float, float]) -> None:
        self._buf.clear()
        for _ in range(COMM_DELAY):
            self._buf.append(initial_center)


class GPSSensor:
    """Simulates GPS noise: returns a Gaussian offset refreshed periodically."""

    def __init__(self) -> None:
        self.ox    = 0.0
        self.oy    = 0.0
        self._timer = 0

    def update(self) -> tuple[float, float]:
        """Return (offset_x, offset_y) for the current frame."""
        self._timer -= 1
        if self._timer <= 0:
            self.ox = random.gauss(0, GPS_NOISE)
            self.oy = random.gauss(0, GPS_NOISE)
            self._timer = random.randint(5, 15)
        return self.ox, self.oy

    def reset(self) -> None:
        self.ox = self.oy = 0.0
        self._timer = 0


class WindModel:
    """Simulates slowly-varying wind drift force."""

    def __init__(self) -> None:
        self.wx    = random.uniform(-WIND_STR, WIND_STR)
        self.wy    = random.uniform(-WIND_STR, WIND_STR)
        self._timer = 0

    def update(self) -> tuple[float, float]:
        """Return (wind_vx, wind_vy) force for the current frame."""
        self._timer -= 1
        if self._timer <= 0:
            self.wx += random.uniform(-0.02, 0.02)
            self.wy += random.uniform(-0.02, 0.02)
            self.wx = max(-WIND_STR, min(WIND_STR, self.wx))
            self.wy = max(-WIND_STR, min(WIND_STR, self.wy))
            self._timer = random.randint(20, 60)
        return self.wx, self.wy

    def speed(self) -> float:
        return math.hypot(self.wx, self.wy)

    def reset(self) -> None:
        self.wx = random.uniform(-WIND_STR, WIND_STR)
        self.wy = random.uniform(-WIND_STR, WIND_STR)
        self._timer = 0
