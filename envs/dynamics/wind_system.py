"""
EnhancedWindSystem — configurable directional wind with building effects.

Improvements over the base WindSystem in environment_assets.py:
  - YAML-configurable base direction vector (not a random angle)
  - Height-dependent speed with configurable altitude_factor
  - Building wind-shadow: reduced force in the lee of tall buildings
  - Building edge turbulence: amplified gusts where flow separates
"""

import math
import numpy as np


class EnhancedWindSystem:
    """
    Wind physics with height scaling, gust cycles, turbulence,
    and optional building interaction effects.

    get_force() has the same signature as environment_assets.WindSystem
    so it can serve as a drop-in replacement in drone_env.py.
    Building effects are activated by calling set_buildings() after
    the scenario has spawned its building_specs.
    """

    def __init__(self, base_speed: float = 2.0,
                 direction=(1.0, 0.5, 0.0),
                 turbulence: float = 0.3,
                 altitude_factor: float = 1.5,
                 seed: int = None):
        self.base_speed     = float(base_speed)
        d    = np.array(direction[:2], dtype=np.float64)
        norm = float(np.linalg.norm(d))
        self.direction      = d / norm if norm > 1e-6 else np.array([1.0, 0.0])
        self.turbulence_std = float(turbulence) * self.base_speed
        self.altitude_factor = float(altitude_factor)
        self._rng           = np.random.default_rng(seed)
        self._gust_phase    = float(self._rng.uniform(0.0, 2.0 * math.pi))
        self._building_specs: list = []

    def set_buildings(self, building_specs: list):
        """Register building geometry for wind-shadow calculations."""
        self._building_specs = building_specs or []

    def get_force(self, position, step: int) -> tuple:
        """
        Wind force (fx, fy, fz) in Newtons at the given world position and step.
        Compatible with environment_assets.WindSystem.get_force().
        """
        z = max(float(position[2]), 0.1)
        x = float(position[0])
        y = float(position[1])

        # Height scaling (logarithmic profile approximation)
        height_mult = 1.0 + (self.altitude_factor - 1.0) * min(z / 10.0, 1.0)

        # Sinusoidal gust cycle (~5-second period at 50 Hz)
        gust = 1.0 + 0.25 * math.sin(step * 0.025 + self._gust_phase)

        effective = self.base_speed * height_mult * gust
        fx = effective * float(self.direction[0])
        fy = effective * float(self.direction[1])
        fz = 0.0

        # Gaussian turbulence
        fx += float(self._rng.standard_normal()) * self.turbulence_std * height_mult
        fy += float(self._rng.standard_normal()) * self.turbulence_std * height_mult
        fz += float(self._rng.standard_normal()) * self.turbulence_std * 0.1

        # Building interaction
        if self._building_specs:
            fx, fy, fz = self._apply_building_effects(x, y, z, fx, fy, fz)

        return (fx, fy, fz)

    def get_direction_degrees(self) -> float:
        return math.degrees(
            math.atan2(float(self.direction[1]), float(self.direction[0]))
        ) % 360.0

    # ── Internals ─────────────────────────────────────────────────────────────

    def _apply_building_effects(self, x: float, y: float, z: float,
                                 fx: float, fy: float, fz: float):
        downwind = -self.direction   # unit vector pointing downwind
        for spec in self._building_specs:
            cx = float(spec['center'][0])
            cy = float(spec['center'][1])
            hx = float(spec['half_extents'][0])
            hy = float(spec['half_extents'][1])
            bh = float(spec['half_extents'][2]) * 2.0
            if z > bh:
                continue
            dx   = x - cx
            dy   = y - cy
            d2   = math.sqrt(dx * dx + dy * dy)
            if d2 > 10.0:
                continue
            # Projection onto downwind axis (positive = downwind of building)
            proj = dx * float(downwind[0]) + dy * float(downwind[1])
            if 0.0 < proj < 6.0:
                # Lateral distance from the wind shadow centreline
                lateral    = abs(dx * float(self.direction[1]) -
                                 dy * float(self.direction[0]))
                shadow_w   = max(hx, hy) * 1.5
                if lateral < shadow_w:
                    shadow_f = max(0.2, lateral / shadow_w)
                    fx *= shadow_f
                    fy *= shadow_f
                    # Amplified gusts at the shadow edge
                    if shadow_w - lateral < 1.0:
                        extra = self.base_speed * 0.4
                        fx += float(self._rng.standard_normal()) * extra
                        fy += float(self._rng.standard_normal()) * extra
        return fx, fy, fz
