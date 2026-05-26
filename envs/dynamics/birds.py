"""
BoidBirds — kinematic flying obstacles with boids flocking algorithm.

Rules applied per-bird each step:
  Separation  — steer away from near neighbours  (radius 2 m)
  Alignment   — match average velocity of flock   (radius 4 m)
  Cohesion    — steer toward flock centre of mass  (radius 6 m)
  Boundary    — soft wall repulsion at arena edge
  Altitude    — spring toward target altitude band
"""

import math
import numpy as np
import pybullet as p


class BoidBirds:
    """
    N flying birds whose positions are updated each step via boids steering.
    Bodies are collidable spheres so LiDAR can detect them as moving obstacles.
    """

    # Boids weights
    W_SEP, W_ALI, W_COH = 2.0, 1.0, 0.8
    W_BND, W_ALT        = 1.5, 1.2

    # Boids radii (meters)
    SEP_R, ALI_R, COH_R = 2.0, 4.0, 6.0

    MAX_SPEED = 4.0   # m/s XY
    MAX_STEER = 2.0   # m/s² steering clamp

    _COLORS = [
        [0.22, 0.18, 0.12, 1.0],   # dark brown
        [0.30, 0.25, 0.18, 1.0],   # warm brown
        [0.32, 0.32, 0.32, 1.0],   # charcoal
    ]

    def __init__(self, client_id: int, num_birds: int = 8,
                 altitude_range=(3.0, 7.0), building_specs=None,
                 rng: np.random.Generator = None):
        self.client_id  = client_id
        self.n          = num_birds
        self.alt_lo     = float(altitude_range[0])
        self.alt_hi     = float(altitude_range[1])
        self.building_specs = building_specs or []
        self.rng        = rng or np.random.default_rng(42)

        self.positions  = np.zeros((num_birds, 3), dtype=np.float64)
        self.velocities = np.zeros((num_birds, 3), dtype=np.float64)
        self.body_ids: list = []

    # ── Public API ────────────────────────────────────────────────────────────

    def spawn(self, arena: float = 9.0) -> list:
        """Spawn bird bodies, return list of PyBullet body IDs."""
        for i in range(self.n):
            x = float(self.rng.uniform(-arena * 0.75, arena * 0.75))
            y = float(self.rng.uniform(-arena * 0.75, arena * 0.75))
            z = float(self.rng.uniform(self.alt_lo, self.alt_hi))
            self.positions[i] = [x, y, z]
            angle = float(self.rng.uniform(0.0, 2.0 * math.pi))
            spd   = float(self.rng.uniform(2.0, self.MAX_SPEED))
            self.velocities[i] = [spd * math.cos(angle), spd * math.sin(angle), 0.0]

            color = list(self._COLORS[int(self.rng.integers(0, len(self._COLORS)))])
            vs  = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=[0.15, 0.06, 0.04],
                                      rgbaColor=color,
                                      physicsClientId=self.client_id)
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.12,
                                          physicsClientId=self.client_id)
            bid = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=col,
                                    baseVisualShapeIndex=vs,
                                    basePosition=[x, y, z],
                                    physicsClientId=self.client_id)
            self.body_ids.append(bid)
        return self.body_ids

    def update(self, dt: float = 0.02, arena: float = 9.0):
        """Apply boids rules and push updated positions to PyBullet."""
        n      = self.n
        new_v  = self.velocities.copy()

        for i in range(n):
            pi = self.positions[i]
            vi = self.velocities[i]

            sep = np.zeros(3)
            ali = np.zeros(3)
            coh = np.zeros(3)
            ns = na = nc = 0

            for j in range(n):
                if i == j:
                    continue
                diff = self.positions[j] - pi
                d    = float(np.linalg.norm(diff))
                if d < self.SEP_R and d > 1e-6:
                    sep -= diff / d
                    ns  += 1
                if d < self.ALI_R:
                    ali += self.velocities[j]
                    na  += 1
                if d < self.COH_R:
                    coh += self.positions[j]
                    nc  += 1

            steer = np.zeros(3)
            if ns:
                steer += self.W_SEP * self._limit(sep / ns, self.MAX_STEER)
            if na:
                steer += self.W_ALI * self._limit(ali / na - vi, self.MAX_STEER)
            if nc:
                toward = coh / nc - pi
                if np.linalg.norm(toward) > 1e-6:
                    steer += self.W_COH * self._limit(
                        self._limit(toward, self.MAX_SPEED) - vi, self.MAX_STEER)

            # Soft boundary walls
            bnd = arena * 0.75
            for dim in (0, 1):
                if pi[dim] > bnd:
                    steer[dim] -= self.W_BND * (pi[dim] - bnd)
                elif pi[dim] < -bnd:
                    steer[dim] += self.W_BND * (-bnd - pi[dim])

            # Altitude spring
            mid_alt    = (self.alt_lo + self.alt_hi) * 0.5
            steer[2]  += self.W_ALT * (mid_alt - float(pi[2]))

            nv = vi + steer * dt
            # Clamp XY speed
            xy_spd = math.sqrt(float(nv[0])**2 + float(nv[1])**2)
            if xy_spd > self.MAX_SPEED:
                scale = self.MAX_SPEED / xy_spd
                nv[0] *= scale
                nv[1] *= scale
            nv[2] = float(np.clip(nv[2], -1.0, 1.0))
            new_v[i] = nv

        self.velocities  = new_v
        self.positions  += self.velocities * dt
        self.positions[:, 2] = np.clip(self.positions[:, 2], self.alt_lo, self.alt_hi)

        # Sync PyBullet bodies, oriented toward velocity
        for i, bid in enumerate(self.body_ids):
            pos = [float(self.positions[i, 0]),
                   float(self.positions[i, 1]),
                   float(self.positions[i, 2])]
            vx, vy = float(self.velocities[i, 0]), float(self.velocities[i, 1])
            xy_spd = math.sqrt(vx * vx + vy * vy)
            if xy_spd > 0.1:
                yaw = math.atan2(vy, vx)
                orn = p.getQuaternionFromEuler([0.0, 0.0, yaw],
                                               physicsClientId=self.client_id)
            else:
                orn = [0.0, 0.0, 0.0, 1.0]
            p.resetBasePositionAndOrientation(bid, pos, orn,
                                              physicsClientId=self.client_id)

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _limit(v: np.ndarray, max_len: float) -> np.ndarray:
        length = float(np.linalg.norm(v))
        if length > max_len and length > 1e-6:
            return v * (max_len / length)
        return v
