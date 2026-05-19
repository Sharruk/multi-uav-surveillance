"""
simulation/algorithms/collision_avoidance.py
==============================================
Pluggable collision avoidance algorithms.

Current implementation: Rule-Based Repulsion
  - Drone-drone: repulsion force when within AVOID_D_R
  - Building:    repulsion force when within AVOID_O_R

HOW TO ADD A NEW ALGORITHM
---------------------------
1. Create a function with the same signature as `rule_based_avoidance`.
2. Set it on a drone:  drone.avoidance_fn = my_new_fn
   (or swap globally in runner.py)

PLANNED ALGORITHMS (stubs provided)
--------------------------------------
- potential_field_avoidance()  → gradient-based repulsion field
- velocity_obstacle_avoidance() → VO / RVO for multi-agent
"""

import math
from simulation.config import (
    AVOID_D_R, AVOID_D_S,
    AVOID_O_R, AVOID_O_S,
)
from simulation.environment.city_map import BLDG_RECTS


def rule_based_avoidance(drone, all_drones: list, col_ref: list,
                         obstacles: list | None = None) -> tuple[float, float, bool]:
    """Apply rule-based repulsion forces.

    Args:
        drone:      the Drone instance to compute forces for
        all_drones: full list of Drone instances
        col_ref:    mutable [int] counter for collision avoidance events
        obstacles:  optional list of DynamicObstacle instances

    Returns:
        (dvx, dvy, avoiding): velocity delta and whether avoidance is active
    """
    dvx = dvy = 0.0
    avoiding = False

    # Drone-drone repulsion
    for o in all_drones:
        if o is drone:
            continue
        dd = math.hypot(drone.x - o.x, drone.y - o.y)
        if 0 < dd < AVOID_D_R:
            avoiding = True
            if drone.idx < o.idx:          # count each pair once
                col_ref[0] += 1
                drone.collision_avoids += 1
            s = (AVOID_D_R - dd) / AVOID_D_R * AVOID_D_S
            dvx -= s * (o.x - drone.x) / dd
            dvy -= s * (o.y - drone.y) / dd

    # Building repulsion
    for br in BLDG_RECTS:
        cx = max(br.left,  min(drone.x, br.right))
        cy = max(br.top,   min(drone.y, br.bottom))
        dd = math.hypot(drone.x - cx, drone.y - cy)
        if 0 < dd < AVOID_O_R:
            avoiding = True
            s = (AVOID_O_R - dd) / AVOID_O_R * AVOID_O_S
            dvx += s * (drone.x - cx) / dd
            dvy += s * (drone.y - cy) / dd

    # Dynamic obstacle repulsion (vehicles + construction zones)
    if obstacles:
        for obs in obstacles:
            if not getattr(obs, 'is_active', True):
                continue
            dd = math.hypot(drone.x - obs.x, drone.y - obs.y)
            avoid_r = obs.radius + 30     # extra buffer around dynamic obstacles
            if 0 < dd < avoid_r:
                avoiding = True
                s = (avoid_r - dd) / avoid_r * 3.5
                dvx += s * (drone.x - obs.x) / dd
                dvy += s * (drone.y - obs.y) / dd

    return dvx, dvy, avoiding


# ── Stubs for future algorithms ────────────────────────────────────────────────

def potential_field_avoidance(drone, all_drones: list, col_ref: list):
    """[STUB] Gradient-descent potential field avoidance."""
    raise NotImplementedError("potential_field_avoidance not yet implemented")


def velocity_obstacle_avoidance(drone, all_drones: list, col_ref: list):
    """[STUB] Velocity Obstacle (VO) / RVO multi-agent avoidance."""
    raise NotImplementedError("velocity_obstacle_avoidance not yet implemented")
