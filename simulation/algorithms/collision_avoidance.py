"""
simulation/algorithms/collision_avoidance.py
==============================================
Pluggable collision avoidance algorithms.

Current implementations:
- rule_based_avoidance: Simple distance-based repulsion.
- potential_field_avoidance: $1/d^2$ gradient repulsion from obstacles/drones.
- velocity_obstacle_avoidance (VO): Time-to-collision cone prediction.
- rvo_avoidance: Reciprocal Velocity Obstacle (shared responsibility).

HOW TO ADD A NEW ALGORITHM
---------------------------
1. Create a function with the same signature.
2. Set it on a drone:  drone.avoidance_fn = my_new_fn
"""

import math
from simulation.config import (
    AVOID_D_R, AVOID_D_S,
    AVOID_O_R, AVOID_O_S,
    AVOID_PF_S, AVOID_VO_TAU
)
from simulation.environment.city_map import BLDG_RECTS


def rule_based_avoidance(drone, all_drones: list, col_ref: list,
                         obstacles: list | None = None) -> tuple[float, float, bool]:
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

    # Dynamic obstacle repulsion
    if obstacles:
        for obs in obstacles:
            if not getattr(obs, 'is_active', True): continue
            dd = math.hypot(drone.x - obs.x, drone.y - obs.y)
            avoid_r = obs.radius + 30
            if 0 < dd < avoid_r:
                avoiding = True
                s = (avoid_r - dd) / avoid_r * 3.5
                dvx += s * (drone.x - obs.x) / dd
                dvy += s * (drone.y - obs.y) / dd

    return dvx, dvy, avoiding


def potential_field_avoidance(drone, all_drones: list, col_ref: list,
                              obstacles: list | None = None) -> tuple[float, float, bool]:
    """Gradient-descent potential field avoidance."""
    dvx = dvy = 0.0
    avoiding = False
    
    # Drone repulsion
    for o in all_drones:
        if o is drone: continue
        dd = math.hypot(drone.x - o.x, drone.y - o.y)
        if 0 < dd < AVOID_D_R * 1.5:
            avoiding = True
            if dd < AVOID_D_R and drone.idx < o.idx:
                col_ref[0] += 1
                drone.collision_avoids += 1
            s = AVOID_PF_S * (1.0/dd - 1.0/(AVOID_D_R*1.5)) * (1.0/(dd**2))
            dvx -= s * (o.x - drone.x) / dd
            dvy -= s * (o.y - drone.y) / dd

    # Building repulsion
    for br in BLDG_RECTS:
        cx = max(br.left,  min(drone.x, br.right))
        cy = max(br.top,   min(drone.y, br.bottom))
        dd = math.hypot(drone.x - cx, drone.y - cy)
        if 0 < dd < AVOID_O_R:
            avoiding = True
            s = AVOID_PF_S * 2.0 * (1.0/dd - 1.0/AVOID_O_R) * (1.0/(dd**2))
            dvx += s * (drone.x - cx) / dd
            dvy += s * (drone.y - cy) / dd
            
    # Dynamic obstacle repulsion
    if obstacles:
        for obs in obstacles:
            if not getattr(obs, 'is_active', True): continue
            dd = math.hypot(drone.x - obs.x, drone.y - obs.y)
            avoid_r = obs.radius + 30
            if 0 < dd < avoid_r:
                avoiding = True
                s = AVOID_PF_S * 1.5 * (1.0/dd - 1.0/avoid_r) * (1.0/(dd**2))
                dvx += s * (drone.x - obs.x) / dd
                dvy += s * (drone.y - obs.y) / dd

    # Cap max force to prevent explosion
    mag = math.hypot(dvx, dvy)
    if mag > 5.0:
        dvx = dvx / mag * 5.0
        dvy = dvy / mag * 5.0

    return dvx, dvy, avoiding


def _compute_vo(drone, all_drones, col_ref, obstacles, rvo=False):
    dvx = dvy = 0.0
    avoiding = False
    
    # 1. Building repulsion (fallback to rule-based for static)
    for br in BLDG_RECTS:
        cx = max(br.left,  min(drone.x, br.right))
        cy = max(br.top,   min(drone.y, br.bottom))
        dd = math.hypot(drone.x - cx, drone.y - cy)
        if 0 < dd < AVOID_O_R:
            avoiding = True
            s = (AVOID_O_R - dd) / AVOID_O_R * AVOID_O_S
            dvx += s * (drone.x - cx) / dd
            dvy += s * (drone.y - cy) / dd

    # 2. VO against drones
    for o in all_drones:
        if o is drone: continue
        dx = o.x - drone.x
        dy = o.y - drone.y
        dist = math.hypot(dx, dy)
        
        if 0 < dist < 120:
            rvx = o.vx - drone.vx
            rvy = o.vy - drone.vy
            rv_sq = rvx**2 + rvy**2
            
            if rv_sq > 0.001:
                t_cpa = -(dx * rvx + dy * rvy) / rv_sq
                if 0 < t_cpa < AVOID_VO_TAU:
                    cpa_x = dx + rvx * t_cpa
                    cpa_y = dy + rvy * t_cpa
                    d_cpa = math.hypot(cpa_x, cpa_y)
                    
                    if d_cpa < AVOID_D_R:
                        avoiding = True
                        if drone.idx < o.idx and dist < AVOID_D_R:
                            col_ref[0] += 1
                            drone.collision_avoids += 1
                            
                        if d_cpa > 0.1:
                            nx, ny = cpa_x / d_cpa, cpa_y / d_cpa
                        else:
                            nx, ny = -dx / dist, -dy / dist
                            
                        mag = (AVOID_D_R - d_cpa) / t_cpa
                        if rvo: mag *= 0.5
                        
                        dvx -= nx * mag
                        dvy -= ny * mag

    # 3. Dynamic obstacles
    if obstacles:
        for obs in obstacles:
            if not getattr(obs, 'is_active', True): continue
            dx = obs.x - drone.x
            dy = obs.y - drone.y
            dist = math.hypot(dx, dy)
            avoid_r = obs.radius + 30
            
            if 0 < dist < 120:
                rvx = getattr(obs, 'vx', 0) - drone.vx
                rvy = getattr(obs, 'vy', 0) - drone.vy
                rv_sq = rvx**2 + rvy**2
                if rv_sq > 0.001:
                    t_cpa = -(dx * rvx + dy * rvy) / rv_sq
                    if 0 < t_cpa < AVOID_VO_TAU:
                        cpa_x = dx + rvx * t_cpa
                        cpa_y = dy + rvy * t_cpa
                        d_cpa = math.hypot(cpa_x, cpa_y)
                        
                        if d_cpa < avoid_r:
                            avoiding = True
                            nx, ny = (cpa_x / d_cpa, cpa_y / d_cpa) if d_cpa > 0.1 else (-dx/dist, -dy/dist)
                            mag = (avoid_r - d_cpa) / t_cpa
                            dvx -= nx * mag
                            dvy -= ny * mag

    return dvx, dvy, avoiding

def velocity_obstacle_avoidance(drone, all_drones: list, col_ref: list,
                                obstacles: list | None = None) -> tuple[float, float, bool]:
    """Velocity Obstacle (VO) prediction avoidance."""
    return _compute_vo(drone, all_drones, col_ref, obstacles, rvo=False)


def rvo_avoidance(drone, all_drones: list, col_ref: list,
                  obstacles: list | None = None) -> tuple[float, float, bool]:
    """Reciprocal Velocity Obstacle (RVO) avoidance."""
    return _compute_vo(drone, all_drones, col_ref, obstacles, rvo=True)
