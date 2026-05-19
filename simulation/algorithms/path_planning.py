"""
simulation/algorithms/path_planning.py
========================================
Pluggable path-planning / target-assignment algorithms.

Current implementations:
- coverage_offset_planner: Simple vector to offset of crowd center.
- follower_planner: Follows leader or uses coverage offset.
- astar_planner: Grid-based A* around buildings.
- dijkstra_planner: Grid-based Dijkstra around buildings.
- potential_field_planner: Attractive to target, repulsive from buildings.
- hybrid_planner: A* for global path, Potential Field for local steering.
- qlearning_planner: Placeholder stub.

HOW TO SWITCH ALGORITHMS
--------------------------
Each planner is a function:
    plan(drone, delayed_crowd_center) -> (tx, ty)
"""

import math
import heapq
import random

from simulation.config import (
    DRONE_NAMES, FORMATION_OFFSETS, GRID_CELL_SIZE,
    PAD, SIM_W, SIM_H, TITLE_H, PF_ATTRACT_S, PF_REPEL_S
)
from simulation.environment.city_map import in_building, BLDG_RECTS

# ── Coverage offset planner (active) ──────────────────────────────────────────

_COVERAGE_OFFSETS = [(-40, -40), (40, -40), (-40, 40), (40, 40)]

def coverage_offset_planner(drone, delayed_center: tuple[float, float]) -> tuple[float, float]:
    ox, oy = _COVERAGE_OFFSETS[drone.idx]
    cx, cy = delayed_center
    return cx + ox, cy + oy

def follower_planner(drone, delayed_center: tuple[float, float]) -> tuple[float, float]:
    is_leader   = getattr(drone, 'is_leader',   True)
    comm_active = getattr(drone, 'comm_active',  True)
    leader_ref  = getattr(drone, 'leader_ref',   None)

    if is_leader or not comm_active or leader_ref is None:
        return coverage_offset_planner(drone, delayed_center)

    ox, oy = FORMATION_OFFSETS[drone.idx]
    return leader_ref.x + ox, leader_ref.y + oy

# ── Grid & Graph for Search Algorithms ────────────────────────────────────────

_GRID = None

def _get_grid():
    global _GRID
    if _GRID is not None:
        return _GRID
    
    _GRID = {"nodes": set(), "neighbors": {}}
    # Build a coarse grid
    xs = list(range(PAD + GRID_CELL_SIZE//2, PAD + SIM_W, GRID_CELL_SIZE))
    ys = list(range(TITLE_H + GRID_CELL_SIZE//2, TITLE_H + SIM_H, GRID_CELL_SIZE))
    
    for x in xs:
        for y in ys:
            if not in_building(x, y):
                _GRID["nodes"].add((x, y))
                
    for (x, y) in _GRID["nodes"]:
        _GRID["neighbors"][(x, y)] = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = x + dx * GRID_CELL_SIZE, y + dy * GRID_CELL_SIZE
            if (nx, ny) in _GRID["nodes"]:
                _GRID["neighbors"][(x, y)].append((nx, ny))
                
    return _GRID

def _closest_node(x, y):
    grid = _get_grid()
    if not grid["nodes"]:
        return x, y
    return min(grid["nodes"], key=lambda n: math.hypot(n[0] - x, n[1] - y))

def _compute_path(start, goal, use_heuristic=True):
    grid = _get_grid()
    start_node = _closest_node(start[0], start[1])
    goal_node = _closest_node(goal[0], goal[1])
    
    if start_node == goal_node:
        return [goal]
        
    frontier = []
    heapq.heappush(frontier, (0, start_node))
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal_node:
            break
            
        for next_node in grid["neighbors"][current]:
            new_cost = cost_so_far[current] + math.hypot(next_node[0]-current[0], next_node[1]-current[1])
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost
                if use_heuristic:
                    priority += math.hypot(goal_node[0]-next_node[0], goal_node[1]-next_node[1])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                
    if goal_node not in came_from:
        return [goal] # No path found
        
    path = []
    curr = goal_node
    while curr != start_node:
        path.append(curr)
        curr = came_from[curr]
    path.reverse()
    path.append(goal) # Exact goal at the end
    return path

def _manage_path_cache(drone, target, use_heuristic=True):
    """Computes and updates drone._path towards target."""
    if not hasattr(drone, '_path'):
        drone._path = []
        
    # Replan if no path or target moved significantly from the end of current path
    replan = False
    if not drone._path:
        replan = True
    else:
        end_x, end_y = drone._path[-1]
        if math.hypot(target[0] - end_x, target[1] - end_y) > GRID_CELL_SIZE * 2:
            replan = True
            
    if replan:
        drone._path = _compute_path((drone.x, drone.y), target, use_heuristic)
        
    # Advance waypoints
    if drone._path:
        wx, wy = drone._path[0]
        if math.hypot(drone.x - wx, drone.y - wy) < GRID_CELL_SIZE:
            drone._path.pop(0)
            
    if drone._path:
        return drone._path[0]
    return target

# ── A* and Dijkstra ──────────────────────────────────────────────────────────

def astar_planner(drone, delayed_center):
    target = coverage_offset_planner(drone, delayed_center)
    return _manage_path_cache(drone, target, use_heuristic=True)

def dijkstra_planner(drone, delayed_center):
    target = coverage_offset_planner(drone, delayed_center)
    return _manage_path_cache(drone, target, use_heuristic=False)

# ── Potential Field ──────────────────────────────────────────────────────────

def potential_field_planner(drone, delayed_center):
    """Attractive goal + repulsive obstacles as a navigation target."""
    target = coverage_offset_planner(drone, delayed_center)
    
    # Attractive force towards target
    dx = target[0] - drone.x
    dy = target[1] - drone.y
    d = math.hypot(dx, dy)
    
    fx = dx * PF_ATTRACT_S
    fy = dy * PF_ATTRACT_S
    
    # Repulsive force from buildings
    for br in BLDG_RECTS:
        cx = max(br.left,  min(drone.x, br.right))
        cy = max(br.top,   min(drone.y, br.bottom))
        dist = math.hypot(drone.x - cx, drone.y - cy)
        
        if 0 < dist < 80: # field of influence
            s = PF_REPEL_S * (1.0/dist - 1.0/80) * (1.0/(dist**2))
            fx += s * (drone.x - cx) / dist
            fy += s * (drone.y - cy) / dist
            
    # The output is a virtual target a short distance ahead
    return drone.x + fx * 20, drone.y + fy * 20

# ── Hybrid ───────────────────────────────────────────────────────────────────

def hybrid_planner(drone, delayed_center):
    """A* for global waypoints, Potential Field for local steering."""
    # 1. Get A* waypoint
    target = coverage_offset_planner(drone, delayed_center)
    waypoint = _manage_path_cache(drone, target, use_heuristic=True)
    
    # 2. Apply Potential Field towards the waypoint
    dx = waypoint[0] - drone.x
    dy = waypoint[1] - drone.y
    d = math.hypot(dx, dy)
    
    fx = dx * PF_ATTRACT_S * 2
    fy = dy * PF_ATTRACT_S * 2
    
    for br in BLDG_RECTS:
        cx = max(br.left,  min(drone.x, br.right))
        cy = max(br.top,   min(drone.y, br.bottom))
        dist = math.hypot(drone.x - cx, drone.y - cy)
        
        if 0 < dist < 60:
            s = PF_REPEL_S * (1.0/dist - 1.0/60) * (1.0/(dist**2))
            fx += s * (drone.x - cx) / dist
            fy += s * (drone.y - cy) / dist
            
    return drone.x + fx * 15, drone.y + fy * 15

# ── Q-Learning (Placeholder) ─────────────────────────────────────────────────

def qlearning_planner(drone, delayed_center):
    """[STUB] Simulates a learned policy with random exploration logic."""
    target = coverage_offset_planner(drone, delayed_center)
    # Just add some random jitter to simulate exploration
    jx = random.uniform(-20, 20)
    jy = random.uniform(-20, 20)
    return target[0] + jx, target[1] + jy
