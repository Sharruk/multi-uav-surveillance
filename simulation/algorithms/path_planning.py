"""
simulation/algorithms/path_planning.py
========================================
Pluggable path-planning / target-assignment algorithms.

Current implementation: Coverage Offset Planner
  - 4 drones spread around the delayed crowd center with fixed offsets.

HOW TO SWITCH ALGORITHMS
--------------------------
Each planner is a function:
    plan(drone, delayed_crowd_center) -> (tx, ty)

Assign to a drone: drone.planner_fn = astar_planner
Or swap all drones at once in runner.py.

PLANNED ALGORITHMS (stubs provided)
--------------------------------------
- astar_planner          → grid-based A* waypoint planning
- dijkstra_planner       → Dijkstra shortest path
- rrt_planner            → Rapidly-exploring Random Tree
- potential_field_planner → attractive/repulsive field navigation
- qlearning_planner      → Q-learning reinforcement learning (placeholder)
"""

from simulation.config import DRONE_NAMES


# ── Coverage offset planner (active) ──────────────────────────────────────────

# Fixed formation offsets so each UAV covers a different quadrant
_COVERAGE_OFFSETS = [(-40, -40), (40, -40), (-40, 40), (40, 40)]


def coverage_offset_planner(drone, delayed_center: tuple[float, float]) -> tuple[float, float]:
    """Assign target as crowd-center + quadrant offset.

    Args:
        drone:          Drone instance (uses drone.idx for offset selection)
        delayed_center: (cx, cy) from the comms-delayed crowd position

    Returns:
        (tx, ty): target position
    """
    ox, oy = _COVERAGE_OFFSETS[drone.idx]
    cx, cy = delayed_center
    return cx + ox, cy + oy


# ── Stubs ──────────────────────────────────────────────────────────────────────

def astar_planner(drone, delayed_center):
    """[STUB] A* grid path planning toward crowd center."""
    raise NotImplementedError("A* planner not yet implemented")


def dijkstra_planner(drone, delayed_center):
    """[STUB] Dijkstra shortest-path planner."""
    raise NotImplementedError("Dijkstra planner not yet implemented")


def rrt_planner(drone, delayed_center):
    """[STUB] Rapidly-exploring Random Tree planner."""
    raise NotImplementedError("RRT planner not yet implemented")


def potential_field_planner(drone, delayed_center):
    """[STUB] Potential field navigation (attractive goal + repulsive obstacles)."""
    raise NotImplementedError("Potential field planner not yet implemented")


def qlearning_planner(drone, delayed_center):
    """[STUB] Q-learning reinforcement learning policy."""
    raise NotImplementedError("Q-learning planner not yet implemented")
