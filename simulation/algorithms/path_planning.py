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

from simulation.config import DRONE_NAMES, FORMATION_OFFSETS


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


def follower_planner(drone, delayed_center: tuple[float, float]) -> tuple[float, float]:
    """Leader-follower planner.

    - Leader drones use coverage_offset_planner (navigate toward crowd).
    - Followers within COMM_RADIUS of leader track the leader's position
      with a small formation offset so they cluster around it.
    - Followers outside COMM_RADIUS fall back to independent navigation.

    Args:
        drone:          Drone instance (checks drone.is_leader, drone.comm_active,
                        drone.leader_ref)
        delayed_center: comms-delayed crowd centre (used for fallback)

    Returns:
        (tx, ty): target position
    """
    is_leader   = getattr(drone, 'is_leader',   True)
    comm_active = getattr(drone, 'comm_active',  True)
    leader_ref  = getattr(drone, 'leader_ref',   None)

    if is_leader or not comm_active or leader_ref is None:
        # Independent navigation
        return coverage_offset_planner(drone, delayed_center)

    # Follow leader with formation offset
    ox, oy = FORMATION_OFFSETS[drone.idx]
    return leader_ref.x + ox, leader_ref.y + oy


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
