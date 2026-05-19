"""
simulation/drone/leader_follower.py
=====================================
LeaderFollowerSystem — manages leader election and follower coordination.

Rules:
  - UAV-A (idx 0) starts as leader.
  - If the current leader's battery drops below LEADER_LOW_BATT or its
    status is "RTB", a new leader is elected from remaining healthy drones
    (highest battery wins).
  - Each non-leader drone checks whether it is within COMM_RADIUS of the
    leader. If it is, it receives the leader's target (comm_active=True)
    and uses the follower formation planner.  If out of range, it falls
    back to the independent coverage-offset planner.

Visual indicators (handled in renderer.py):
  - Gold ring + "L" badge on current leader.
  - Semi-transparent COMM_RADIUS circle around leader.
  - Thin lines between UAVs within COMM_RADIUS of each other.
  - A brief yellow flash on the panel when a leader switch occurs.

Teammate note: drone/ module owner can extend election logic here
(e.g. signal-strength-based, role-rotation) without touching crowd code.
"""

import math

from simulation.config import COMM_RADIUS, LEADER_LOW_BATT


class LeaderFollowerSystem:
    """Manages leader election and per-drone comm/follower state."""

    def __init__(self, drones: list) -> None:
        self.leader_idx        = 0          # index into drones list
        self.switch_flash      = 0          # frames remaining for switch flash
        self.last_leader_name  = drones[0].name
        self._init_drone_attrs(drones)

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _init_drone_attrs(self, drones: list) -> None:
        """Add leader-follower attributes to drone objects."""
        for d in drones:
            d.is_leader   = (d.idx == self.leader_idx)
            d.comm_active = True
            d.leader_ref  = None

    def _elect_leader(self, drones: list) -> int:
        """Return idx of best candidate for leader."""
        healthy = [d for d in drones
                   if d.battery > LEADER_LOW_BATT and d.status != "RTB"]
        if not healthy:
            healthy = drones   # fallback: pick from anyone
        return max(healthy, key=lambda d: d.battery).idx

    # ── Public API ─────────────────────────────────────────────────────────────

    def reset(self, drones: list) -> None:
        """Reset to initial state (SPACE key)."""
        self.leader_idx       = 0
        self.switch_flash     = 0
        self.last_leader_name = drones[0].name
        self._init_drone_attrs(drones)

    def update(self, drones: list) -> None:
        """Evaluate leader health and update all drone coordination state.

        Call this once per frame before drone.update().
        """
        leader = drones[self.leader_idx]

        # Check if leader needs replacement
        if leader.battery <= LEADER_LOW_BATT or leader.status == "RTB":
            new_idx = self._elect_leader(drones)
            if new_idx != self.leader_idx:
                self.leader_idx       = new_idx
                self.switch_flash     = 240   # ~4 s flash at 60 fps
                self.last_leader_name = drones[new_idx].name

        if self.switch_flash > 0:
            self.switch_flash -= 1

        leader = drones[self.leader_idx]

        # Set per-drone flags
        for d in drones:
            d.is_leader  = (d.idx == self.leader_idx)
            d.leader_ref = leader
            if d.is_leader:
                d.comm_active = True
            else:
                dist = math.hypot(d.x - leader.x, d.y - leader.y)
                d.comm_active = (dist <= COMM_RADIUS)

    @property
    def leader_switched(self) -> bool:
        return self.switch_flash > 0

    def comm_links(self, drones: list) -> list[tuple]:
        """Return list of (drone_a, drone_b) pairs within COMM_RADIUS."""
        links = []
        for i, a in enumerate(drones):
            for b in drones[i + 1:]:
                if math.hypot(a.x - b.x, a.y - b.y) <= COMM_RADIUS:
                    links.append((a, b))
        return links
