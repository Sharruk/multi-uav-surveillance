"""
MetricsDashboard — real-time simulation monitoring for STIRS-2025.

Tracks FPS, drone states, crowd analytics, environment conditions, and
research metrics. Supports three output modes:

  console  — human-readable table printed to stdout each call
  overlay  — PyBullet addUserDebugText labels in the GUI window
  silent   — accumulate only, no output (use get_summary() later)

Usage:
    dash = MetricsDashboard("downtown")
    # inside step loop:
    dash.tick(env, step_start_time)
    dash.display_console()          # or dash.display_overlay(client_id)
    # after episode:
    print(dash.get_summary())
"""

import time
import math
import collections


class MetricsDashboard:

    def __init__(self, scenario_name: str, history_len: int = 100):
        self.scenario      = scenario_name
        self._history_len  = history_len

        # FPS tracking
        self._frame_times: collections.deque = collections.deque(maxlen=history_len)
        self._step_start: float = 0.0
        self.fps_current: float = 0.0
        self.fps_avg:     float = 0.0
        self.fps_min:     float = float('inf')
        self.fps_max:     float = 0.0

        # Drone state cache
        self.drone_states: dict = {}   # drone_id → {pos, vel, action}

        # Crowd analytics
        self.crowd_total:  int  = 0
        self.crowd_states: dict = {}   # state_name → count

        # Environment status
        self.wind_vec:    tuple = (0.0, 0.0, 0.0)
        self.bird_count:  int   = 0
        self.step_count:  int   = 0

        # Research metrics
        self.collision_total: int   = 0
        self.collision_events: list = []   # list of {step, obj1, obj2, pos}
        self._coverage_cells_seen: set = set()
        self._total_coverage_cells: int = 0

        # Overlay text handle IDs (for clearing previous frame's text)
        self._overlay_ids: list = []

    # ── Public API ─────────────────────────────────────────────────────────────

    def tick(self, env, step_start_time: float):
        """
        Update all metrics from the current environment state.

        Call this immediately after env.step() returns, passing the timestamp
        recorded just before the step call:

            t0 = time.perf_counter()
            obs, rew, term, trunc, info = env.step(actions)
            dash.tick(env, t0)
        """
        elapsed = time.perf_counter() - step_start_time
        if elapsed > 0:
            fps = 1.0 / elapsed
            self._frame_times.append(fps)
            self.fps_current = fps
            self.fps_avg     = sum(self._frame_times) / len(self._frame_times)
            self.fps_min     = min(self.fps_min, fps)
            self.fps_max     = max(self.fps_max, fps)

        self.step_count += 1

        # Drone states
        self.drone_states = {}
        for drone_id in env.agents:
            drone = env.drones.get(drone_id) if hasattr(env, 'drones') else None
            pos = list(env._get_drone_position(drone_id)) if hasattr(env, '_get_drone_position') else [0, 0, 0]
            self.drone_states[drone_id] = {'pos': pos}

        # Crowd analytics
        if env.crowd_sim is not None:
            agents = env.crowd_sim.agents
            self.crowd_total = len(agents)
            states = collections.Counter(a['state'] for a in agents)
            self.crowd_states = dict(states)
        else:
            self.crowd_total  = 0
            self.crowd_states = {}

        # Birds
        if env.boid_birds is not None:
            self.bird_count = len(env.boid_birds.body_ids)
        else:
            self.bird_count = 0

    def log_collision(self, obj1: str, obj2: str, position: tuple):
        """Record a collision event."""
        self.collision_total += 1
        self.collision_events.append({
            'step':  self.step_count,
            'obj1':  obj1,
            'obj2':  obj2,
            'pos':   position,
        })

    def display_console(self):
        """Print a compact metrics table to stdout."""
        bar    = '-' * 58
        fps_ok = 'OK' if self.fps_current >= 100 else 'LOW'
        lines  = [
            bar,
            f"  STIRS-2025 | {self.scenario:<14}  step {self.step_count:>6}",
            bar,
            f"  FPS  current {self.fps_current:6.1f}  avg {self.fps_avg:6.1f}"
            f"  min {self.fps_min:6.1f}  max {self.fps_max:6.1f}  [{fps_ok}]",
            f"  Crowd  {self.crowd_total:3d} agents  "
            + '  '.join(f"{s}:{n}" for s, n in self.crowd_states.items()),
            f"  Birds  {self.bird_count:2d}",
            f"  Collisions  {self.collision_total}",
        ]
        if self.drone_states:
            lines.append(f"  Drones ({len(self.drone_states)}):")
            for did, ds in self.drone_states.items():
                p = ds['pos']
                lines.append(f"    {did}  pos=({p[0]:5.1f},{p[1]:5.1f},{p[2]:4.1f})")
        lines.append(bar)
        print('\n'.join(lines))

    def display_overlay(self, client_id: int):
        """
        Render metrics as PyBullet debug text in the GUI window.

        Text is positioned in world space at fixed coordinates near the
        arena origin; adjust positions if your city layout differs.
        Only meaningful in GUI (non-headless) mode.
        """
        try:
            import pybullet as p
        except ImportError:
            return

        # Remove previous frame's text
        for tid in self._overlay_ids:
            try:
                p.removeUserDebugItem(tid, physicsClientId=client_id)
            except Exception:
                pass
        self._overlay_ids = []

        fps_color = [0.2, 1.0, 0.2] if self.fps_current >= 100 else [1.0, 0.4, 0.1]

        def _add(text, pos, color, size=0.9):
            tid = p.addUserDebugText(
                text, pos, textColorRGB=color,
                textSize=size, lifeTime=0.12,
                physicsClientId=client_id)
            self._overlay_ids.append(tid)

        # Top-left area (high above city, visible from any angle)
        _add(f"Scenario: {self.scenario}   Step: {self.step_count}",
             [-12, 12, 18], [1.0, 1.0, 1.0], size=1.0)

        _add(f"FPS: {self.fps_current:.1f}  avg {self.fps_avg:.1f}  "
             f"min {self.fps_min:.0f}  max {self.fps_max:.0f}",
             [-12, 12, 16.5], fps_color, size=0.95)

        crowd_str = f"Crowd: {self.crowd_total}"
        if self.crowd_states:
            crowd_str += '  ' + '  '.join(f"{s[0].upper()}:{n}"
                                           for s, n in self.crowd_states.items())
        _add(crowd_str, [-12, 12, 15], [0.3, 0.9, 1.0])

        _add(f"Birds: {self.bird_count}   Collisions: {self.collision_total}",
             [-12, 12, 13.5], [1.0, 0.9, 0.3])

        for i, (did, ds) in enumerate(self.drone_states.items()):
            p_ = ds['pos']
            _add(f"{did}  ({p_[0]:.1f},{p_[1]:.1f},z={p_[2]:.1f})",
                 [-12, 12, 12.0 - i * 1.4], [0.8, 0.8, 0.8], size=0.8)

    def get_summary(self) -> dict:
        """Return a dict of all accumulated metrics for logging or display."""
        return {
            'scenario':        self.scenario,
            'steps':           self.step_count,
            'fps_avg':         round(self.fps_avg, 2),
            'fps_min':         round(self.fps_min, 2),
            'fps_max':         round(self.fps_max, 2),
            'crowd_total':     self.crowd_total,
            'crowd_states':    self.crowd_states,
            'bird_count':      self.bird_count,
            'collision_total': self.collision_total,
        }

    def reset(self):
        """Reset all counters for a new episode (keeps scenario name)."""
        self._frame_times.clear()
        self.fps_current = 0.0
        self.fps_avg     = 0.0
        self.fps_min     = float('inf')
        self.fps_max     = 0.0
        self.step_count  = 0
        self.collision_total  = 0
        self.collision_events = []
        self.drone_states     = {}
        self._overlay_ids     = []
