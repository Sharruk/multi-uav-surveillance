"""
DataLogger — per-step CSV logging for STIRS-2025 experiments.

Creates four CSV files under output_dir/experiment_name/:

  metrics.csv      — one row per step: fps, drone count, crowd, birds, wind, collisions
  trajectories.csv — one row per drone per step: position, velocity, action
  collisions.csv   — one row per collision event
  summary.csv      — one row written on finalize(): episode totals

Usage:
    logger = DataLogger("run_01", output_dir="logs/")
    # inside step loop:
    logger.log_step(env, obs, actions, fps)
    # on collision:
    logger.log_collision("drone_0", "building", (3.2, -1.0, 5.5))
    # after episode:
    logger.finalize()
"""

import os
import csv
import time


class DataLogger:

    def __init__(self, experiment_name: str, output_dir: str = "logs/"):
        self.experiment  = experiment_name
        self.output_dir  = os.path.join(output_dir, experiment_name)
        os.makedirs(self.output_dir, exist_ok=True)

        self._step = 0
        self._start_wall = time.time()
        self._collision_count = 0

        # Open CSV writers
        self._metrics_f, self._metrics_w = self._open(
            "metrics.csv",
            ["step", "wall_time", "fps", "drone_count",
             "crowd_count", "bird_count", "collisions_total"])

        self._traj_f, self._traj_w = self._open(
            "trajectories.csv",
            ["step", "drone_id",
             "pos_x", "pos_y", "pos_z",
             "act_thrust", "act_roll", "act_pitch", "act_yaw"])

        self._col_f, self._col_w = self._open(
            "collisions.csv",
            ["step", "wall_time", "obj1", "obj2",
             "pos_x", "pos_y", "pos_z"])

    # ── Public API ─────────────────────────────────────────────────────────────

    def log_step(self, env, obs: dict, actions: dict, fps: float = 0.0):
        """
        Log one simulation step.

        env     — DroneSurveillanceEnv instance (after step())
        obs     — observation dict returned by step()
        actions — action dict passed into step()
        fps     — current step FPS (pass 0 if not measured)
        """
        wall = round(time.time() - self._start_wall, 3)
        crowd_count = len(env.crowd_sim.agents) if env.crowd_sim else 0
        bird_count  = len(env.boid_birds.body_ids) if env.boid_birds else 0

        self._metrics_w.writerow([
            self._step, wall, round(fps, 2),
            len(env.agents), crowd_count, bird_count,
            self._collision_count,
        ])

        for drone_id in env.agents:
            pos = obs[drone_id]["position"] if drone_id in obs else [0, 0, 0]
            act = actions.get(drone_id, [0, 0, 0, 0])
            self._traj_w.writerow([
                self._step, drone_id,
                round(float(pos[0]), 4), round(float(pos[1]), 4), round(float(pos[2]), 4),
                round(float(act[0]), 4), round(float(act[1]), 4),
                round(float(act[2]), 4), round(float(act[3]), 4),
            ])

        self._step += 1

    def log_collision(self, obj1: str, obj2: str, position: tuple):
        """Record a collision event with world position."""
        self._collision_count += 1
        wall = round(time.time() - self._start_wall, 3)
        self._col_w.writerow([
            self._step, wall, obj1, obj2,
            round(float(position[0]), 4),
            round(float(position[1]), 4),
            round(float(position[2]), 4),
        ])

    def finalize(self):
        """Flush and close all CSV files, write summary row."""
        duration = round(time.time() - self._start_wall, 2)
        summary_path = os.path.join(self.output_dir, "summary.csv")
        with open(summary_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["experiment", "steps", "duration_s", "collisions_total"])
            w.writerow([self.experiment, self._step, duration, self._collision_count])

        self._metrics_f.close()
        self._traj_f.close()
        self._col_f.close()

    def get_output_dir(self) -> str:
        return self.output_dir

    # ── Internal ───────────────────────────────────────────────────────────────

    def _open(self, filename: str, fieldnames: list):
        path = os.path.join(self.output_dir, filename)
        f    = open(path, 'w', newline='')
        w    = csv.writer(f)
        w.writerow(fieldnames)
        return f, w
