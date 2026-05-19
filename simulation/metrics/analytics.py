"""
simulation/metrics/analytics.py
=================================
Research metrics: grid-based coverage, tracking accuracy, fleet aggregates.

calc_metrics() is called every 30 frames to avoid per-frame grid sampling cost.

Teammate note: metrics/ owner can add new KPIs here without touching drone
or crowd code. Expose them by adding keys to the returned dict and then
display them in visualization/panel.py.
"""

import math

from simulation.config import COVERAGE_R, PAD, SIM_W, SIM_H, TITLE_H
from simulation.environment.city_map import in_building


def calc_metrics(drones: list, crowd, collisions: int = 0) -> dict:
    """Compute research metrics for the current simulation frame.

    Args:
        drones:     list of Drone instances
        crowd:      CrowdSystem instance
        collisions: total number of collisions

    Returns:
        dict with keys:
            coverage     – % of open map area covered by at least one drone
            accuracy     – tracking accuracy score [0, 100]
            total_dist   – total distance flown by all drones (scaled metres)
            wind_spd     – average wind speed across fleet
            path_eff     – path efficiency % (ideal vs actual)
            energy       – energy estimate
            delay        – total frames spent avoiding
            success      – mission success %
    """
    # ── Spatial coverage ─────────────────────────────────────────────────────
    step = 40
    covered = total = 0
    for gx in range(PAD, PAD + SIM_W, step):
        for gy in range(TITLE_H, TITLE_H + SIM_H, step):
            if in_building(gx, gy):
                continue
            total += 1
            if any(math.hypot(d.x - gx, d.y - gy) < COVERAGE_R for d in drones):
                covered += 1
    coverage = (covered / total * 100) if total else 0.0

    # ── Tracking accuracy ─────────────────────────────────────────────────────
    # Based on closest drone to crowd centroid
    cx, cy = crowd.center
    min_d   = min(math.hypot(d.x - cx, d.y - cy) for d in drones)
    accuracy = max(0.0, 100.0 - min_d * 0.4)

    # ── Fleet aggregates ──────────────────────────────────────────────────────
    total_dist = sum(d.distance * 0.1 for d in drones)
    avg_wind   = sum(d.wind_speed for d in drones) / len(drones)
    
    total_raw_dist = sum(d.distance for d in drones)
    total_ideal    = sum(d.ideal_dist for d in drones)
    path_eff = (total_ideal / total_raw_dist * 100) if total_raw_dist > 0 else 100.0
    path_eff = max(0.0, min(100.0, path_eff))

    energy   = sum(d.energy for d in drones)
    delay    = sum(d.delay_caused for d in drones)

    # Mission success is a composite of coverage, accuracy, and collision penalties
    success = (coverage * 0.4) + (accuracy * 0.6)
    success = max(0.0, success - (collisions * 5.0))

    return {
        "coverage":   coverage,
        "accuracy":   accuracy,
        "total_dist": total_dist,
        "wind_spd":   avg_wind,
        "path_eff":   path_eff,
        "energy":     energy,
        "delay":      delay,
        "success":    success,
    }
