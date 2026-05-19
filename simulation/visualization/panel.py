"""
simulation/visualization/panel.py
===================================
HUD / metrics panel rendering.

draw_panel() renders the right-side panel with:
  - Active Algorithms (collision avoidance & path planning)
  - Mission overview (time, status, crowd stats)
  - Research metrics (coverage, accuracy, path efficiency, etc.)
  - UAV Coordination (leader-follower status)
  - Event Log
  - Crowd hotspot breakdown
  - Per-UAV status cards

Teammate note: to add a new metric, add a call to row() inside the
appropriate sec() block and make sure the key exists in the metrics dict.
"""

import math
import pygame

from simulation.config import (
    SIM_W, SIM_H, TITLE_H, PANEL_W, WIN_H,
    GPS_NOISE, COMM_DELAY, NUM_PEOPLE, COMM_RADIUS,
    C_PANEL_BG, C_PANEL_LINE,
    C_TEXT_PRI, C_TEXT_SEC, C_TEXT_OK, C_TEXT_WARN, C_TEXT_ACT, C_TEXT_ERR, C_TEXT_LOG
)


def _batt_color(b: float) -> tuple:
    """Return colour based on battery level."""
    if b > 50:
        return C_TEXT_OK
    if b > 20:
        return C_TEXT_WARN
    return C_TEXT_ERR


def draw_panel(surf, fh, fsm, fxs,
               drones: list, crowd,
               metrics: dict, elapsed: float, paused: bool,
               lf_system=None, 
               active_avoidance="Rule-based", active_planner="Coverage Offset",
               event_log=None) -> None:
    """Render the full right-side HUD panel."""
    px = SIM_W;  py = TITLE_H
    # Dynamic panel height support if needed, but we keep it inside SIM_H limits
    # except that the new items might overflow SIM_H. Wait, WIN_H - TITLE_H is SIM_H.
    pygame.draw.rect(surf, C_PANEL_BG, (px, py, PANEL_W, SIM_H))
    pygame.draw.line(surf, C_PANEL_LINE, (px, py), (px, py + SIM_H), 1)
    y = py + 10;  mg = 14

    def row(lbl, val, lc=C_TEXT_SEC, vc=C_TEXT_PRI):
        nonlocal y
        ls = fsm.render(lbl, True, lc)
        vs = fsm.render(str(val), True, vc)
        surf.blit(ls, (px + mg, y))
        surf.blit(vs, (px + PANEL_W - vs.get_width() - mg, y))
        y += ls.get_height() + 2  # reduced spacing to fit everything

    def sec(title):
        nonlocal y
        y += 4
        pygame.draw.line(surf, C_PANEL_LINE, (px + mg, y), (px + PANEL_W - mg, y), 1)
        y += 4
        h = fxs.render(title.upper(), True, C_TEXT_SEC)
        surf.blit(h, (px + mg, y));  y += h.get_height() + 3

    # Header
    hdr = fh.render("Research Dashboard", True, C_TEXT_PRI)
    surf.blit(hdr, (px + PANEL_W // 2 - hdr.get_width() // 2, y))
    y += hdr.get_height() + 3

    # Active Algorithms
    sec("Active Algorithms")
    row("Avoidance", active_avoidance, vc=C_TEXT_ACT)
    row("Planner", active_planner, vc=C_TEXT_ACT)

    # Overview
    sec("Mission Overview")
    mm = int(elapsed // 60);  ss = int(elapsed % 60)
    row("Elapsed Time",  f"{mm:02d}:{ss:02d}")
    row("Status", "PAUSED" if paused else "RUNNING",
        vc=C_TEXT_WARN if paused else C_TEXT_OK)
    row("Success Score", f"{metrics.get('success', 0):.1f}%", vc=C_TEXT_OK)

    # Research Metrics
    sec("Research Metrics")
    row("Avoidance Events", metrics.get("collisions", 0))
    row("Coverage Area",    f"{metrics.get('coverage', 0):.1f}%", vc=C_TEXT_ACT)
    acc = metrics.get("accuracy", 0)
    row("Tracking Accuracy", f"{acc:.1f}%",
        vc=C_TEXT_OK if acc > 70 else C_TEXT_WARN)
    row("Total Distance",   f"{metrics.get('total_dist', 0):.0f}m")
    row("Path Efficiency",  f"{metrics.get('path_eff', 100):.1f}%")
    row("Energy Estimate",  f"{metrics.get('energy', 0):.1f} kJ")
    row("Delay Caused",     f"{metrics.get('delay', 0)} frames")

    # Environmental
    sec("Environmental Factors")
    row("Wind Speed",       f"{metrics.get('wind_spd', 0):.3f} m/s")
    row("GPS Noise",        f"+/-{GPS_NOISE:.1f}px")
    row("Comm Delay",       f"{COMM_DELAY} frames")

    # UAV Coordination (leader-follower)
    if lf_system is not None:
        sec("UAV Coordination")
        leader = drones[lf_system.leader_idx]
        gold = (255, 215, 0)
        row("Leader",      leader.name, vc=gold)
        row("Comm Radius", f"{COMM_RADIUS}px")

    # Event Log
    if event_log is not None:
        sec("Event Log")
        for ev in event_log[-4:]:  # last 4 events
            t = fxs.render(f"> {ev}", True, C_TEXT_LOG)
            surf.blit(t, (px + mg, y)); y += t.get_height() + 2

    # UAV Fleet Status
    sec("UAV Fleet Status")
    bar_w = PANEL_W - mg * 2 - 55
    for d in drones:
        ns = fsm.render(d.name, True, d.color)
        surf.blit(ns, (px + mg, y))
        stc = (C_TEXT_OK   if d.status == "On-Station" else
               C_TEXT_WARN if d.status == "Avoiding"   else
               C_TEXT_ERR  if d.status == "RTB"        else C_TEXT_ACT)
        ss2 = fxs.render(d.status, True, stc)
        surf.blit(ss2, (px + PANEL_W - ss2.get_width() - mg, y))
        y += ns.get_height() + 2

        bc2 = _batt_color(d.battery)
        pygame.draw.rect(surf, C_PANEL_LINE, (px + mg, y, bar_w, 7), border_radius=2)
        fill = max(2, int(bar_w * d.battery / 100))
        pygame.draw.rect(surf, bc2, (px + mg, y, fill, 7), border_radius=2)
        bv = fxs.render(f"{d.battery:.0f}%", True, bc2)
        surf.blit(bv, (px + mg + bar_w + 4, y - 1));  y += 11

    # Footer
    y = py + SIM_H - 28
    pygame.draw.line(surf, C_PANEL_LINE, (px + mg, y), (px + PANEL_W - mg, y), 1)
    y += 7
    hint = fxs.render("1-4: Avoidance  5-9: Planner  SPACE: Reset  ESC: Quit", True, C_TEXT_SEC)
    surf.blit(hint, (px + PANEL_W // 2 - hint.get_width() // 2, y))
