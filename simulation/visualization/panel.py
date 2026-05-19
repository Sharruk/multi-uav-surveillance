"""
simulation/visualization/panel.py
===================================
HUD / metrics panel rendering.

draw_panel() renders the right-side panel with:
  - Mission overview (time, status, crowd stats)
  - Research metrics (coverage, accuracy, wind, comms)
  - UAV Coordination (leader-follower status)
  - Crowd hotspot breakdown
  - Per-UAV status cards (name, battery bar, speed, distance)

Teammate note: to add a new metric, add a call to row() inside the
appropriate sec() block and make sure the key exists in the metrics dict.
"""

import math
import pygame

from simulation.config import (
    SIM_W, SIM_H, TITLE_H, PANEL_W,
    GPS_NOISE, COMM_DELAY, NUM_PEOPLE, COMM_RADIUS,
    C_PANEL_BG, C_PANEL_LINE,
    C_TEXT_PRI, C_TEXT_SEC, C_TEXT_OK, C_TEXT_WARN, C_TEXT_ACT, C_TEXT_ERR,
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
               lf_system=None) -> None:
    """Render the full right-side HUD panel."""
    px = SIM_W;  py = TITLE_H
    pygame.draw.rect(surf, C_PANEL_BG, (px, py, PANEL_W, SIM_H))
    pygame.draw.line(surf, C_PANEL_LINE, (px, py), (px, py + SIM_H), 1)
    y = py + 10;  mg = 14

    def row(lbl, val, lc=C_TEXT_SEC, vc=C_TEXT_PRI):
        nonlocal y
        ls = fsm.render(lbl, True, lc)
        vs = fsm.render(str(val), True, vc)
        surf.blit(ls, (px + mg, y))
        surf.blit(vs, (px + PANEL_W - vs.get_width() - mg, y))
        y += ls.get_height() + 4

    def sec(title):
        nonlocal y
        y += 5
        pygame.draw.line(surf, C_PANEL_LINE, (px + mg, y), (px + PANEL_W - mg, y), 1)
        y += 5
        h = fxs.render(title.upper(), True, C_TEXT_SEC)
        surf.blit(h, (px + mg, y));  y += h.get_height() + 5

    # Header
    hdr = fh.render("Mission Statistics", True, C_TEXT_PRI)
    surf.blit(hdr, (px + PANEL_W // 2 - hdr.get_width() // 2, y))
    y += hdr.get_height() + 3

    # Overview
    sec("Overview")
    mm = int(elapsed // 60);  ss = int(elapsed % 60)
    row("Elapsed Time",  f"{mm:02d}:{ss:02d}")
    row("Status", "PAUSED" if paused else "RUNNING",
        vc=C_TEXT_WARN if paused else C_TEXT_OK)
    row("Crowd Density", f"{crowd.density * 100:.0f}%")
    row("Monitored",     f"{crowd.monitored:.0f}%", vc=C_TEXT_ACT)
    ccx, ccy = crowd.center
    row("Crowd Center",  f"({ccx:.0f}, {ccy - TITLE_H:.0f})")

    # Research Metrics
    sec("Research Metrics")
    row("Avoidance Events", metrics["collisions"])
    row("Coverage Area",    f"{metrics['coverage']:.1f}%", vc=C_TEXT_ACT)
    acc = metrics["accuracy"]
    row("Tracking Accuracy", f"{acc:.1f}%",
        vc=C_TEXT_OK if acc > 70 else C_TEXT_WARN)
    row("Total Distance",   f"{metrics['total_dist']:.0f}m")
    row("Wind Speed",       f"{metrics['wind_spd']:.3f} m/s")
    row("GPS Noise",        f"+/-{GPS_NOISE:.1f}px")
    row("Comm Delay",       f"{COMM_DELAY} frames")

    # UAV Coordination (leader-follower)
    if lf_system is not None:
        sec("UAV Coordination")
        leader = drones[lf_system.leader_idx]
        gold = (255, 215, 0)
        row("Leader",      leader.name, vc=gold)
        row("Comm Radius", f"{COMM_RADIUS}px")
        if lf_system.leader_switched:
            row("! Leader Switch", lf_system.last_leader_name,
                lc=C_TEXT_WARN, vc=C_TEXT_WARN)
        for d in drones:
            if d.is_leader:
                tag, tc = "LEAD", gold
            elif d.comm_active:
                tag, tc = "IN RANGE", C_TEXT_OK
            else:
                tag, tc = "OUT RANGE", C_TEXT_ERR
            row(f"  {d.name}", tag, vc=tc)

    # Crowd Hotspots
    sec("Crowd Hotspots")
    for i, cnt in enumerate(crowd.hotspot_counts):
        row(f"  Hotspot {i + 1}", f"{cnt} people",
            vc=C_TEXT_WARN if cnt > NUM_PEOPLE // 4 else C_TEXT_PRI)

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

        spd_px = math.hypot(d.vx, d.vy)
        sv = fxs.render(f"Spd:{spd_px:.2f} Dist:{d.distance * 0.1:.0f}m",
                        True, C_TEXT_SEC)
        surf.blit(sv, (px + mg, y));  y += sv.get_height() + 2
        av = fxs.render(f"Avoid:{d.collision_avoids}", True, C_TEXT_SEC)
        surf.blit(av, (px + mg, y));  y += av.get_height() + 5

    # Footer
    y = py + SIM_H - 28
    pygame.draw.line(surf, C_PANEL_LINE, (px + mg, y), (px + PANEL_W - mg, y), 1)
    y += 7
    hint = fxs.render("SPACE: Reset    ESC: Quit    P: Pause", True, C_TEXT_SEC)
    surf.blit(hint, (px + PANEL_W // 2 - hint.get_width() // 2, y))
