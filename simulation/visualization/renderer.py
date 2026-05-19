"""
simulation/visualization/renderer.py
======================================
Map / city rendering functions.

All draw_* functions accept a pygame.Surface as first argument and draw
directly to it. They are purely visual — no simulation state is mutated.

Teammate note: UI/visualization owner works exclusively in this file and
panel.py. No changes to drone/ or crowd/ are needed for visual tweaks.
"""

import math
import pygame

from simulation.config import (
    SIM_W, SIM_H, TITLE_H, WIN_W,
    PAD, C_ROAD, C_ROAD_MARK,
    C_BLDG_FILL, C_BLDG_BORDER, C_BLDG_LABEL,
    C_TITLE_BG, C_PANEL_LINE, C_TEXT_PRI, C_TEXT_SEC,
    COMM_RADIUS,
)
from simulation.environment.city_map import V_ROADS, H_ROADS, BUILDINGS, ZONES


# ── Utility ────────────────────────────────────────────────────────────────────

def draw_dash(surf, color: tuple, start: tuple, end: tuple,
              seg: int = 6, gap: int = 4) -> None:
    """Draw a dashed line from start to end."""
    dx = end[0] - start[0];  dy = end[1] - start[1]
    total = math.hypot(dx, dy)
    if total < 1:
        return
    ux, uy = dx / total, dy / total
    dim = tuple(max(0, c - 110) for c in color)
    pos = 0;  on = True
    while pos < total:
        ln = seg if on else gap
        if on:
            x1 = int(start[0] + ux * pos);         y1 = int(start[1] + uy * pos)
            x2 = int(start[0] + ux * min(pos + ln, total))
            y2 = int(start[1] + uy * min(pos + ln, total))
            pygame.draw.line(surf, dim, (x1, y1), (x2, y2), 1)
        pos += ln;  on = not on


# ── Map elements ───────────────────────────────────────────────────────────────

def draw_roads(surf) -> None:
    """Render road rectangles and centre-line dashes."""
    for x0, x1 in V_ROADS:
        pygame.draw.rect(surf, C_ROAD, (x0, TITLE_H, x1 - x0, SIM_H))
    for y0, y1 in H_ROADS:
        pygame.draw.rect(surf, C_ROAD, (PAD, y0, SIM_W, y1 - y0))
    dash, gap = 14, 10
    for x0, x1 in V_ROADS:
        cx = (x0 + x1) // 2;  y = TITLE_H
        while y < TITLE_H + SIM_H:
            pygame.draw.line(surf, C_ROAD_MARK, (cx, y),
                             (cx, min(y + dash, TITLE_H + SIM_H)), 1)
            y += dash + gap
    for y0, y1 in H_ROADS:
        cy = (y0 + y1) // 2;  x = PAD
        while x < PAD + SIM_W:
            pygame.draw.line(surf, C_ROAD_MARK, (x, cy),
                             (min(x + dash, PAD + SIM_W), cy), 1)
            x += dash + gap


def draw_buildings(surf, fsm, fxs) -> None:
    """Render building blocks with badge and label."""
    for rect, short, full in BUILDINGS:
        pygame.draw.rect(surf, C_BLDG_FILL,   rect, border_radius=3)
        pygame.draw.rect(surf, C_BLDG_BORDER,  rect, 1, border_radius=3)
        badge = fxs.render(short, True, C_BLDG_BORDER)
        surf.blit(badge, (rect.x + 5, rect.y + 5))
        lines = full.split("\n")
        th    = len(lines) * (fsm.get_height() + 2)
        sy    = rect.centery - th // 2
        for i, ln in enumerate(lines):
            t = fsm.render(ln, True, C_BLDG_LABEL)
            surf.blit(t, (rect.centerx - t.get_width() // 2,
                          sy + i * (fsm.get_height() + 2)))


def draw_zones(surf, fsm) -> None:
    """Render translucent zone overlays with border and label."""
    for z in ZONES:
        r  = z["rect"];  fc = z["fill"];  bc = z["border"]
        zs = pygame.Surface((r.w, r.h), pygame.SRCALPHA)
        zs.fill((*fc, 22))
        surf.blit(zs, (r.x, r.y))
        pygame.draw.rect(surf, bc, r, 1, border_radius=2)
        lbl = fsm.render(z["label"], True, bc)
        surf.blit(lbl, (r.centerx - lbl.get_width() // 2,
                        r.bottom - fsm.get_height() - 5))


def draw_title(surf, ft, fxs) -> None:
    """Render the top title bar."""
    pygame.draw.rect(surf, C_TITLE_BG, (0, 0, WIN_W, TITLE_H))
    pygame.draw.line(surf, C_PANEL_LINE, (0, TITLE_H), (WIN_W, TITLE_H), 1)
    t = ft.render("Smart City Multi-UAV Crowd Surveillance", True, C_TEXT_PRI)
    surf.blit(t, (12, TITLE_H // 2 - t.get_height() // 2))
    s = fxs.render("Research Prototype  |  Python + Pygame", True, C_TEXT_SEC)
    surf.blit(s, (WIN_W - s.get_width() - 10, TITLE_H // 2 - s.get_height() // 2))


# ── Dynamic obstacles ───────────────────────────────────────────────────────────

def draw_obstacles(surf, obstacles: list) -> None:
    """Draw all dynamic obstacles (vehicles and construction zones)."""
    for obs in obstacles:
        obs.draw(surf)


# ── Leader-follower visualisation ───────────────────────────────────────────────

def draw_comm_links(surf, drones: list, lf_system) -> None:
    """Draw communication network: links, leader indicator, comm radius.

    Renders:
      - Semi-transparent comm-radius ring around the leader
      - Thin coloured lines between UAV pairs within COMM_RADIUS
      - Gold halo + crown icon on the current leader drone
      - Flash ring when a leader switch occurs
    """
    if not drones:
        return
    leader = drones[lf_system.leader_idx]
    lx, ly = int(leader.x), int(leader.y)

    # 1. Comm-radius ring around leader
    cr = COMM_RADIUS
    rs = pygame.Surface((cr * 2, cr * 2), pygame.SRCALPHA)
    pygame.draw.circle(rs, (100, 200, 255, 18), (cr, cr), cr)
    surf.blit(rs, (lx - cr, ly - cr))
    pygame.draw.circle(surf, (100, 200, 255, 80), (lx, ly), cr, 1)

    # 2. Comm links between all pairs within COMM_RADIUS
    for a, b in lf_system.comm_links(drones):
        ax, ay = int(a.x), int(a.y)
        bx, by = int(b.x), int(b.y)
        link_surf = pygame.Surface(surf.get_size(), pygame.SRCALPHA)
        pygame.draw.line(link_surf, (100, 200, 255, 60), (ax, ay), (bx, by), 1)
        surf.blit(link_surf, (0, 0))

    # 3. Leader gold halo
    if lf_system.leader_switched:
        # Flash: alternate colour
        flash_col = (255, 255, 100, 100) if (lf_system.switch_flash // 8) % 2 == 0 \
                    else (255, 180, 0, 60)
        fs = pygame.Surface((40, 40), pygame.SRCALPHA)
        pygame.draw.circle(fs, flash_col, (20, 20), 20)
        surf.blit(fs, (lx - 20, ly - 20))

    pygame.draw.circle(surf, (255, 215, 0), (lx, ly), 16, 2)   # gold ring

    # 4. "L" crown badge above leader
    badge_x, badge_y = lx - 4, ly - 30
    pygame.draw.polygon(surf, (255, 215, 0), [
        (badge_x,      badge_y + 8),
        (badge_x + 8,  badge_y + 8),
        (badge_x + 8,  badge_y),
        (badge_x + 6,  badge_y + 4),
        (badge_x + 4,  badge_y),
        (badge_x + 2,  badge_y + 4),
        (badge_x,      badge_y),
    ])
