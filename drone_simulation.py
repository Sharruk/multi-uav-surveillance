"""
=======================================================================
  Smart City Multi-UAV Surveillance Simulation
  Research Prototype — Python + Pygame
=======================================================================

INSTALLATION:
    pip install pygame

RUN:
    python drone_simulation.py
    (or press the Run button in VS Code)

CONTROLS:
    SPACE  — Reset simulation
    ESC    — Quit
=======================================================================
"""

import pygame
import math
import time

# -----------------------------------------------------------------------
# WINDOW LAYOUT
# -----------------------------------------------------------------------
SIM_W   = 660          # Simulation viewport width
SIM_H   = 660          # Simulation viewport height
PANEL_W = 340          # Right-side statistics panel width
TITLE_H = 42           # Top title bar height
WIN_W   = SIM_W + PANEL_W   # 1000
WIN_H   = SIM_H + TITLE_H   # 702
FPS     = 60

# -----------------------------------------------------------------------
# COLOUR PALETTE  (professional / academic)
# -----------------------------------------------------------------------
C_BG          = (14,  17,  28)    # Simulation background
C_ROAD        = (30,  37,  56)    # Road surface
C_ROAD_MARK   = (55,  65,  95)    # Dashed centre-line colour
C_BLDG_FILL   = (38,  47,  72)    # Building fill
C_BLDG_BORDER = (70,  88, 130)    # Building outline
C_BLDG_LABEL  = (130, 150, 195)   # Building text
C_TITLE_BG    = (10,  13,  22)    # Title bar background
C_PANEL_BG    = (10,  13,  22)    # Panel background
C_PANEL_LINE  = (32,  42,  68)    # Panel divider
C_TEXT_PRI    = (218, 228, 255)   # Primary text
C_TEXT_SEC    = (115, 135, 175)   # Secondary / dim text
C_TEXT_OK     = ( 75, 210, 130)   # Arrived (green)
C_TEXT_WARN   = (255, 180,  50)   # Avoiding (amber)
C_TEXT_ACT    = (100, 180, 255)   # Active (blue)

# Drone colours — distinct, not neon
DRONE_COLORS = [
    ( 80, 190, 255),   # A — steel blue
    ( 75, 210, 130),   # B — muted green
    (255, 175,  50),   # C — amber
    (200, 100, 255),   # D — soft violet
]
DRONE_NAMES = ["Drone A", "Drone B", "Drone C", "Drone D"]

# -----------------------------------------------------------------------
# CITY GRID  — 3×3 block layout
#   Block size : 200 × 200 px
#   Road width :  28 px
#   Total      : 3×200 + 2×28 = 656 px  (centred in 660 with 2px pad each side)
# -----------------------------------------------------------------------
PAD   = 2     # 2 px outer padding
BLK   = 200   # block size
RD    = 28    # road width

# Helper: top-left corner of block (col, row)
def blk(c, r):
    return (PAD + c * (BLK + RD), TITLE_H + PAD + r * (BLK + RD))

# Vertical road x-bands  [x_start, x_end]
V_ROADS = [(PAD + BLK, PAD + BLK + RD), (PAD + 2*BLK + RD, PAD + 2*BLK + 2*RD)]
# Horizontal road y-bands
H_ROADS = [(TITLE_H + PAD + BLK, TITLE_H + PAD + BLK + RD),
           (TITLE_H + PAD + 2*BLK + RD, TITLE_H + PAD + 2*BLK + 2*RD)]

# -----------------------------------------------------------------------
# BUILDINGS  — (rect, short_name, long_name)
# Blocks used as buildings: TL, TC, TR, MM (mall), BR
# -----------------------------------------------------------------------
BUILDINGS = [
    (pygame.Rect(blk(0,0)[0]+3, blk(0,0)[1]+3, BLK-6, BLK-6), "COM", "Commercial\nDistrict"),
    (pygame.Rect(blk(1,0)[0]+3, blk(1,0)[1]+3, BLK-6, BLK-6), "CTH", "City Hall"),
    (pygame.Rect(blk(2,0)[0]+3, blk(2,0)[1]+3, BLK-6, BLK-6), "TEC", "Tech Hub"),
    (pygame.Rect(blk(1,1)[0]+3, blk(1,1)[1]+3, BLK-6, BLK-6), "MAL", "Central\nMall"),
    (pygame.Rect(blk(2,2)[0]+3, blk(2,2)[1]+3, BLK-6, BLK-6), "UNI", "University"),
]

# -----------------------------------------------------------------------
# MONITORING ZONES  — (rect, label, fill_color, border_color)
# Open blocks: ML, MR, BL, BC
# -----------------------------------------------------------------------
ZONE_COLORS = [
    (( 60, 185,  90), ( 80, 220, 110)),   # A — green
    ((220, 110,  40), (255, 140,  55)),   # C — orange
    (( 80, 155, 255), (110, 185, 255)),   # D — blue
    ((200,  80,  80), (230, 110, 100)),   # B — red
]
ZONES = []
zone_defs = [(0,1,"Crowd Zone A"), (2,1,"Crowd Zone B"), (0,2,"Crowd Zone C"), (1,2,"Crowd Zone D")]
for idx, (c, r, lbl) in enumerate(zone_defs):
    bx, by = blk(c, r)
    ZONES.append({
        "rect"  : pygame.Rect(bx+3, by+3, BLK-6, BLK-6),
        "label" : lbl,
        "fill"  : ZONE_COLORS[idx][0],
        "border": ZONE_COLORS[idx][1],
    })

# -----------------------------------------------------------------------
# DRONE START & TARGET POSITIONS  (centres of zone blocks)
# Drones cross paths to demonstrate avoidance
# -----------------------------------------------------------------------
def zone_center(c, r):
    bx, by = blk(c, r)
    return (bx + BLK // 2, by + BLK // 2)

START_POS = [
    zone_center(0, 1),   # A starts: Zone A  → targets Zone D
    zone_center(2, 1),   # B starts: Zone B  → targets Zone C
    zone_center(0, 2),   # C starts: Zone C  → targets Zone B
    zone_center(1, 2),   # D starts: Zone D  → targets Zone A
]
TARGET_POS = [
    zone_center(1, 2),   # A → Zone D
    zone_center(0, 2),   # B → Zone C
    zone_center(2, 1),   # C → Zone B
    zone_center(0, 1),   # D → Zone A
]

# -----------------------------------------------------------------------
# PHYSICS SETTINGS
# -----------------------------------------------------------------------
DRONE_RADIUS    = 11
DRONE_SPEED     = 1.8
ARRIVE_DIST     = 16
AVOID_DRONE_R   = 65    # drone–drone avoidance radius
AVOID_DRONE_S   = 2.0   # avoidance push strength
AVOID_OBS_R     = 58    # obstacle avoidance radius
AVOID_OBS_S     = 2.8   # obstacle push strength


# =======================================================================
#  DRONE CLASS
# =======================================================================
class Drone:
    def __init__(self, idx):
        self.idx    = idx
        self.name   = DRONE_NAMES[idx]
        self.color  = DRONE_COLORS[idx]

        sx, sy = START_POS[idx]
        self.x  = float(sx)
        self.y  = float(sy)
        tx, ty  = TARGET_POS[idx]
        self.tx = float(tx)
        self.ty = float(ty)

        self.vx = 0.0
        self.vy = 0.0

        # Metrics
        self.status    = "Active"    # "Active" | "Avoiding" | "Arrived"
        self.distance  = 0.0        # total distance travelled (px)
        self.start_time = time.time()
        self.travel_time = 0.0      # seconds to arrive

    def reset(self):
        sx, sy = START_POS[self.idx]
        self.x, self.y = float(sx), float(sy)
        tx, ty = TARGET_POS[self.idx]
        self.tx, self.ty = float(tx), float(ty)
        self.vx = self.vy = 0.0
        self.status = "Active"
        self.distance = 0.0
        self.start_time = time.time()
        self.travel_time = 0.0

    def dist_to(self, ox, oy):
        return math.hypot(self.x - ox, self.y - oy)

    def update(self, all_drones, buildings, collisions_ref):
        if self.status == "Arrived":
            return

        # -- Step 1: velocity toward target --
        dx = self.tx - self.x
        dy = self.ty - self.y
        d  = math.hypot(dx, dy)

        if d < ARRIVE_DIST:
            self.status      = "Arrived"
            self.travel_time = time.time() - self.start_time
            return

        self.vx = (dx / d) * DRONE_SPEED
        self.vy = (dy / d) * DRONE_SPEED
        self.status = "Active"

        # -- Step 2: drone–drone avoidance --
        avoiding = False
        for other in all_drones:
            if other is self:
                continue
            dist = self.dist_to(other.x, other.y)
            if 0 < dist < AVOID_DRONE_R:
                avoiding = True
                # Count avoidance event (only once per pair, guard with order)
                if self.idx < other.idx:
                    collisions_ref[0] += 1
                strength = (AVOID_DRONE_R - dist) / AVOID_DRONE_R * AVOID_DRONE_S
                self.vx -= strength * (other.x - self.x) / dist
                self.vy -= strength * (other.y - self.y) / dist

        # -- Step 3: obstacle (building) avoidance --
        for bldg_rect, _, _ in buildings:
            # Closest point on the rectangle to the drone
            cx = max(bldg_rect.left, min(self.x, bldg_rect.right))
            cy = max(bldg_rect.top,  min(self.y, bldg_rect.bottom))
            dist = math.hypot(self.x - cx, self.y - cy)
            if 0 < dist < AVOID_OBS_R:
                avoiding = True
                strength = (AVOID_OBS_R - dist) / AVOID_OBS_R * AVOID_OBS_S
                self.vx += strength * (self.x - cx) / dist
                self.vy += strength * (self.y - cy) / dist

        if avoiding:
            self.status = "Avoiding"

        # -- Step 4: apply movement --
        prev_x, prev_y = self.x, self.y
        self.x += self.vx
        self.y += self.vy

        # Clamp to simulation area
        self.x = max(PAD + DRONE_RADIUS, min(PAD + SIM_W - DRONE_RADIUS, self.x))
        self.y = max(TITLE_H + PAD + DRONE_RADIUS, min(TITLE_H + PAD + SIM_H - DRONE_RADIUS, self.y))

        # Accumulate distance travelled
        self.distance += math.hypot(self.x - prev_x, self.y - prev_y)

    def draw(self, surface, font_sm):
        ix, iy = int(self.x), int(self.y)

        # Dotted line from drone to target (mission path indicator)
        tx, ty = int(self.tx), int(self.ty)
        draw_dotted_line(surface, self.color, (ix, iy), (tx, ty), 6, 3, 60)

        # Drone body
        pygame.draw.circle(surface, self.color, (ix, iy), DRONE_RADIUS)
        pygame.draw.circle(surface, (200, 215, 255), (ix, iy), DRONE_RADIUS, 2)

        # Directional tick (shows heading)
        if self.status != "Arrived":
            speed = math.hypot(self.vx, self.vy)
            if speed > 0.01:
                ex = ix + int((self.vx / speed) * (DRONE_RADIUS + 5))
                ey = iy + int((self.vy / speed) * (DRONE_RADIUS + 5))
                pygame.draw.line(surface, (255, 255, 255), (ix, iy), (ex, ey), 2)

        # Label above drone
        lbl = font_sm.render(self.name, True, self.color)
        surface.blit(lbl, (ix - lbl.get_width() // 2, iy - DRONE_RADIUS - 16))


# =======================================================================
#  DRAWING HELPERS
# =======================================================================

def draw_dotted_line(surface, color, start, end, seg_len, gap_len, alpha=120):
    """Draw a dashed line from start to end."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    total = math.hypot(dx, dy)
    if total < 1:
        return
    ux, uy = dx / total, dy / total
    pos = 0
    draw_phase = True
    r, g, b = color
    while pos < total:
        length = seg_len if draw_phase else gap_len
        if draw_phase:
            x1 = int(start[0] + ux * pos)
            y1 = int(start[1] + uy * pos)
            x2 = int(start[0] + ux * min(pos + length, total))
            y2 = int(start[1] + uy * min(pos + length, total))
            # Use a dim version of the colour
            dim = tuple(max(0, c - 120) for c in (r, g, b))
            pygame.draw.line(surface, dim, (x1, y1), (x2, y2), 1)
        pos += length
        draw_phase = not draw_phase


def draw_road_markings(surface):
    """Draw road surface and dashed centre lines."""
    # Fill road bands
    for x0, x1 in V_ROADS:
        pygame.draw.rect(surface, C_ROAD, (x0, TITLE_H, x1 - x0, SIM_H))
    for y0, y1 in H_ROADS:
        pygame.draw.rect(surface, C_ROAD, (PAD, y0, SIM_W, y1 - y0))

    # Dashed centre lines
    dash, gap = 14, 10
    for x0, x1 in V_ROADS:
        cx = (x0 + x1) // 2
        y = TITLE_H
        while y < TITLE_H + SIM_H:
            pygame.draw.line(surface, C_ROAD_MARK, (cx, y), (cx, min(y + dash, TITLE_H + SIM_H)), 1)
            y += dash + gap

    for y0, y1 in H_ROADS:
        cy = (y0 + y1) // 2
        x = PAD
        while x < PAD + SIM_W:
            pygame.draw.line(surface, C_ROAD_MARK, (x, cy), (min(x + dash, PAD + SIM_W), cy), 1)
            x += dash + gap


def draw_buildings(surface, font_sm, font_xs):
    """Draw building blocks with labels."""
    for rect, short, full in BUILDINGS:
        pygame.draw.rect(surface, C_BLDG_FILL,   rect, border_radius=3)
        pygame.draw.rect(surface, C_BLDG_BORDER, rect, 1, border_radius=3)

        # Short code badge (top-left corner of building)
        badge = font_xs.render(short, True, C_BLDG_BORDER)
        surface.blit(badge, (rect.x + 5, rect.y + 5))

        # Full name centred
        lines = full.split("\n")
        total_h = len(lines) * (font_sm.get_height() + 2)
        start_y = rect.centery - total_h // 2
        for i, line in enumerate(lines):
            txt = font_sm.render(line, True, C_BLDG_LABEL)
            surface.blit(txt, (rect.centerx - txt.get_width() // 2,
                               start_y + i * (font_sm.get_height() + 2)))


def draw_zones(surface, font_sm):
    """Draw semi-transparent monitoring zones."""
    for z in ZONES:
        r = z["rect"]
        fc = z["fill"]
        bc = z["border"]

        # Semi-transparent fill
        zone_surf = pygame.Surface((r.w, r.h), pygame.SRCALPHA)
        zone_surf.fill((*fc, 28))
        surface.blit(zone_surf, (r.x, r.y))

        # Dashed border
        pygame.draw.rect(surface, bc, r, 1, border_radius=2)

        # Zone label at bottom of zone
        lbl = font_sm.render(z["label"], True, bc)
        surface.blit(lbl, (r.centerx - lbl.get_width() // 2,
                           r.bottom - font_sm.get_height() - 5))

        # Small target crosshair at center
        cx, cy = r.centerx, r.centery
        cl = 10
        pygame.draw.line(surface, (*bc, 160), (cx - cl, cy), (cx + cl, cy), 1)
        pygame.draw.line(surface, (*bc, 160), (cx, cy - cl), (cx, cy + cl), 1)
        pygame.draw.circle(surface, bc, (cx, cy), 4, 1)


def draw_title_bar(surface, font_title, font_xs):
    """Draw the top title bar."""
    pygame.draw.rect(surface, C_TITLE_BG, (0, 0, WIN_W, TITLE_H))
    pygame.draw.line(surface, C_PANEL_LINE, (0, TITLE_H), (WIN_W, TITLE_H), 1)

    title = font_title.render("Smart City Multi-UAV Surveillance System", True, C_TEXT_PRI)
    surface.blit(title, (12, TITLE_H // 2 - title.get_height() // 2))

    sub = font_xs.render("Research Prototype  |  Python + Pygame", True, C_TEXT_SEC)
    surface.blit(sub, (WIN_W - sub.get_width() - 10, TITLE_H // 2 - sub.get_height() // 2))


def draw_panel(surface, font_h, font_md, font_sm, font_xs, drones, metrics, elapsed):
    """Draw the right-side statistics panel."""
    px = SIM_W   # panel starts here (x)
    py = TITLE_H

    # Background
    pygame.draw.rect(surface, C_PANEL_BG, (px, py, PANEL_W, SIM_H))
    pygame.draw.line(surface, C_PANEL_LINE, (px, py), (px, py + SIM_H), 1)

    y = py + 12
    margin = 14

    def row(label, value, label_col=C_TEXT_SEC, val_col=C_TEXT_PRI):
        nonlocal y
        lbl_surf = font_sm.render(label, True, label_col)
        val_surf = font_sm.render(str(value), True, val_col)
        surface.blit(lbl_surf, (px + margin, y))
        surface.blit(val_surf, (px + PANEL_W - val_surf.get_width() - margin, y))
        y += lbl_surf.get_height() + 5

    def section(title):
        nonlocal y
        y += 6
        pygame.draw.line(surface, C_PANEL_LINE, (px + margin, y), (px + PANEL_W - margin, y), 1)
        y += 6
        hdr = font_xs.render(title.upper(), True, C_TEXT_SEC)
        surface.blit(hdr, (px + margin, y))
        y += hdr.get_height() + 6

    # -- Title --
    hdr = font_h.render("Mission Statistics", True, C_TEXT_PRI)
    surface.blit(hdr, (px + PANEL_W // 2 - hdr.get_width() // 2, y))
    y += hdr.get_height() + 4

    # -- Overview --
    section("Overview")
    arrived    = sum(1 for d in drones if d.status == "Arrived")
    active     = sum(1 for d in drones if d.status == "Active")
    avoiding   = sum(1 for d in drones if d.status == "Avoiding")
    completion = int(arrived / len(drones) * 100)

    row("Mission Completion",  f"{completion}%",   val_col=C_TEXT_OK if completion == 100 else C_TEXT_PRI)
    row("Active Drones",       active,             val_col=C_TEXT_ACT)
    row("Arrived Drones",      arrived,            val_col=C_TEXT_OK)
    row("Avoiding Drones",     avoiding,           val_col=C_TEXT_WARN if avoiding else C_TEXT_SEC)
    row("Collisions Avoided",  metrics["collisions"])
    row("Elapsed Time",        f"{elapsed:.1f}s")

    # Average travel time (only for arrived drones)
    arrived_drones = [d for d in drones if d.status == "Arrived" and d.travel_time > 0]
    avg_t = (sum(d.travel_time for d in arrived_drones) / len(arrived_drones)) if arrived_drones else 0
    row("Avg Travel Time",     f"{avg_t:.1f}s" if avg_t else "--")

    # -- Drone Details --
    section("Drone Status")
    for d in drones:
        y += 2
        # Colour dot
        pygame.draw.circle(surface, d.color, (px + margin + 5, y + 6), 5)

        # Name & status
        name_surf = font_sm.render(d.name, True, d.color)
        surface.blit(name_surf, (px + margin + 14, y))

        st_col = C_TEXT_OK if d.status == "Arrived" else (C_TEXT_WARN if d.status == "Avoiding" else C_TEXT_ACT)
        st_surf = font_sm.render(d.status, True, st_col)
        surface.blit(st_surf, (px + PANEL_W - st_surf.get_width() - margin, y))
        y += name_surf.get_height() + 3

    # -- Distance travelled --
    section("Distance Travelled (px)")
    for d in drones:
        row(d.name, f"{d.distance:,.0f} px", label_col=d.color)

    # -- Controls hint --
    y = py + SIM_H - 32
    pygame.draw.line(surface, C_PANEL_LINE, (px + margin, y), (px + PANEL_W - margin, y), 1)
    y += 8
    hint = font_xs.render("SPACE: Reset    ESC: Quit", True, C_TEXT_SEC)
    surface.blit(hint, (px + PANEL_W // 2 - hint.get_width() // 2, y))


# =======================================================================
#  MAIN
# =======================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Smart City UAV Surveillance — Research Prototype")
    clock = pygame.time.Clock()

    # Fonts — using system fonts for clean look
    font_title = pygame.font.SysFont("segoeui",    15, bold=True)
    font_h     = pygame.font.SysFont("segoeui",    15, bold=True)
    font_md    = pygame.font.SysFont("segoeui",    13)
    font_sm    = pygame.font.SysFont("segoeui",    12)
    font_xs    = pygame.font.SysFont("consolas",   10)

    # Create drones
    drones = [Drone(i) for i in range(4)]

    # Shared metrics
    metrics     = {"collisions": 0}
    sim_start   = time.time()
    collisions_ref = [0]   # mutable container so Drone.update can write to it

    def reset_all():
        nonlocal sim_start
        for d in drones:
            d.reset()
        metrics["collisions"] = 0
        collisions_ref[0] = 0
        sim_start = time.time()

    # Pre-render static scene onto a surface (roads, buildings, zones)
    # We re-draw each frame because zones have drone-dependent overlays,
    # but buildings/roads are static — drawn first, then drones on top.

    running = True
    while running:
        # ---- Events --------------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    reset_all()

        # ---- Update --------------------------------------------------------
        for drone in drones:
            drone.update(drones, BUILDINGS, collisions_ref)
        metrics["collisions"] = collisions_ref[0]
        elapsed = time.time() - sim_start

        # ---- Draw ----------------------------------------------------------
        screen.fill(C_BG)

        # Simulation area border
        pygame.draw.rect(screen, C_PANEL_LINE, (PAD, TITLE_H + PAD, SIM_W, SIM_H), 1)

        draw_road_markings(screen)
        draw_zones(screen, font_sm)
        draw_buildings(screen, font_sm, font_xs)

        # Draw drones
        for drone in drones:
            drone.draw(screen, font_xs)

        # Target markers (small cross on each zone centre)
        for i, (tx, ty) in enumerate(TARGET_POS):
            color = DRONE_COLORS[i]
            pygame.draw.circle(screen, color, (tx, ty), 5, 1)

        # Stats panel
        draw_panel(screen, font_h, font_md, font_sm, font_xs,
                   drones, metrics, elapsed)

        # Title bar (drawn last so it overlaps cleanly)
        draw_title_bar(screen, font_title, font_xs)

        # FPS counter (bottom-left of sim)
        fps_txt = font_xs.render(f"FPS: {int(clock.get_fps())}", True, (55, 70, 100))
        screen.blit(fps_txt, (PAD + 4, TITLE_H + SIM_H - 16))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
