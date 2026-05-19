"""
simulation/config.py
====================
Central configuration for the Smart City Multi-UAV Surveillance simulation.

All tunable parameters live here. Teammates working on different modules
should import from this file instead of hard-coding values.

To run an experiment with different parameters, either:
  - Edit values here, or
  - Override in your script:  import simulation.config as cfg; cfg.MAX_SPEED = 5.0
"""

# ── Window / layout ────────────────────────────────────────────────────────────
SIM_W   = 660          # simulation canvas width  (px)
SIM_H   = 660          # simulation canvas height (px)
PANEL_W = 340          # right-side metrics panel width
TITLE_H = 42           # top title bar height
WIN_W   = SIM_W + PANEL_W
WIN_H   = SIM_H + TITLE_H
FPS     = 60

# ── Colour palette ─────────────────────────────────────────────────────────────
C_BG          = (14,  17,  28)
C_ROAD        = (30,  37,  56)
C_ROAD_MARK   = (55,  65,  95)
C_BLDG_FILL   = (38,  47,  72)
C_BLDG_BORDER = (70,  88, 130)
C_BLDG_LABEL  = (130, 150, 195)
C_TITLE_BG    = (10,  13,  22)
C_PANEL_BG    = (10,  13,  22)
C_PANEL_LINE  = (32,  42,  68)
C_TEXT_PRI    = (218, 228, 255)
C_TEXT_SEC    = (115, 135, 175)
C_TEXT_OK     = (75,  210, 130)
C_TEXT_WARN   = (255, 180,  50)
C_TEXT_ACT    = (100, 180, 255)
C_TEXT_ERR    = (255,  90,  80)
C_CROWD       = (255, 220, 100)

# ── Drone identity ─────────────────────────────────────────────────────────────
DRONE_COLORS = [(80, 190, 255), (75, 210, 130), (255, 175, 50), (200, 100, 255)]
DRONE_NAMES  = ["UAV-A", "UAV-B", "UAV-C", "UAV-D"]

# ── City grid geometry ─────────────────────────────────────────────────────────
PAD = 2      # border padding (px)
BLK = 200    # city block side length (px)
RD  = 28     # road width (px)

# ── UAV physics ────────────────────────────────────────────────────────────────
DRONE_R       = 11     # drone body radius (px)
DRONE_SPD     = 1.8    # legacy reference speed (kept for compatibility)
MAX_SPEED     = 3.5    # top speed (px/frame)
MAX_ACCEL     = 0.22   # max acceleration magnitude (px/frame²)
FRICTION      = 0.92   # velocity damping per frame (inertia)
MAX_TURN_RATE = 0.12   # max heading rotation per frame (rad)
ARRIVE_D      = 22     # arrival threshold distance (px)

# ── Collision avoidance ────────────────────────────────────────────────────────
AVOID_D_R = 65   # drone-drone repulsion radius (px)
AVOID_D_S = 2.0  # drone-drone repulsion strength
AVOID_O_R = 62   # obstacle repulsion radius (px)
AVOID_O_S = 3.2  # obstacle repulsion strength

# ── Crowd / boid parameters ────────────────────────────────────────────────────
NUM_PEOPLE = 30
PERSON_SPD = 0.55
FLOCK_R    = 65    # boid neighbourhood radius (px)

# ── Crowd hotspot parameters ───────────────────────────────────────────────────
NUM_HOTSPOTS  = 4
HOTSPOT_DRIFT = 0.18    # drift speed (px/frame)
HOTSPOT_PULSE = 0.004   # weight oscillation speed (rad/frame)

# ── Environmental uncertainties ────────────────────────────────────────────────
WIND_STR   = 0.06   # max wind magnitude (px/frame)
GPS_NOISE  = 1.4    # GPS noise std-dev (px)
COMM_DELAY = 8      # communication delay (frames)

# ── Battery ────────────────────────────────────────────────────────────────────
BATT_DRAIN = 0.0025   # battery drain per frame (%)

# ── Sensor ────────────────────────────────────────────────────────────────────
COVERAGE_R = 90    # drone sensor coverage radius (px)

# NOTE: ZONES, BUILDINGS, and BLDG_RECTS are defined in
#       simulation.environment.city_map (they require pygame.Rect at import time).
#       Import them from there, not from here.

