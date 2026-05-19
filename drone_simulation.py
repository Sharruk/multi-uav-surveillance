"""
Smart City Multi-UAV Surveillance — Research Prototype
Python + Pygame  |  pip install pygame  |  python drone_simulation.py
SPACE=Reset  ESC=Quit
"""
import pygame, math, time, random
from collections import deque

SIM_W, SIM_H = 660, 660
PANEL_W = 340
TITLE_H = 42
WIN_W   = SIM_W + PANEL_W
WIN_H   = SIM_H + TITLE_H
FPS     = 60

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

DRONE_COLORS = [(80,190,255),(75,210,130),(255,175,50),(200,100,255)]
DRONE_NAMES  = ["UAV-A","UAV-B","UAV-C","UAV-D"]

PAD, BLK, RD = 2, 200, 28

def blk(c, r):
    return (PAD + c*(BLK+RD), TITLE_H + PAD + r*(BLK+RD))

V_ROADS = [(PAD+BLK, PAD+BLK+RD),(PAD+2*BLK+RD, PAD+2*BLK+2*RD)]
H_ROADS = [(TITLE_H+PAD+BLK, TITLE_H+PAD+BLK+RD),
           (TITLE_H+PAD+2*BLK+RD, TITLE_H+PAD+2*BLK+2*RD)]

BUILDINGS = [
    (pygame.Rect(blk(0,0)[0]+3,blk(0,0)[1]+3,BLK-6,BLK-6),"COM","Commercial\nDistrict"),
    (pygame.Rect(blk(1,0)[0]+3,blk(1,0)[1]+3,BLK-6,BLK-6),"CTH","City Hall"),
    (pygame.Rect(blk(2,0)[0]+3,blk(2,0)[1]+3,BLK-6,BLK-6),"TEC","Tech Hub"),
    (pygame.Rect(blk(1,1)[0]+3,blk(1,1)[1]+3,BLK-6,BLK-6),"MAL","Central\nMall"),
    (pygame.Rect(blk(2,2)[0]+3,blk(2,2)[1]+3,BLK-6,BLK-6),"UNI","University"),
]

ZONE_COLORS = [((60,185,90),(80,220,110)),((220,110,40),(255,140,55)),
               ((80,155,255),(110,185,255)),((200,80,80),(230,110,100))]
ZONES = []
for _idx,(_c,_r,_lbl) in enumerate([(0,1,"Zone-A"),(2,1,"Zone-B"),(0,2,"Zone-C"),(1,2,"Zone-D")]):
    _bx,_by = blk(_c,_r)
    ZONES.append({"rect":pygame.Rect(_bx+3,_by+3,BLK-6,BLK-6),
                  "label":_lbl,"fill":ZONE_COLORS[_idx][0],"border":ZONE_COLORS[_idx][1]})

DRONE_R    = 11
DRONE_SPD  = 1.8
ARRIVE_D   = 22
AVOID_D_R  = 65
AVOID_D_S  = 2.0
AVOID_O_R  = 62
AVOID_O_S  = 3.2
NUM_PEOPLE = 30
PERSON_SPD = 0.55
FLOCK_R    = 65
WIND_STR   = 0.06
GPS_NOISE  = 1.4
COMM_DELAY = 8
BATT_DRAIN = 0.0025
COVERAGE_R = 90

# UAV physics
MAX_SPEED     = 3.5
MAX_ACCEL     = 0.22
FRICTION      = 0.92
MAX_TURN_RATE = 0.12

# Crowd hotspots
NUM_HOTSPOTS  = 4
HOTSPOT_DRIFT = 0.18
HOTSPOT_PULSE = 0.004

BLDG_RECTS = [b[0] for b in BUILDINGS]

def _in_building(x, y):
    for r in BLDG_RECTS:
        if r.collidepoint(x, y):
            return True
    return False

def _rand_open_pos():
    for _ in range(200):
        x = random.uniform(PAD+10, PAD+SIM_W-10)
        y = random.uniform(TITLE_H+10, TITLE_H+SIM_H-10)
        if not _in_building(x, y):
            return x, y
    return PAD+SIM_W//2, TITLE_H+SIM_H//2


class Hotspot:
    """Drifting crowd-attraction centre with a pulsing weight."""
    def __init__(self):
        self.x, self.y = _rand_open_pos()
        self.angle  = random.uniform(0, math.tau)
        self.weight = random.uniform(0.5, 1.0)
        self.phase  = random.uniform(0, math.tau)

    def reset(self):
        self.x, self.y = _rand_open_pos()
        self.angle  = random.uniform(0, math.tau)
        self.weight = random.uniform(0.5, 1.0)
        self.phase  = random.uniform(0, math.tau)

    def update(self):
        self.phase  += HOTSPOT_PULSE
        self.weight  = 0.5 + 0.5 * math.sin(self.phase)
        if random.random() < 0.01:
            self.angle += random.uniform(-0.5, 0.5)
        nx = self.x + math.cos(self.angle) * HOTSPOT_DRIFT
        ny = self.y + math.sin(self.angle) * HOTSPOT_DRIFT
        if nx < PAD+20 or nx > PAD+SIM_W-20:
            self.angle = math.pi - self.angle
        if ny < TITLE_H+20 or ny > TITLE_H+SIM_H-20:
            self.angle = -self.angle
        nx = max(PAD+20, min(PAD+SIM_W-20, nx))
        ny = max(TITLE_H+20, min(TITLE_H+SIM_H-20, ny))
        if not _in_building(nx, ny):
            self.x, self.y = nx, ny
        else:
            self.angle += math.pi * random.uniform(0.4, 0.8)

    def draw(self, surf):
        ix, iy = int(self.x), int(self.y)
        r = int(40 + 20 * self.weight)
        s = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 200, 60, int(30+25*self.weight)), (r,r), r)
        surf.blit(s, (ix-r, iy-r))
        pygame.draw.circle(surf, (255,200,60), (ix,iy), 5, 1)
        pygame.draw.line(surf, (255,200,60), (ix-12,iy), (ix+12,iy), 1)
        pygame.draw.line(surf, (255,200,60), (ix,iy-12), (ix,iy+12), 1)


class Person:
    def __init__(self, hx=None, hy=None):
        if hx is not None:
            for _ in range(40):
                x = max(PAD+8, min(PAD+SIM_W-8, hx + random.gauss(0, 22)))
                y = max(TITLE_H+8, min(TITLE_H+SIM_H-8, hy + random.gauss(0, 22)))
                if not _in_building(x, y):
                    self.x, self.y = x, y; break
            else:
                self.x, self.y = _rand_open_pos()
        else:
            self.x, self.y = _rand_open_pos()
        self.angle = random.uniform(0, math.tau)
        self.spd   = PERSON_SPD * random.uniform(0.6, 1.4)
        self.wander_timer = random.randint(0, 60)

    def update(self, others, hotspots):
        self.wander_timer -= 1
        if self.wander_timer <= 0:
            self.angle += random.uniform(-0.9, 0.9)
            self.wander_timer = random.randint(30, 90)

        cx = cy = 0.0
        sx = sy = 0.0
        ax = ay = 0.0
        n = 0
        for o in others:
            if o is self:
                continue
            d = math.hypot(self.x-o.x, self.y-o.y)
            if d < FLOCK_R and d > 0:
                cx += o.x; cy += o.y
                ax += math.cos(o.angle); ay += math.sin(o.angle)
                n += 1
                if d < 18:
                    sx -= (o.x-self.x)/d
                    sy -= (o.y-self.y)/d
        if n:
            cx /= n; cy /= n
            cohx = (cx-self.x)*0.002
            cohy = (cy-self.y)*0.002
            alx  = ax/n*0.04; aly = ay/n*0.04
            self.angle += math.atan2(cohy+aly+sy*0.05, cohx+alx+sx*0.05)*0.3

        # Hotspot attraction
        if hotspots:
            hs = min(hotspots, key=lambda h: math.hypot(self.x-h.x, self.y-h.y))
            ha = math.atan2(hs.y - self.y, hs.x - self.x)
            self.angle += math.sin(ha - self.angle) * hs.weight * 0.06

        self.angle = self.angle % math.tau
        nx = self.x + math.cos(self.angle)*self.spd
        ny = self.y + math.sin(self.angle)*self.spd

        if _in_building(nx, ny):
            self.angle += math.pi * random.uniform(0.4, 0.6)
            nx = self.x + math.cos(self.angle)*self.spd
            ny = self.y + math.sin(self.angle)*self.spd

        self.x = max(PAD+4, min(PAD+SIM_W-4, nx))
        self.y = max(TITLE_H+4, min(TITLE_H+SIM_H-4, ny))

        if _in_building(self.x, self.y):
            self.x, self.y = _rand_open_pos()


class CrowdSystem:
    def __init__(self):
        self.hotspots = [Hotspot() for _ in range(NUM_HOTSPOTS)]
        self.people   = []
        per_hs = NUM_PEOPLE // NUM_HOTSPOTS
        for hs in self.hotspots:
            for _ in range(per_hs):
                self.people.append(Person(hs.x, hs.y))
        while len(self.people) < NUM_PEOPLE:
            self.people.append(Person())
        self.center         = (SIM_W//2, TITLE_H+SIM_H//2)
        self.density        = 0.0
        self.monitored      = 0.0
        self.hotspot_counts = [0] * NUM_HOTSPOTS

    def update(self, drones=None):
        for hs in self.hotspots:
            hs.update()
        for p in self.people:
            p.update(self.people, self.hotspots)
        xs = [p.x for p in self.people]
        ys = [p.y for p in self.people]
        self.center  = (sum(xs)/len(xs), sum(ys)/len(ys))
        cx, cy = self.center
        self.density = sum(1 for p in self.people
                          if math.hypot(p.x-cx,p.y-cy)<80) / NUM_PEOPLE
        for i, hs in enumerate(self.hotspots):
            self.hotspot_counts[i] = sum(
                1 for p in self.people if math.hypot(p.x-hs.x,p.y-hs.y)<70)
        if drones:
            self.monitored = sum(
                1 for p in self.people
                if any(math.hypot(p.x-d.x,p.y-d.y)<COVERAGE_R for d in drones)
            ) / NUM_PEOPLE * 100
        else:
            self.monitored = 0.0

    def draw(self, surf):
        for hs in self.hotspots:
            hs.draw(surf)
        cx, cy = int(self.center[0]), int(self.center[1])
        r = 28
        s = pygame.Surface((r*2,r*2), pygame.SRCALPHA)
        pygame.draw.circle(s,(255,220,100,35),(r,r),r)
        surf.blit(s,(cx-r,cy-r))
        pygame.draw.circle(surf,(255,200,60),  (cx,cy),6,1)
        pygame.draw.line(surf,(255,200,60),(cx-10,cy),(cx+10,cy),1)
        pygame.draw.line(surf,(255,200,60),(cx,cy-10),(cx,cy+10),1)
        for p in self.people:
            pygame.draw.circle(surf,C_CROWD,(int(p.x),int(p.y)),3)
            pygame.draw.circle(surf,(200,160,40),(int(p.x),int(p.y)),3,1)


class Drone:
    def __init__(self, idx, crowd):
        self.idx    = idx
        self.name   = DRONE_NAMES[idx]
        self.color  = DRONE_COLORS[idx]
        self.crowd  = crowd
        self.x = float(ZONES[idx]["rect"].centerx)
        self.y = float(ZONES[idx]["rect"].centery)
        self.vx = self.vy = 0.0
        self.ax = self.ay = 0.0
        self.heading = 0.0
        self.tx = self.x
        self.ty = self.y
        self.target_buf = deque(maxlen=COMM_DELAY)
        for _ in range(COMM_DELAY):
            self.target_buf.append(crowd.center)
        self.wind_x = random.uniform(-WIND_STR, WIND_STR)
        self.wind_y = random.uniform(-WIND_STR, WIND_STR)
        self.wind_timer = 0
        self.battery   = 100.0
        self.distance  = 0.0
        self.status    = "Tracking"
        self.collision_avoids = 0
        self.sim_start = time.time()
        self.gps_ox = 0.0
        self.gps_oy = 0.0
        self.gps_timer = 0

    def reset(self, crowd):
        self.crowd = crowd
        self.x = float(ZONES[self.idx]["rect"].centerx)
        self.y = float(ZONES[self.idx]["rect"].centery)
        self.vx = self.vy = 0.0
        self.ax = self.ay = 0.0
        self.heading = 0.0
        self.tx = self.x; self.ty = self.y
        self.target_buf = deque(maxlen=COMM_DELAY)
        for _ in range(COMM_DELAY):
            self.target_buf.append(crowd.center)
        self.wind_x = random.uniform(-WIND_STR, WIND_STR)
        self.wind_y = random.uniform(-WIND_STR, WIND_STR)
        self.wind_timer = 0
        self.battery = 100.0
        self.distance = 0.0
        self.status   = "Tracking"
        self.collision_avoids = 0
        self.sim_start = time.time()

    def perceived_pos(self):
        return self.x + self.gps_ox, self.y + self.gps_oy

    def update(self, all_drones, col_ref):
        self.battery = max(0.0, self.battery - BATT_DRAIN)

        # Wind drift update
        self.wind_timer -= 1
        if self.wind_timer <= 0:
            self.wind_x += random.uniform(-0.02, 0.02)
            self.wind_y += random.uniform(-0.02, 0.02)
            self.wind_x = max(-WIND_STR, min(WIND_STR, self.wind_x))
            self.wind_y = max(-WIND_STR, min(WIND_STR, self.wind_y))
            self.wind_timer = random.randint(20, 60)

        # GPS noise update
        self.gps_timer -= 1
        if self.gps_timer <= 0:
            self.gps_ox = random.gauss(0, GPS_NOISE)
            self.gps_oy = random.gauss(0, GPS_NOISE)
            self.gps_timer = random.randint(5, 15)

        # Comms-delayed crowd center
        self.target_buf.append(self.crowd.center)
        delayed_cx, delayed_cy = self.target_buf[0]

        # Assign coverage offset so drones spread around crowd center
        offsets = [(-40,-40),(40,-40),(-40,40),(40,40)]
        ox, oy = offsets[self.idx]
        self.tx = delayed_cx + ox
        self.ty = delayed_cy + oy

        px, py = self.perceived_pos()
        dx = self.tx - px
        dy = self.ty - py
        d  = math.hypot(dx, dy)

        top_spd = MAX_SPEED * (0.5 if self.battery < 15 else 1.0)
        if d < ARRIVE_D:
            self.vx *= FRICTION; self.vy *= FRICTION
            self.status = "On-Station"
        else:
            desired_vx = (dx/d) * top_spd
            desired_vy = (dy/d) * top_spd
            self.ax = desired_vx - self.vx
            self.ay = desired_vy - self.vy
            a_mag = math.hypot(self.ax, self.ay)
            if a_mag > MAX_ACCEL:
                self.ax = self.ax/a_mag*MAX_ACCEL
                self.ay = self.ay/a_mag*MAX_ACCEL
            self.vx = (self.vx + self.ax) * FRICTION
            self.vy = (self.vy + self.ay) * FRICTION
            spd = math.hypot(self.vx, self.vy)
            if spd > top_spd:
                self.vx = self.vx/spd*top_spd
                self.vy = self.vy/spd*top_spd
            self.status = "Tracking"

        avoiding = False
        # Drone-drone avoidance
        for o in all_drones:
            if o is self: continue
            dd = math.hypot(self.x-o.x, self.y-o.y)
            if 0 < dd < AVOID_D_R:
                avoiding = True
                if self.idx < o.idx:
                    col_ref[0] += 1
                    self.collision_avoids += 1
                s = (AVOID_D_R-dd)/AVOID_D_R * AVOID_D_S
                self.vx -= s*(o.x-self.x)/dd
                self.vy -= s*(o.y-self.y)/dd

        # Building avoidance
        for br in BLDG_RECTS:
            cx2 = max(br.left, min(self.x, br.right))
            cy2 = max(br.top,  min(self.y, br.bottom))
            dd = math.hypot(self.x-cx2, self.y-cy2)
            if 0 < dd < AVOID_O_R:
                avoiding = True
                s = (AVOID_O_R-dd)/AVOID_O_R * AVOID_O_S
                self.vx += s*(self.x-cx2)/dd
                self.vy += s*(self.y-cy2)/dd

        if avoiding:
            self.status = "Avoiding"

        # Apply wind
        self.vx += self.wind_x
        self.vy += self.wind_y

        if self.battery <= 0:
            self.status = "RTB"
            self.vx *= 0.3; self.vy *= 0.3

        # Smooth heading toward velocity direction
        _spd = math.hypot(self.vx, self.vy)
        if _spd > 0.05:
            th = math.atan2(self.vy, self.vx)
            dh = (th - self.heading + math.pi) % math.tau - math.pi
            self.heading = (self.heading + max(-MAX_TURN_RATE, min(MAX_TURN_RATE, dh))) % math.tau

        prev_x, prev_y = self.x, self.y
        self.x += self.vx
        self.y += self.vy
        self.x = max(PAD+DRONE_R, min(PAD+SIM_W-DRONE_R, self.x))
        self.y = max(TITLE_H+PAD+DRONE_R, min(TITLE_H+PAD+SIM_H-DRONE_R, self.y))
        self.distance += math.hypot(self.x-prev_x, self.y-prev_y)

    def draw(self, surf, font_xs):
        ix, iy = int(self.x), int(self.y)
        tx, ty = int(self.tx), int(self.ty)
        _draw_dash(surf, self.color, (ix,iy),(tx,ty))
        # Quadrotor arms at ±45° and ±135° from heading
        arm_len = DRONE_R + 5
        dark = tuple(max(0, c-70) for c in self.color)
        for ang in (self.heading+math.pi/4, self.heading-math.pi/4,
                    self.heading+3*math.pi/4, self.heading-3*math.pi/4):
            ex2 = ix + int(math.cos(ang)*arm_len)
            ey2 = iy + int(math.sin(ang)*arm_len)
            pygame.draw.line(surf, dark, (ix,iy), (ex2,ey2), 2)
            pygame.draw.circle(surf, dark, (ex2,ey2), 4)
        # Body core
        pygame.draw.circle(surf, self.color, (ix,iy), DRONE_R-2)
        pygame.draw.circle(surf, (200,215,255), (ix,iy), DRONE_R-2, 2)
        # Heading arrow
        if math.hypot(self.vx, self.vy) > 0.05:
            ex = ix + int(math.cos(self.heading)*(DRONE_R+7))
            ey = iy + int(math.sin(self.heading)*(DRONE_R+7))
            pygame.draw.line(surf,(255,255,255),(ix,iy),(ex,ey),2)
        # Coverage ring
        s2 = pygame.Surface((COVERAGE_R*2,COVERAGE_R*2),pygame.SRCALPHA)
        pygame.draw.circle(s2,(*self.color,18),(COVERAGE_R,COVERAGE_R),COVERAGE_R)
        surf.blit(s2,(ix-COVERAGE_R,iy-COVERAGE_R))
        pygame.draw.circle(surf,(*self.color,80),(ix,iy),COVERAGE_R,1)
        # Label
        lbl = font_xs.render(self.name, True, self.color)
        surf.blit(lbl,(ix-lbl.get_width()//2, iy-DRONE_R-18))


# ── Drawing Helpers ──────────────────────────────────────────────────────────

def _draw_dash(surf, color, start, end, seg=6, gap=4):
    dx = end[0]-start[0]; dy = end[1]-start[1]
    total = math.hypot(dx, dy)
    if total < 1: return
    ux, uy = dx/total, dy/total
    pos = 0; on = True
    dim = tuple(max(0,c-110) for c in color)
    while pos < total:
        ln = seg if on else gap
        if on:
            x1=int(start[0]+ux*pos);y1=int(start[1]+uy*pos)
            x2=int(start[0]+ux*min(pos+ln,total));y2=int(start[1]+uy*min(pos+ln,total))
            pygame.draw.line(surf,dim,(x1,y1),(x2,y2),1)
        pos+=ln; on=not on


def draw_roads(surf):
    for x0,x1 in V_ROADS:
        pygame.draw.rect(surf,C_ROAD,(x0,TITLE_H,x1-x0,SIM_H))
    for y0,y1 in H_ROADS:
        pygame.draw.rect(surf,C_ROAD,(PAD,y0,SIM_W,y1-y0))
    dash,gap=14,10
    for x0,x1 in V_ROADS:
        cx=(x0+x1)//2; y=TITLE_H
        while y<TITLE_H+SIM_H:
            pygame.draw.line(surf,C_ROAD_MARK,(cx,y),(cx,min(y+dash,TITLE_H+SIM_H)),1)
            y+=dash+gap
    for y0,y1 in H_ROADS:
        cy=(y0+y1)//2; x=PAD
        while x<PAD+SIM_W:
            pygame.draw.line(surf,C_ROAD_MARK,(x,cy),(min(x+dash,PAD+SIM_W),cy),1)
            x+=dash+gap


def draw_buildings(surf, fsm, fxs):
    for rect,short,full in BUILDINGS:
        pygame.draw.rect(surf,C_BLDG_FILL,  rect,border_radius=3)
        pygame.draw.rect(surf,C_BLDG_BORDER,rect,1,border_radius=3)
        badge=fxs.render(short,True,C_BLDG_BORDER)
        surf.blit(badge,(rect.x+5,rect.y+5))
        lines=full.split("\n")
        th=len(lines)*(fsm.get_height()+2)
        sy=rect.centery-th//2
        for i,ln in enumerate(lines):
            t=fsm.render(ln,True,C_BLDG_LABEL)
            surf.blit(t,(rect.centerx-t.get_width()//2,sy+i*(fsm.get_height()+2)))


def draw_zones(surf, fsm):
    for z in ZONES:
        r=z["rect"]; fc=z["fill"]; bc=z["border"]
        zs=pygame.Surface((r.w,r.h),pygame.SRCALPHA)
        zs.fill((*fc,22)); surf.blit(zs,(r.x,r.y))
        pygame.draw.rect(surf,bc,r,1,border_radius=2)
        lbl=fsm.render(z["label"],True,bc)
        surf.blit(lbl,(r.centerx-lbl.get_width()//2,r.bottom-fsm.get_height()-5))


def draw_title(surf, ft, fxs):
    pygame.draw.rect(surf,C_TITLE_BG,(0,0,WIN_W,TITLE_H))
    pygame.draw.line(surf,C_PANEL_LINE,(0,TITLE_H),(WIN_W,TITLE_H),1)
    t=ft.render("Smart City Multi-UAV Crowd Surveillance",True,C_TEXT_PRI)
    surf.blit(t,(12,TITLE_H//2-t.get_height()//2))
    s=fxs.render("Research Prototype  |  Python + Pygame",True,C_TEXT_SEC)
    surf.blit(s,(WIN_W-s.get_width()-10,TITLE_H//2-s.get_height()//2))


def _batt_color(b):
    if b > 50: return C_TEXT_OK
    if b > 20: return C_TEXT_WARN
    return C_TEXT_ERR


def draw_panel(surf, fh, fsm, fxs, drones, crowd, metrics, elapsed, paused):
    px = SIM_W; py = TITLE_H
    pygame.draw.rect(surf,C_PANEL_BG,(px,py,PANEL_W,SIM_H))
    pygame.draw.line(surf,C_PANEL_LINE,(px,py),(px,py+SIM_H),1)
    y = py+10; mg=14

    def row(lbl, val, lc=C_TEXT_SEC, vc=C_TEXT_PRI):
        nonlocal y
        ls=fsm.render(lbl,True,lc); vs=fsm.render(str(val),True,vc)
        surf.blit(ls,(px+mg,y)); surf.blit(vs,(px+PANEL_W-vs.get_width()-mg,y))
        y+=ls.get_height()+4

    def sec(title):
        nonlocal y
        y+=5
        pygame.draw.line(surf,C_PANEL_LINE,(px+mg,y),(px+PANEL_W-mg,y),1)
        y+=5
        h=fxs.render(title.upper(),True,C_TEXT_SEC)
        surf.blit(h,(px+mg,y)); y+=h.get_height()+5

    hdr=fh.render("Mission Statistics",True,C_TEXT_PRI)
    surf.blit(hdr,(px+PANEL_W//2-hdr.get_width()//2,y)); y+=hdr.get_height()+3

    sec("Overview")
    mm=int(elapsed//60); ss=int(elapsed%60)
    row("Elapsed Time",       f"{mm:02d}:{ss:02d}")
    row("Status", "PAUSED" if paused else "RUNNING",
        vc=C_TEXT_WARN if paused else C_TEXT_OK)
    row("Crowd Density",      f"{crowd.density*100:.0f}%")
    row("Monitored",          f"{crowd.monitored:.0f}%", vc=C_TEXT_ACT)
    ccx,ccy=crowd.center
    row("Crowd Center",       f"({ccx:.0f}, {ccy-TITLE_H:.0f})")

    sec("Research Metrics")
    row("Avoidance Events",   metrics["collisions"])
    row("Coverage Area",      f"{metrics['coverage']:.1f}%",  vc=C_TEXT_ACT)
    row("Tracking Accuracy",  f"{metrics['accuracy']:.1f}%",  vc=C_TEXT_OK if metrics['accuracy']>70 else C_TEXT_WARN)
    row("Total Distance",     f"{metrics['total_dist']:.0f}m")
    row("Wind Speed",         f"{metrics['wind_spd']:.3f} m/s")
    row("GPS Noise",          f"±{GPS_NOISE:.1f}px")
    row("Comm Delay",         f"{COMM_DELAY} frames")

    sec("Crowd Hotspots")
    for i, cnt in enumerate(crowd.hotspot_counts):
        row(f"  Hotspot {i+1}", f"{cnt} people",
            vc=C_TEXT_WARN if cnt > NUM_PEOPLE//4 else C_TEXT_PRI)

    sec("UAV Fleet Status")
    bar_w = PANEL_W - mg*2 - 55
    for d in drones:
        # Name + status
        nc=d.color
        ns=fsm.render(d.name,True,nc)
        surf.blit(ns,(px+mg,y))
        stc=(C_TEXT_OK if d.status=="On-Station" else
             C_TEXT_WARN if d.status=="Avoiding" else
             C_TEXT_ERR if d.status=="RTB" else C_TEXT_ACT)
        ss2=fxs.render(d.status,True,stc)
        surf.blit(ss2,(px+PANEL_W-ss2.get_width()-mg,y)); y+=ns.get_height()+2
        # Battery bar
        bc2=_batt_color(d.battery)
        pygame.draw.rect(surf,C_PANEL_LINE,(px+mg,y,bar_w,7),border_radius=2)
        fill=max(2,int(bar_w*d.battery/100))
        pygame.draw.rect(surf,bc2,(px+mg,y,fill,7),border_radius=2)
        bv=fxs.render(f"{d.battery:.0f}%",True,bc2)
        surf.blit(bv,(px+mg+bar_w+4,y-1)); y+=11
        # Speed + Distance
        spd_px = math.hypot(d.vx, d.vy)
        sv=fxs.render(f"Spd: {spd_px:.2f} px/f | Dist: {d.distance*0.1:.0f}m",True,C_TEXT_SEC)
        surf.blit(sv,(px+mg,y)); y+=sv.get_height()+2
        av=fxs.render(f"Avoid events: {d.collision_avoids}",True,C_TEXT_SEC)
        surf.blit(av,(px+mg,y)); y+=av.get_height()+5

    y=py+SIM_H-28
    pygame.draw.line(surf,C_PANEL_LINE,(px+mg,y),(px+PANEL_W-mg,y),1)
    y+=7
    hint=fxs.render("SPACE: Reset    ESC: Quit    P: Pause",True,C_TEXT_SEC)
    surf.blit(hint,(px+PANEL_W//2-hint.get_width()//2,y))


def calc_metrics(drones, crowd):
    # Coverage: sample grid points, check if any drone within COVERAGE_R
    step=40; covered=0; total=0
    for gx in range(PAD, PAD+SIM_W, step):
        for gy in range(TITLE_H, TITLE_H+SIM_H, step):
            if _in_building(gx,gy): continue
            total+=1
            if any(math.hypot(d.x-gx,d.y-gy)<COVERAGE_R for d in drones):
                covered+=1
    coverage = (covered/total*100) if total else 0
    # Tracking accuracy: nearest drone to crowd center
    cx,cy=crowd.center
    min_d=min(math.hypot(d.x-cx,d.y-cy) for d in drones)
    accuracy=max(0, 100-min_d*0.4)
    total_dist=sum(d.distance*0.1 for d in drones)
    avg_wind=sum(math.hypot(d.wind_x,d.wind_y) for d in drones)/len(drones)
    return {"coverage":coverage,"accuracy":accuracy,
            "total_dist":total_dist,"wind_spd":avg_wind}


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    screen=pygame.display.set_mode((WIN_W,WIN_H))
    pygame.display.set_caption("Smart City UAV Surveillance — Research Prototype")
    clock=pygame.time.Clock()

    ft  =pygame.font.SysFont("segoeui",  15,bold=True)
    fh  =pygame.font.SysFont("segoeui",  14,bold=True)
    fsm =pygame.font.SysFont("segoeui",  12)
    fxs =pygame.font.SysFont("consolas", 10)

    crowd  = CrowdSystem()
    drones = [Drone(i,crowd) for i in range(4)]
    col_ref=[0]
    metrics={"collisions":0,"coverage":0.0,"accuracy":100.0,
             "total_dist":0.0,"wind_spd":0.0}
    sim_start=time.time()
    paused=False
    metric_timer=0

    def reset_all():
        nonlocal sim_start, crowd
        crowd=CrowdSystem()
        for d in drones: d.reset(crowd)
        col_ref[0]=0
        metrics.update(collisions=0,coverage=0.0,accuracy=100.0,
                       total_dist=0.0,wind_spd=0.0)
        sim_start=time.time()

    running=True
    while running:
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: running=False
            if ev.type==pygame.KEYDOWN:
                if ev.key==pygame.K_ESCAPE: running=False
                if ev.key==pygame.K_SPACE:  reset_all()
                if ev.key==pygame.K_p:      paused=not paused

        if not paused:
            crowd.update(drones)
            for d in drones:
                d.update(drones, col_ref)
            metrics["collisions"]=col_ref[0]
            metric_timer+=1
            if metric_timer>=30:
                m=calc_metrics(drones,crowd)
                metrics.update(m); metric_timer=0

        elapsed=time.time()-sim_start

        screen.fill(C_BG)
        pygame.draw.rect(screen,C_PANEL_LINE,(PAD,TITLE_H+PAD,SIM_W,SIM_H),1)
        draw_roads(screen)
        draw_zones(screen,fsm)
        crowd.draw(screen)
        draw_buildings(screen,fsm,fxs)
        for d in drones:
            d.draw(screen,fxs)
        draw_panel(screen,fh,fsm,fxs,drones,crowd,metrics,elapsed,paused)
        draw_title(screen,ft,fxs)

        fps_t=fxs.render(f"FPS:{int(clock.get_fps())}",True,(55,70,100))
        screen.blit(fps_t,(PAD+4,TITLE_H+SIM_H-16))
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__=="__main__":
    main()
