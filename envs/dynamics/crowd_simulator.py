"""
CrowdSimulator — sidewalk-aware pedestrian crowd for STIRS-2025.

Agents navigate on sidewalk strips derived from CityLayout road geometry.
Gathering agents (park / plaza zones) wander within their block interior.
Bus-stop agents wait near road edges, occasionally relocating.

Per-scenario agent counts and zone splits are hardcoded in SCENARIO_CONFIGS
and can be overridden by passing a custom zones list to __init__.

Body budget:
  2 PyBullet bodies per agent (collision cylinder + visual-only head sphere).
  Cylinder body IS collidable → agents show up in drone LiDAR scans.
"""

import math
import numpy as np
import pybullet as p


class CrowdSimulator:
    """
    Sidewalk-aware pedestrian crowd with zone-based density control.

    Agents have three states:
      WALK   — move along sidewalk strip toward a randomly chosen waypoint;
               occasional brief pauses (5% chance when waypoint reached).
      GATHER — slow wander within a park / plaza block (gathering zones).
      WAIT   — stationary for wait_timer steps (bus stop, crossing pause).
    """

    # 12 clothing colour variants
    CLOTHING_COLORS = [
        [0.88, 0.18, 0.18, 1.0],   # red
        [0.20, 0.40, 0.88, 1.0],   # blue
        [0.16, 0.72, 0.20, 1.0],   # green
        [0.90, 0.55, 0.08, 1.0],   # orange
        [0.65, 0.28, 0.80, 1.0],   # purple
        [0.88, 0.82, 0.14, 1.0],   # yellow
        [0.10, 0.68, 0.68, 1.0],   # teal
        [0.88, 0.40, 0.58, 1.0],   # pink
        [0.50, 0.34, 0.18, 1.0],   # brown
        [0.78, 0.78, 0.78, 1.0],   # light grey
        [0.24, 0.24, 0.24, 1.0],   # dark grey
        [0.54, 0.78, 0.36, 1.0],   # lime green
    ]

    # Per-scenario default configurations
    SCENARIO_CONFIGS = {
        'downtown': {
            'zones': [
                {'type': 'sidewalk',  'count': 10},
                {'type': 'bus_stop',  'count': 5},
            ]
        },
        'residential': {
            'zones': [
                {'type': 'park',     'count': 5},
                {'type': 'sidewalk', 'count': 3},
            ]
        },
        'event': {
            'zones': [
                {'type': 'plaza_dense', 'count': 60},
                {'type': 'sidewalk',    'count': 20},
            ]
        },
        'mixed': {
            'zones': [
                {'type': 'sidewalk', 'count': 25},
                {'type': 'park',     'count': 10},
            ]
        },
        'industrial': {
            'zones': [
                {'type': 'sidewalk', 'count': 8},
                {'type': 'park',     'count': 2},
            ]
        },
    }

    # Agent body dimensions (shared across all agents for efficiency)
    _BODY_H  = 0.58   # cylinder half-height (bottom at z=0, top at z=2*0.58=1.16 m)
    _BODY_R  = 0.12   # cylinder radius
    _HEAD_R  = 0.10   # head sphere radius (visual-only)
    _HEAD_Z  = 2.0 * _BODY_H + 0.11   # head centre Z above ground

    def __init__(self, client_id: int, layout, scenario: str,
                 building_specs=None, zones_override=None,
                 rng: np.random.Generator = None):
        self.client_id      = client_id
        self.layout         = layout
        self.scenario       = scenario
        self.building_specs = building_specs or []
        self.rng            = rng or np.random.default_rng(42)

        # Pre-create shared collision / visual shapes (body uses collision for LiDAR)
        self._col_shape = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=self._BODY_R, height=self._BODY_H * 2.0,
            physicsClientId=client_id)
        self._vis_body = p.createVisualShape(
            p.GEOM_CYLINDER, radius=self._BODY_R, length=self._BODY_H * 2.0,
            rgbaColor=[1.0, 1.0, 1.0, 1.0],   # overridden per-agent via changeVisualShape
            physicsClientId=client_id)
        self._vis_head = p.createVisualShape(
            p.GEOM_SPHERE, radius=self._HEAD_R,
            rgbaColor=[0.72, 0.52, 0.38, 1.0],   # neutral skin tone
            physicsClientId=client_id)

        # Build position pools from the city layout
        self._sidewalk_strips  = self._build_sidewalk_strips()
        self._all_sidewalk     = [pt for s in self._sidewalk_strips for pt in s]
        self._plaza_positions  = self._build_gather_positions(['plaza'])
        self._park_positions   = self._build_gather_positions(['park'])
        self._bus_stop_pos     = self._build_bus_stop_positions()

        # Spawn agents
        self.agents: list = []
        cfg = zones_override or self.SCENARIO_CONFIGS.get(
            scenario, {'zones': [{'type': 'sidewalk', 'count': 10}]})
        self._spawn_all(cfg['zones'])

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, dt: float = 0.02):
        """Advance all agents one step and sync PyBullet positions."""
        for a in self.agents:
            if a['state'] == 'wait':
                a['wait_timer'] -= 1
                if a['wait_timer'] <= 0:
                    self._exit_wait(a)
            else:
                self._move(a, dt)
            self._sync(a)

    @property
    def boid_positions(self) -> list:
        """Positions as np.array([x, y]) for each agent — used by reward calc."""
        return [np.array(a['pos']) for a in self.agents]

    # ── Spawning ──────────────────────────────────────────────────────────────

    def _spawn_all(self, zones: list):
        agent_id = 0
        for zone in zones:
            for _ in range(zone.get('count', 0)):
                self._spawn_one(zone['type'], agent_id)
                agent_id += 1

    def _spawn_one(self, zone_type: str, agent_id: int):
        color = list(self.CLOTHING_COLORS[
            int(self.rng.integers(0, len(self.CLOTHING_COLORS)))])
        speed = float(self.rng.uniform(0.8, 1.5))

        pos, pool, state = self._pick_start(zone_type)
        target = self._next_target(pool, pos)
        wait_timer = 0

        if zone_type == 'bus_stop':
            state = 'wait'
            wait_timer = int(self.rng.integers(60, 160))

        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=self._col_shape,
            baseVisualShapeIndex=self._vis_body,
            basePosition=[pos[0], pos[1], self._BODY_H],
            physicsClientId=self.client_id)
        p.changeVisualShape(body_id, -1, rgbaColor=color,
                            physicsClientId=self.client_id)

        head_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=self._vis_head,
            basePosition=[pos[0], pos[1], self._HEAD_Z],
            physicsClientId=self.client_id)

        self.agents.append({
            'agent_id':   agent_id,
            'body_id':    body_id,
            'head_id':    head_id,
            'pos':        list(pos),
            'state':      state,
            'target':     list(target),
            'pool':       pool,
            'pool_type':  zone_type,
            'speed':      speed,
            'wait_timer': wait_timer,
        })

    def _pick_start(self, zone_type: str):
        """Return (pos, pool, initial_state) for a zone type."""
        if zone_type in ('sidewalk', 'walking'):
            pool = self._pick_strip_or_all()
            pos  = self._sample(pool)
            return pos, pool, 'walk'

        if zone_type == 'bus_stop':
            pool = self._bus_stop_pos or self._all_sidewalk or [(0.0, 0.0)]
            pos  = self._sample(pool)
            return pos, pool, 'wait'

        if zone_type == 'plaza_dense':
            pool = self._plaza_positions or self._all_sidewalk or [(0.0, 0.0)]
            pos  = self._sample(pool)
            return pos, pool, 'gather'

        if zone_type == 'park':
            pool = self._park_positions or self._all_sidewalk or [(0.0, 0.0)]
            pos  = self._sample(pool)
            return pos, pool, 'gather'

        # Fallback
        pool = self._all_sidewalk or [(0.0, 0.0)]
        return self._sample(pool), pool, 'walk'

    def _pick_strip_or_all(self) -> list:
        """Return one sidewalk strip (for realistic strip-walking) or all positions."""
        if self._sidewalk_strips:
            idx = int(self.rng.integers(0, len(self._sidewalk_strips)))
            return self._sidewalk_strips[idx]
        return self._all_sidewalk or [(0.0, 0.0)]

    def _sample(self, pool: list) -> tuple:
        if not pool:
            return (0.0, 0.0)
        return pool[int(self.rng.integers(0, len(pool)))]

    # ── Movement ──────────────────────────────────────────────────────────────

    def _move(self, a: dict, dt: float):
        tx, ty = a['target']
        px, py = a['pos']
        dx, dy = tx - px, ty - py
        dist   = math.sqrt(dx * dx + dy * dy)

        walk_spd  = a['speed'] * (0.55 if a['state'] == 'gather' else 1.0)
        step_dist = walk_spd * dt

        if dist < step_dist * 2.0:
            # Reached waypoint
            if a['state'] == 'walk' and float(self.rng.random()) < 0.05:
                a['state']      = 'wait'
                a['wait_timer'] = int(self.rng.integers(40, 100))
            else:
                a['target'] = list(self._next_target(a['pool'], a['pos']))
        else:
            nx = px + step_dist * dx / dist
            ny = py + step_dist * dy / dist
            if not self._in_building(nx, ny):
                a['pos'] = [nx, ny]
            else:
                a['target'] = list(self._next_target(a['pool'], a['pos']))

    def _exit_wait(self, a: dict):
        zone_type = a['pool_type']
        if zone_type == 'bus_stop':
            # 25% chance to walk to another bus stop position; else keep waiting
            if float(self.rng.random()) < 0.25:
                a['state']  = 'walk'
                a['target'] = list(self._next_target(a['pool'], a['pos']))
            else:
                a['wait_timer'] = int(self.rng.integers(80, 200))
        else:
            resume = 'gather' if zone_type in ('plaza_dense', 'park') else 'walk'
            a['state']  = resume
            a['target'] = list(self._next_target(a['pool'], a['pos']))

    def _next_target(self, pool: list, current_pos=None) -> tuple:
        if not pool:
            return current_pos or (0.0, 0.0)
        return pool[int(self.rng.integers(0, len(pool)))]

    def _sync(self, a: dict):
        x, y = a['pos']
        p.resetBasePositionAndOrientation(
            a['body_id'], [x, y, self._BODY_H], [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(
            a['head_id'], [x, y, self._HEAD_Z], [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client_id)

    # ── Position pool builders ────────────────────────────────────────────────

    def _build_sidewalk_strips(self) -> list:
        """
        One list of (x, y) waypoints per sidewalk strip.
        Strips run parallel to roads on both sides, offset by
        road_half + sidewalk_half (0.8 m sidewalk width assumed).
        """
        strips   = []
        rw_half  = self.layout.road_width / 2.0
        sw_off   = rw_half + 0.4   # road_half + sidewalk_half(=0.4)
        SPACING  = 2.0             # waypoints every 2 m along strip
        city_hx  = self.layout.city_size[0] / 2.0
        city_hy  = self.layout.city_size[1] / 2.0

        for road in self.layout.roads:
            if road.orientation == 'vertical':
                for side in (-1, 1):
                    sx = road.x + side * sw_off
                    if not (-city_hx < sx < city_hx):
                        continue
                    strip = []
                    y = -city_hy + SPACING / 2.0
                    while y <= city_hy - SPACING / 2.0 + 1e-6:
                        if not self._in_building(sx, y):
                            strip.append((sx, y))
                        y += SPACING
                    if len(strip) >= 2:
                        strips.append(strip)
            else:   # horizontal
                for side in (-1, 1):
                    sy = road.y + side * sw_off
                    if not (-city_hy < sy < city_hy):
                        continue
                    strip = []
                    x = -city_hx + SPACING / 2.0
                    while x <= city_hx - SPACING / 2.0 + 1e-6:
                        if not self._in_building(x, sy):
                            strip.append((x, sy))
                        x += SPACING
                    if len(strip) >= 2:
                        strips.append(strip)
        return strips

    def _build_gather_positions(self, block_types: list) -> list:
        """Interior grid positions within park / plaza blocks."""
        positions = []
        SPACING   = 1.2
        for block in self.layout.blocks:
            if block.block_type not in block_types:
                continue
            hx = block.width  / 2.0 - 0.5   # 0.5 m margin from block edge
            hy = block.depth  / 2.0 - 0.5
            x  = block.x - hx
            while x <= block.x + hx + 1e-6:
                y = block.y - hy
                while y <= block.y + hy + 1e-6:
                    if not self._in_building(x, y, margin=0.1):
                        positions.append((x, y))
                    y += SPACING
                x += SPACING
        return positions

    def _build_bus_stop_positions(self) -> list:
        """
        Approximate bus-stop positions: endpoints of sidewalk strips
        that lie in the outer 60% of the city (near road ends).
        """
        city_hx = self.layout.city_size[0] / 2.0
        city_hy = self.layout.city_size[1] / 2.0
        edge_th = 0.55   # positions beyond 55% of city half-extent qualify
        result  = []
        for strip in self._sidewalk_strips:
            if strip:
                result.append(strip[0])
                result.append(strip[-1])
        outer = [(x, y) for (x, y) in result
                 if abs(x) > city_hx * edge_th or abs(y) > city_hy * edge_th]
        return outer if outer else result[:10]

    def _in_building(self, x: float, y: float, margin: float = 0.25) -> bool:
        for spec in self.building_specs:
            cx = float(spec['center'][0])
            cy = float(spec['center'][1])
            hx = float(spec['half_extents'][0])
            hy = float(spec['half_extents'][1])
            if abs(x - cx) < hx + margin and abs(y - cy) < hy + margin:
                return True
        return False
