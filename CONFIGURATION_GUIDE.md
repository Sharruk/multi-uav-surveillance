# Configuration Guide — STIRS-2025

How to tune, extend, and create custom environments for the multi-UAV surveillance simulation.

---

## Table of Contents

1. [YAML Configuration Reference](#yaml-configuration-reference)
2. [Per-Parameter Tuning Guide](#per-parameter-tuning-guide)
3. [Performance vs Quality Trade-offs](#performance-vs-quality-trade-offs)
4. [Creating a Custom Scenario](#creating-a-custom-scenario)
5. [Programmatic Configuration (no YAML)](#programmatic-configuration-no-yaml)
6. [Environment Constructor Reference](#environment-constructor-reference)

---

## YAML Configuration Reference

The primary config file is `config/environment_config.yaml`. All keys are optional — the system falls back to per-scenario defaults if a key is missing.

```yaml
# ── Active scenario ─────────────────────────────────────────────────────────
scenario: "downtown"          # downtown | residential | event | mixed | industrial
use_scenario_system: true     # must be true for Phase 3 features (crowd, birds, wind)

# ── Terrain ──────────────────────────────────────────────────────────────────
terrain:
  city_size: [26.0, 26.0]     # [width, depth] in meters
  block_size: 6.0             # city block side length (meters)
  road_width: 2.0             # road lane width (meters)
  sidewalk_width: 0.8         # sidewalk width (meters, informational only)

# ── Buildings ────────────────────────────────────────────────────────────────
buildings:
  height_range: [4.0, 16.0]  # [min, max] height in meters
  density: "high"             # low | medium | high
  style: "mixed"              # modern | traditional | mixed

# ── Crowd simulation ─────────────────────────────────────────────────────────
crowd:
  enabled: true

  # Per-scenario zone definitions — override CrowdSimulator.SCENARIO_CONFIGS
  downtown:
    total_agents: 15
    zones:
      - {type: "sidewalk",  count: 10}
      - {type: "bus_stop",  count: 5}

  residential:
    total_agents: 8
    zones:
      - {type: "park",     count: 5}
      - {type: "sidewalk", count: 3}

  event:
    total_agents: 70
    zones:
      - {type: "plaza_dense", count: 50}
      - {type: "sidewalk",    count: 20}

  mixed:
    total_agents: 35
    zones:
      - {type: "sidewalk", count: 25}
      - {type: "park",     count: 10}

  industrial:
    total_agents: 10
    zones:
      - {type: "sidewalk", count: 8}
      - {type: "park",     count: 2}

# ── Wind dynamics ────────────────────────────────────────────────────────────
weather:
  wind:
    enabled: true
    base_speed: 0.4           # m/s
    direction: [1, 0.5, 0]   # [x, y, z] — normalised internally
    turbulence: 0.35          # std dev as fraction of base_speed
    altitude_factor: 1.5      # multiplier at 10 m altitude

# ── Bird obstacles ────────────────────────────────────────────────────────────
obstacles:
  birds:
    enabled: true
    count: 8
    altitude_range: [3, 7]   # meters
    use_boids: true           # true = flocking; false = random walk

  trees:
    enabled: true
    count: 12

  poles:
    enabled: true

  houses:
    enabled: false            # replaced by scenario building system

# ── Arena limits ─────────────────────────────────────────────────────────────
arena:
  xy_bound: 16.0             # drone OOB threshold (meters from origin)
  z_min: 0.4                 # minimum flight altitude
  z_max: 18.0                # maximum flight altitude

# ── Visual quality ────────────────────────────────────────────────────────────
visual_quality:
  detail_level: "high"       # low | medium | high

# ── Per-scenario overrides ───────────────────────────────────────────────────
# These are deep-merged with top-level keys when a scenario is loaded.
scenarios:
  downtown:
    terrain:
      city_size: [26.0, 26.0]
      block_size: 6.0
      road_width: 2.0
    buildings:
      height_range: [6.0, 18.0]
      density: "high"
    arena:
      xy_bound: 15.0
  # ... other scenarios
```

---

## Per-Parameter Tuning Guide

### Crowd Density

| Level | Total Agents | Recommended Zone Config | Scenario Use |
|:------|:------------|:------------------------|:-------------|
| Sparse | 5–10 | sidewalk: 5–8, park: 2–3 | Residential, industrial |
| Moderate | 15–20 | sidewalk: 10–15, bus_stop: 3–5 | Downtown |
| Dense | 25–40 | sidewalk: 20–30, park: 5–10 | Mixed |
| Very Dense | 50–80 | plaza_dense: 40–60, sidewalk: 10–20 | Event |
| Maximum | > 80 | Expect < 100 FPS at 5-drone load | Testing only |

The 5-drone FPS floor of 100 Hz is maintained up to approximately 70 crowd agents with sphere collision shapes. Above 80 agents, monitor FPS with `scripts/phase3_showcase.py` before training.

### Zone Types

| Type | Behaviour | Best For |
|:-----|:----------|:---------|
| `sidewalk` | Walk along road-parallel strip, 5% pause chance | Urban pedestrian traffic |
| `bus_stop` | Wait 80–200 steps, 25% chance to relocate | Transit gathering |
| `park` | Slow wander (55% speed) in park block interior | Leisure areas |
| `plaza_dense` | Slow wander in central plaza block interior | Event / market gatherings |

### Wind Strength

| Label | base_speed (m/s) | Effect on Drones |
|:------|:----------------|:----------------|
| Calm | 0–0.5 | Barely noticeable, < 0.1 N force |
| Light | 0.5–1.5 | Gentle drift, easily corrected |
| Moderate | 1.5–3.0 | Noticeable deflection, recommended for training |
| Strong | 3.0–4.5 | Significant control challenge |
| Extreme | 4.5+ | Very difficult; useful for resilience stress tests |

Wind force scales with altitude: at 10 m, force = `base_speed × altitude_factor × drone_mass`. Default `altitude_factor=1.5` means a drone at 10 m experiences 1.5× surface wind.

### Turbulence

```yaml
turbulence: 0.35   # Gaussian std = 0.35 × base_speed per axis per step
```

At `base_speed=2.0` and `turbulence=0.35`, each step samples a random gust with std = 0.7 m/s per axis. Set to 0 for deterministic wind (useful for reproducible experiments).

### Wind Direction Vector

The direction vector is normalised internally; any non-zero vector works.

```yaml
direction: [1, 0, 0]       # Pure east
direction: [0, 1, 0]       # Pure north
direction: [1, 1, 0]       # Northeast (45°)
direction: [1, 0.5, 0]     # ENE (default)
direction: [-1, -0.5, 0]   # WSW (reverse default)
direction: [0, 0, 1]       # Updraft (unusual)
```

### Bird Count

| Count | Effect | FPS Impact |
|:------|:-------|:----------|
| 0 | No bird obstacles | Baseline |
| 5 | Minimal collision risk | Negligible |
| 8 | Standard flock (recommended) | ~1–2 FPS |
| 15 | High avoidance stress | ~3–5 FPS |
| 20+ | Collision gauntlet | Monitor FPS |

### Visual Quality

| Level | Bodies | Detail | FPS Impact | Use Case |
|:------|:-------|:-------|:----------|:---------|
| `low` | Minimal furniture | No window bands or ledges | +15–25% FPS | Headless training |
| `medium` | Standard furniture | Window grid, basic ledges | Baseline | Standard research |
| `high` | Full furniture | Window bands, ledges, rooftop detail | −10–20% FPS | Screenshots, demos |

---

## Performance vs Quality Trade-offs

### Training Configuration (Maximum FPS)

```yaml
scenario: "residential"        # highest FPS scenario
visual_quality:
  detail_level: "low"
obstacles:
  birds:
    enabled: false             # remove bird LiDAR bodies
  trees:
    enabled: false
crowd:
  residential:
    total_agents: 5            # minimal crowd
```

Expected FPS: 200+ (residential, 3 drones, headless).

### Research Demo Configuration (Balanced)

```yaml
scenario: "downtown"
visual_quality:
  detail_level: "medium"
obstacles:
  birds:
    enabled: true
    count: 8
crowd:
  downtown:
    total_agents: 15
weather:
  wind:
    enabled: true
    base_speed: 1.5
```

Expected FPS: 110–130 (downtown, 3 drones, headless).

### Publication Demo Configuration (Maximum Visual Quality)

```yaml
scenario: "event"
visual_quality:
  detail_level: "high"
obstacles:
  birds:
    enabled: true
    count: 8
crowd:
  event:
    total_agents: 70
    zones:
      - {type: "plaza_dense", count: 50}
      - {type: "sidewalk",    count: 20}
weather:
  wind:
    enabled: true
    base_speed: 2.0
```

Expected FPS: 80–100 (event, headless, GUI mode reduces further).

---

## Creating a Custom Scenario

### Step 1 — Create the Scenario File

```python
# envs/scenarios/campus.py
"""
University Campus Scenario
Open quads, lecture buildings, pedestrian paths.
"""
from typing import List, Dict
from .scenario_base import BaseScenario


class CampusScenario(BaseScenario):
    SCENARIO_NAME       = 'campus'
    SCENARIO_DESC       = 'University campus — open quads, mixed-height buildings'
    DEFAULT_ARENA_BOUND = 14.0

    def __init__(self, physics_client, config, seed=42):
        cfg = dict(config)
        cfg.setdefault('terrain', {})
        cfg['terrain'].setdefault('city_size',  [24.0, 24.0])
        cfg['terrain'].setdefault('block_size', 7.0)
        cfg['terrain'].setdefault('road_width', 2.0)
        cfg.setdefault('buildings', {})
        cfg['buildings'].setdefault('height_range', [4.0, 12.0])
        cfg['buildings'].setdefault('density', 'medium')
        super().__init__(physics_client, cfg, seed)

    def _extra_spawn(self) -> List[int]:
        ids = []
        # Central quad fountain
        ids.append(self._vis_cylinder(0, 0, 0.3, 1.2, 0.3,
                                       (0.72, 0.72, 0.76, 1.0)))
        # Flagpole
        ids.append(self._vis_cylinder(4, 0, 3.5, 0.04, 7.0,
                                       (0.60, 0.58, 0.55, 1.0)))
        return ids

    def get_crowd_zones(self) -> List[Dict]:
        return [
            {'center': [0.0,  0.0],  'radius': 5.0, 'density': 'medium'},
            {'center': [5.0,  5.0],  'radius': 3.0, 'density': 'low'},
            {'center': [-5.0, -5.0], 'radius': 3.0, 'density': 'low'},
        ]
```

**Available `_vis_*` helpers (inherited from BaseScenario):**

| Method | Signature | Creates |
|:-------|:----------|:--------|
| `_vis_box` | `(cx, cy, cz, hx, hy, hz, rgba)` | Box visual body |
| `_vis_cylinder` | `(cx, cy, cz, radius, length, rgba)` | Cylinder visual body |
| `_vis_sphere` | `(cx, cy, cz, radius, rgba)` | Sphere visual body |

All helpers create visual-only bodies (`baseCollisionShapeIndex=-1`) — zero LiDAR overhead.

### Step 2 — Register the Scenario

In `envs/scenarios/__init__.py`, add to the import block and `_MAP` dict:

```python
from .campus import CampusScenario   # add this line

_MAP = {
    'downtown':    DowntownScenario,
    'residential': ResidentialScenario,
    'event':       EventScenario,
    'mixed':       MixedUrbanScenario,
    'industrial':  IndustrialScenario,
    'campus':      CampusScenario,     # add this line
}
```

Also add to `available_scenarios()`:

```python
def available_scenarios():
    return ['downtown', 'residential', 'event', 'mixed', 'industrial', 'campus']
```

### Step 3 — Add Crowd Configuration

In `config/environment_config.yaml`, add under the `crowd:` key:

```yaml
crowd:
  # ... existing scenarios ...
  campus:
    total_agents: 25
    zones:
      - {type: "sidewalk",    count: 15}
      - {type: "plaza_dense", count: 10}
```

Also add per-scenario terrain overrides if needed:

```yaml
scenarios:
  # ... existing ...
  campus:
    terrain:
      city_size: [24.0, 24.0]
      block_size: 7.0
      road_width: 2.0
    buildings:
      height_range: [4.0, 12.0]
      density: "medium"
    arena:
      xy_bound: 14.0
```

### Step 4 — Add to GUI Selector (Optional)

In `main_selector.py`, locate the scenario list and add `'campus'` to the list of available options. The GUI reads `available_scenarios()` automatically if it is wired to that function.

### Step 5 — Test

```bash
# Headless validation
python scripts/phase3_showcase.py   # add campus to SCENARIOS_5D list for FPS test

# Or direct:
HEADLESS=1 python main_selector.py --scenario campus --algorithm ppo
```

---

## Programmatic Configuration (no YAML)

Pass config directly to `DroneSurveillanceEnv` via `env_config`:

```python
from envs.drone_env import DroneSurveillanceEnv

env = DroneSurveillanceEnv(
    render_mode="headless",
    fixed_layout=True,
    env_config={
        # Scenario selection
        "use_scenario_system": True,
        "scenario": "event",

        # Phase 3 features
        "enable_birds": True,
        "enable_trees": True,
        "enable_poles": True,
        "enable_houses": False,
        "num_birds": 8,
        "enable_wind_physics": True,
        "wind_base_speed": 2.0,

        # Crowd zones override (bypasses YAML)
        # Pass via CrowdSimulator zones_override if needed
    }
)
obs, info = env.reset()
```

### Crowd Zones Override at Runtime

To override crowd configuration without editing YAML, pass `zones_override` when constructing `CrowdSimulator` directly (advanced use):

```python
from envs.dynamics.crowd_simulator import CrowdSimulator

crowd = CrowdSimulator(
    client_id=env.client_id,
    layout=env.layout,
    scenario="event",
    building_specs=env.building_specs,
    zones_override={
        'zones': [
            {'type': 'plaza_dense', 'count': 80},
            {'type': 'sidewalk',    'count': 30},
        ]
    },
    rng=np.random.default_rng(42)
)
```

---

## Environment Constructor Reference

```python
DroneSurveillanceEnv(
    render_mode: str = "human",     # "human" | "headless"
    fixed_layout: bool = False,     # True → seed 42 for reproducibility
    env_config: dict = {}           # see keys below
)
```

### `env_config` Keys

| Key | Type | Default | Description |
|:----|:-----|:--------|:------------|
| `use_scenario_system` | bool | `False` | Enable Phase 2/3 scenario pipeline |
| `scenario` | str | `"downtown"` | Active scenario name |
| `enable_birds` | bool | `True` | Spawn bird obstacles |
| `num_birds` | int | `8` | Number of birds |
| `enable_trees` | bool | `True` | Spawn tree decorations |
| `enable_poles` | bool | `True` | Spawn utility poles |
| `enable_houses` | bool | `False` | Legacy house objects (replaced by scenario buildings) |
| `num_trees` | int | `12` | Number of trees |
| `enable_wind_physics` | bool | `True` | Apply wind force to drones |
| `wind_base_speed` | float | `0.4` | Wind base speed (m/s) |

### Noise Parameters

```python
noise_params = {
    "flip_prob":    0.04,     # probability of occupancy grid cell flip per step
    "gaussian_std": 0.05,     # Gaussian noise std on LiDAR readings
    "drift_offset": (0, 0),   # SLAM drift (x_shift, y_shift) in grid cells
}
env = DroneSurveillanceEnv(..., noise_params=noise_params)
```

### Observation Space

```python
obs[drone_id] = {
    "position":       Box(3,),        # [x, y, z] world position
    "velocity":       Box(3,),        # [vx, vy, vz] current velocity
    "lidar":          Box(36,),       # 36 horizontal ray distances (m)
    "occupancy_grid": Box(16, 16),    # local SLAM grid (0=free, 1=occupied)
}
```

### Action Space

```python
action[drone_id] = Box(4,)   # [thrust, roll, pitch, yaw_rate]
                              # thrust: 0.0–1.0 (hover ≈ 0.3734)
                              # roll, pitch: −1.0–1.0
                              # yaw_rate: −1.0–1.0
```
