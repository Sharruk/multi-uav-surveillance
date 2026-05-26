# STIRS-2025 Simulation Environment — Reference Manual

> **Research Context:** Multi-UAV Surveillance Simulation for crowd monitoring, formation control, and obstacle avoidance research. Part of the STIRS-2025 framework.

---

## Project Overview

A high-fidelity, physics-based urban simulation built on PyBullet for training and evaluating decentralized multi-UAV surveillance algorithms. The environment models realistic city scenarios with procedurally generated buildings, moving pedestrian crowds, flocking bird obstacles, and configurable wind dynamics.

### Key Features

| Feature | Details |
|:--------|:--------|
| **5 research scenarios** | Downtown, Event, Residential, Mixed, Industrial |
| **Pedestrian crowd** | Sidewalk-aware agents with state machine (WALK / WAIT / GATHER) |
| **Boids bird obstacles** | Flocking flock at 3–7 m altitude, collidable with LiDAR |
| **Wind dynamics** | Configurable direction, turbulence, building-shadow effects |
| **Sensor model** | 36-ray LiDAR + 16×16 occupancy grid with SLAM drift noise |
| **Multi-agent API** | PettingZoo ParallelEnv (drop-in for PPO, SDDPG, Attention) |
| **Performance** | > 100 FPS at 5-drone load across all scenarios (headless) |

### Research Applications

- Decentralized formation control under partial observability
- Crowd density estimation and pedestrian tracking
- Obstacle avoidance in cluttered urban airspace
- Sensor fusion and SLAM robustness testing
- Algorithm comparison under standardised scenario conditions

---

## Quick Start

### Prerequisites

- Python 3.10–3.13 (3.10 recommended — has pre-built PyBullet wheels and supports Ray RLlib)
- Windows / Linux / macOS

### Installation

```bash
git clone https://github.com/Sharruk/multi-uav-surveillance.git
cd multi-uav-surveillance

# Create and activate venv
python -m venv venv
# Windows:
venv\Scripts\activate
# Linux/macOS:
source venv/bin/activate

pip install -r requirements.txt
```

### Run GUI Selector (Recommended)

```bash
python main_selector.py
```

Launches a dark-themed control panel — choose scenario and algorithm, then click **Launch Simulation**.

### Run Specific Scenario via CLI

```bash
# Downtown scenario with PPO
python main_selector.py --scenario downtown --algorithm ppo

# Event scenario with SDDPG-NAV
python main_selector.py --scenario event --algorithm ddpg

# Residential with Attention Distillation
python main_selector.py --scenario residential --algorithm distill
```

### Headless Mode (Training / CI)

```bash
# Set HEADLESS=1 to run without a display window
HEADLESS=1 python main_selector.py --scenario downtown --algorithm ppo

# Or export once:
export HEADLESS=1
python main_selector.py --scenario event --algorithm distill
```

### Benchmark & Screenshots

```bash
# Phase 3 showcase: screenshots + 5-drone FPS benchmark + algorithm smoke tests
python scripts/phase3_showcase.py

# Legacy per-scenario screenshots + FPS
python scripts/capture_screenshots.py
```

---

## Scenarios Guide

### 1. Downtown Surveillance

**Description:** Dense commercial urban core with high-rise glass towers, street canyons, and a central fountain plaza. The tightest navigation environment with the highest building density.

| Property | Value |
|:---------|:------|
| City size | 26 × 26 m |
| Block size | 6 m |
| Road width | 2 m |
| Building height range | 6–18 m |
| Building density | High |
| Crowd agents | **15** (10 sidewalk walkers + 5 bus-stop gatherers) |
| Birds | 8 (boids flock, 3–7 m altitude) |
| Wind | Configurable (default 0.4 m/s NE) |
| Arena XY bound | 15 m |
| Single-drone FPS | **126.4** |
| 5-drone FPS | **108.9** |

**Landmark extras:** Raised stone plaza, fountain, four corner planters.

**Best for:**
- Occlusion testing (tall buildings block LiDAR line-of-sight)
- Navigation in confined urban canyons
- Bus-stop crowd gathering behaviour
- Communication interference simulation (buildings block inter-drone links)

---

### 2. Event Crowd Control

**Description:** Large open plaza surrounded by mid-rise commercial buildings. A dense crowd gathers for a public event around a central stage. Highest pedestrian density of all scenarios.

| Property | Value |
|:---------|:------|
| City size | 26 × 26 m |
| Block size | 7 m |
| Road width | 2 m |
| Building height range | 4–12 m |
| Building density | Medium |
| Crowd agents | **70** (50 in central plaza + 20 sidewalk) |
| Birds | 8 (boids flock, 3–7 m altitude) |
| Wind | Configurable (default 0.4 m/s NE) |
| Arena XY bound | 15 m |
| Single-drone FPS | **132.7** |
| 5-drone FPS | **118.1** |

**Landmark extras:** Stage platform with backdrop and spotlights, three event tent canopies with poles, crowd barrier fencing.

**Best for:**
- Crowd density estimation algorithms
- Multi-target tracking under occlusion
- Formation control over a dense gathering
- Stress-testing LiDAR with maximum collidable body count

---

### 3. Residential Monitoring

**Description:** Low-rise neighbourhood with brick houses, parks, and playgrounds. Sparse crowd. Widest open sightlines and highest FPS of all scenarios.

| Property | Value |
|:---------|:------|
| City size | 24 × 24 m |
| Block size | 6 m |
| Road width | 2 m |
| Building height range | 3–9 m |
| Building density | Medium |
| Crowd agents | **8** (5 park visitors + 3 sidewalk) |
| Birds | 8 (boids flock, 3–7 m altitude) |
| Wind | Configurable (default 0.4 m/s NE) |
| Arena XY bound | 14 m |
| Single-drone FPS | **193.8** |
| 5-drone FPS | **134.7** |

**Landmark extras:** Playground equipment (swing set, sandbox), garden hedges, picnic benches.

**Best for:**
- Coverage optimisation in sparse environments
- Baseline algorithm benchmarking (low occlusion)
- Park-gathering pedestrian behaviour study
- Training with fast simulation (highest FPS)

---

### 4. Mixed Urban Area

**Description:** Varied commercial and residential mix with medium density. Street market stalls add distinctive clutter. Balanced between occlusion challenge and open movement.

| Property | Value |
|:---------|:------|
| City size | 24 × 24 m |
| Block size | 6 m |
| Road width | 2 m |
| Building height range | 3–14 m |
| Building density | Medium |
| Crowd agents | **35** (25 sidewalk + 10 park gatherers) |
| Birds | 8 (boids flock, 3–7 m altitude) |
| Wind | Configurable (default 0.4 m/s NE) |
| Arena XY bound | 14 m |
| Single-drone FPS | **153.8** |
| 5-drone FPS | **117.2** |

**Landmark extras:** Street market stalls, varied commercial/residential building palettes.

**Best for:**
- General-purpose algorithm development and testing
- Diverse condition robustness evaluation
- Heterogeneous crowd distribution (sidewalk + park)

---

### 5. Industrial Zone

**Description:** Port/warehouse district with wide road lanes, stacked shipping containers, a crane structure, and sparse workers. Unique obstacle profile compared to urban scenarios.

| Property | Value |
|:---------|:------|
| City size | 26 × 26 m |
| Block size | 8 m |
| Road width | 3 m |
| Building height range | 4–10 m |
| Building density | Low |
| Crowd agents | **10** (8 sidewalk workers + 2 park/yard) |
| Birds | 8 (boids flock, 3–7 m altitude) |
| Wind | Configurable (default 0.4 m/s NE) |
| Arena XY bound | 15 m |
| Single-drone FPS | **179.5** |
| 5-drone FPS | **133.5** |

**Landmark extras:** Stacked shipping containers (multi-height), crane arm structure, fuel tanks, perimeter fencing.

**Best for:**
- Long-range target tracking in open terrain
- Unique container obstacle avoidance patterns
- Worker surveillance in sparse industrial zones

---

## Configuration Guide

Edit `config/environment_config.yaml` to change scenario, crowd density, wind, and visual quality.

```yaml
# ── Active scenario ─────────────────────────────────────────────────────────
scenario: "downtown"   # downtown | residential | event | mixed | industrial
use_scenario_system: true

# ── Terrain ──────────────────────────────────────────────────────────────────
terrain:
  city_size: [26.0, 26.0]   # meters
  block_size: 6.0
  road_width: 2.0

# ── Crowd simulation ─────────────────────────────────────────────────────────
crowd:
  enabled: true
  downtown:
    total_agents: 15
    zones:
      - {type: "sidewalk",  count: 10}
      - {type: "bus_stop",  count: 5}
  event:
    total_agents: 70
    zones:
      - {type: "plaza_dense", count: 50}
      - {type: "sidewalk",    count: 20}
  # residential, mixed, industrial follow the same pattern

# ── Wind dynamics ────────────────────────────────────────────────────────────
weather:
  wind:
    enabled: true
    base_speed: 0.4           # m/s — calm:0.4, noticeable:1.5, strong:3.0
    direction: [1, 0.5, 0]   # [x, y, z] unit vector (normalised internally)
    turbulence: 0.35          # fraction of base_speed used as std dev
    altitude_factor: 1.5      # wind at 10 m = 1.5× base_speed

# ── Bird obstacles ────────────────────────────────────────────────────────────
obstacles:
  birds:
    enabled: true
    count: 8
    altitude_range: [3, 7]   # meters — within UAV operating zone
    use_boids: true           # true = flocking; false = legacy random walk

# ── Visual quality ────────────────────────────────────────────────────────────
visual_quality:
  detail_level: "high"   # low | medium | high
```

### Zone Types

| Zone Type | Behaviour | Typical Use |
|:----------|:----------|:------------|
| `sidewalk` | Walk along road-parallel strips, occasional pause | Pedestrian traffic |
| `bus_stop` | Wait at road-edge positions (75% wait, 25% relocate) | Bus stop queuing |
| `park` | Slow wander within park block interior | Leisure/recreation |
| `plaza_dense` | Slow wander in central plaza block | Event crowd gathering |

---

## Algorithm Integration

All three RL algorithms are compatible with the Phase 3 environment. The `crowd_sim.boid_positions` property exposes crowd agent positions for reward computation.

### Compatible Algorithms

| Algorithm | CLI Flag | Key Strength |
|:----------|:---------|:-------------|
| Multi-Agent PPO | `--algorithm ppo` | Stable baseline, shared policy |
| SDDPG-NAV | `--algorithm ddpg` | State-decomposition, reactive navigation |
| Attention Distillation | `--algorithm distill` | Graph attention, formation awareness |

### Observation Space Keys

Every drone receives a dict observation:

```python
obs[drone_id] = {
    "position":      np.array([x, y, z]),         # drone world position
    "velocity":      np.array([vx, vy, vz]),       # current velocity
    "lidar":         np.array([...]),              # 36 ray distances (m)
    "occupancy_grid": np.array([16, 16]),          # local SLAM grid
}
```

### Reward Signal

The step reward includes:
- **Tracking reward** — proximity bonus to crowd agent positions (`boid_positions`)
- **Formation reward** — inter-drone distance penalty
- **Collision penalty** — negative reward on obstacle contact
- **Boundary penalty** — negative reward outside arena XY bound

---

## Performance Benchmarks

Measured headless (DIRECT mode, `fixed_layout=True`). 5-drone load uses 3 full `step()` calls + 2 extra `_get_drone_sensors()` per step.

| Scenario | Crowd | Birds | 3-Drone FPS | 5-Drone FPS | Target |
|:---------|------:|------:|------------:|------------:|:-------|
| Downtown | 15 | 8 | 126.4 | 108.9 | ≥ 100 ✓ |
| Event | 70 | 8 | 132.7 | 118.1 | ≥ 100 ✓ |
| Residential | 8 | 8 | 193.8 | 134.7 | ≥ 100 ✓ |
| Mixed | 35 | 8 | 153.8 | 117.2 | ≥ 100 ✓ |
| Industrial | 10 | 8 | 179.5 | 133.5 | ≥ 100 ✓ |

All scenarios exceed the **100 FPS** real-time multi-UAV operation target.

**Optimization notes:**
- Crowd collision shapes use `GEOM_SPHERE` (analytically fast LiDAR intersection) instead of `GEOM_CYLINDER`
- Street furniture is visual-only (`baseCollisionShapeIndex=-1`) — zero LiDAR overhead
- Shared PyBullet collision and visual shape templates (one shape per crowd type, reused across all agents)

---

## Troubleshooting

### Low FPS (< 100)

- Reduce event crowd count: set `plaza_dense` count to 30–40 in `config/environment_config.yaml`
- Disable birds: `obstacles.birds.enabled: false`
- Use `detail_level: "low"` (fewer visual-only bodies)
- Run headless: set `HEADLESS=1`

### Crowd agents not appearing

- Check `crowd.enabled: true` in config
- Ensure `use_scenario_system: true` is set
- Verify the scenario name matches one of: `downtown`, `residential`, `event`, `mixed`, `industrial`

### Drones not affected by wind

- Set `weather.wind.enabled: true` in config
- Wind is applied via `p.applyExternalForce()` each step; verify `enable_wind_physics: true` is passed in `env_config`

### PyBullet GUI window does not open

- Unset `HEADLESS` environment variable
- Ensure OpenGL drivers are installed
- On Linux: install `libgl1-mesa-glx` and `xvfb` if running on a server

### "ERROR: Failed building wheel for pybullet"

Python 3.13 has no pre-built PyBullet wheel. Options:
1. Install VS C++ Build Tools (Windows) and rerun `pip install pybullet`
2. Use Python 3.10 (recommended — has pre-built wheels and supports Ray RLlib)

```bash
py -3.10 -m venv venv310
venv310\Scripts\activate
pip install -r requirements.txt
```

### Ray RLlib import errors

Ray does not support Python 3.13. Use Python 3.10–3.12 for training.

---

## Reproducibility

Set `fixed_layout=True` when constructing `DroneSurveillanceEnv` to fix the random seed (42) for building placement, crowd agent starting positions, and bird spawn locations. All dynamic elements use `np.random.default_rng(seed)` internally.

```python
env = DroneSurveillanceEnv(
    render_mode="headless",
    fixed_layout=True,          # deterministic layout
    env_config={
        "use_scenario_system": True,
        "scenario": "downtown",
    }
)
```

---

## Citation

If you use this simulation environment in your research, please cite:

```
[Paper citation to be added upon publication]
```

---

## Contact

Project: STIRS-2025 Multi-UAV Surveillance Simulation
Repository: https://github.com/Sharruk/multi-uav-surveillance
