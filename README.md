# 🚁 Smart City Multi-UAV Crowd Surveillance

> A research-grade multi-UAV surveillance simulation for smart city environments.  
> Built with Python + Pygame. Designed for algorithm comparison, paper publication, and future hardware integration.

---

## 📋 Project Overview

This project simulates a fleet of **4 autonomous UAVs** monitoring a simulated smart city populated by **30 crowd agents**. UAVs coordinate to maximise surveillance coverage while avoiding collisions, adapting to GPS noise, wind disturbances, and communication delays.

The architecture is **modular and research-oriented** — algorithms can be swapped without touching agent or rendering code.

---

## 🎯 Research Objectives

- Model and compare multi-UAV coordination strategies in dynamic urban environments
- Evaluate surveillance coverage under realistic sensor uncertainty (GPS noise, comm delay, wind)
- Benchmark path-planning and collision-avoidance algorithms against shared metrics
- Provide a lightweight simulation baseline for UAV coordination research papers

---

## ✨ Features

| Feature | Detail |
|---|---|
| **Realistic UAV physics** | Acceleration, friction, inertia, smooth heading rotation |
| **Quadrotor silhouette** | Arms + rotor discs rendered from heading angle |
| **Crowd hotspots** | 4 drifting attraction centres with pulsing weights |
| **Boid flocking** | Cohesion, alignment, separation + hotspot bias |
| **GPS noise** | Gaussian offset refreshed periodically |
| **Wind drift** | Slowly-varying random force applied to velocity |
| **Comms delay** | Target positions buffered and delivered N frames late |
| **Battery drain** | Speed reduction + RTB status at depletion |
| **Collision avoidance** | Rule-based repulsion (drone-drone + building) |
| **Metrics HUD** | Coverage %, tracking accuracy, monitored %, fleet status |
| **Hotspot breakdown** | Per-hotspot crowd count displayed in real time |

---

## 🏗️ Project Architecture

```
multi-uav-surveillance/
├── main.py                         ← Primary entry point
├── drone_simulation.py             ← Legacy entry point (backward compat)
├── requirements.txt
├── README.md
│
└── simulation/                     ← Core research package
    ├── config.py                   ← All constants & tunable parameters
    ├── runner.py                   ← Game loop, reset, integration
    │
    ├── environment/
    │   └── city_map.py             ← Buildings, roads, zones, geometry utils
    │
    ├── crowd/
    │   ├── hotspot.py              ← Drifting crowd-attraction centre
    │   ├── person.py               ← Boid pedestrian agent
    │   └── crowd_system.py         ← Crowd orchestrator + density metrics
    │
    ├── drone/
    │   └── drone.py                ← UAV agent (physics + pluggable algorithms)
    │
    ├── algorithms/
    │   ├── collision_avoidance.py  ← Rule-based + stubs (VO, potential field)
    │   ├── path_planning.py        ← Coverage offset + stubs (A*, RRT, RL)
    │   └── communication.py        ← Comms delay, GPS noise, wind models
    │
    ├── metrics/
    │   └── analytics.py            ← Coverage grid, tracking accuracy, KPIs
    │
    └── visualization/
        ├── renderer.py             ← Map drawing (roads, buildings, zones)
        └── panel.py                ← HUD panel (metrics, fleet status)
```

### Module Responsibilities

| Module | Responsibility |
|---|---|
| `config.py` | Single source of truth for all constants |
| `city_map.py` | Static city geometry; geometry helpers |
| `hotspot.py` | Dynamic crowd-attraction centre |
| `person.py` | Individual pedestrian boid agent |
| `crowd_system.py` | Crowd orchestration, density & coverage stats |
| `drone.py` | UAV physics, sensor models, pluggable algorithm hooks |
| `collision_avoidance.py` | Pluggable avoidance functions |
| `path_planning.py` | Pluggable target-assignment planners |
| `communication.py` | Comms delay buffer, GPS noise, wind model |
| `analytics.py` | Periodic metric computation (coverage, accuracy) |
| `renderer.py` | Pure map / city drawing |
| `panel.py` | HUD panel drawing |
| `runner.py` | Game loop, clock, reset logic |

---

## 📁 Folder Structure

```
multi-uav-surveillance/
├── main.py
├── drone_simulation.py
├── requirements.txt
├── README.md
├── check_syntax.py             ← Developer utility (safe to delete)
└── simulation/
    ├── __init__.py
    ├── config.py
    ├── runner.py
    ├── environment/
    │   ├── __init__.py
    │   └── city_map.py
    ├── crowd/
    │   ├── __init__.py
    │   ├── hotspot.py
    │   ├── person.py
    │   └── crowd_system.py
    ├── drone/
    │   ├── __init__.py
    │   └── drone.py
    ├── algorithms/
    │   ├── __init__.py
    │   ├── collision_avoidance.py
    │   ├── path_planning.py
    │   └── communication.py
    ├── metrics/
    │   ├── __init__.py
    │   └── analytics.py
    └── visualization/
        ├── __init__.py
        ├── renderer.py
        └── panel.py
```

---

## ⚙️ Installation

```bash
# 1. Clone the repository
git clone https://github.com/your-username/multi-uav-surveillance.git
cd multi-uav-surveillance

# 2. (Optional) Create a virtual environment
python -m venv venv
venv\Scripts\activate        # Windows
source venv/bin/activate     # Linux / macOS

# 3. Install dependencies
pip install -r requirements.txt
```

**Requirements:** Python 3.10+ · pygame 2.1+

---

## ▶️ How to Run

```bash
python main.py
```

The legacy filename still works:
```bash
python drone_simulation.py
```

---

## 🎮 Controls

| Key | Action |
|---|---|
| `SPACE` | Reset simulation |
| `P` | Pause / Resume |
| `ESC` | Quit |

---

## 🔬 Research Modules

### Swapping Path-Planning Algorithms

All planners share the same signature:

```python
def my_planner(drone, delayed_center: tuple) -> tuple[float, float]:
    ...
```

To assign at runtime:

```python
from simulation.algorithms.path_planning import astar_planner
for d in runner.drones:
    d.planner_fn = astar_planner
```

**Available planners:**

| Function | Status |
|---|---|
| `coverage_offset_planner` | ✅ Active |
| `astar_planner` | 🔧 Stub |
| `dijkstra_planner` | 🔧 Stub |
| `rrt_planner` | 🔧 Stub |
| `potential_field_planner` | 🔧 Stub |
| `qlearning_planner` | 🔧 Stub |

### Swapping Collision Avoidance Algorithms

All avoidance functions share the same signature:

```python
def my_avoidance(drone, all_drones, col_ref) -> tuple[float, float, bool]:
    # returns (dvx, dvy, avoiding)
```

**Available algorithms:**

| Function | Status |
|---|---|
| `rule_based_avoidance` | ✅ Active |
| `potential_field_avoidance` | 🔧 Stub |
| `velocity_obstacle_avoidance` | 🔧 Stub |

---

## 👥 Git Collaboration Workflow

The project is split so that 3–4 teammates can work in parallel with **minimal merge conflicts**:

| Teammate | Files to own |
|---|---|
| **Crowd behaviour** | `simulation/crowd/` |
| **Drone coordination / planners** | `simulation/drone/`, `simulation/algorithms/` |
| **Metrics & analytics** | `simulation/metrics/`, `simulation/config.py` |
| **UI / Visualization** | `simulation/visualization/` |

### Suggested branching strategy

```
main            ← stable, runnable
├── dev         ← integration branch
│   ├── feature/astar-planner
│   ├── feature/social-force-crowd
│   ├── feature/coverage-metrics-v2
│   └── feature/dark-hud-redesign
```

**Rules:**
1. Never commit directly to `main`.
2. Open a PR into `dev`; review with one teammate.
3. Merge `dev` → `main` only when the simulation runs cleanly.

---

## 🗺️ Future Roadmap

- [ ] A* / Dijkstra grid path-planning implementation
- [ ] RRT planner for dynamic obstacle environments
- [ ] Q-learning policy for adaptive surveillance
- [ ] Velocity Obstacle (VO / RVO) collision avoidance
- [ ] Social force crowd model
- [ ] Leader-follower UAV formation control
- [ ] Multi-mission zone prioritisation
- [ ] CSV / JSON metric logging for paper plots
- [ ] Configurable experiment profiles (YAML / JSON)
- [ ] Hardware-in-the-loop adapter (MAVLink / ROS bridge)
- [ ] Web dashboard for remote monitoring

---

## 📸 Screenshots

> _Add screenshots or screen recordings here._

| Simulation View | Metrics Panel |
|---|---|
| _(screenshot placeholder)_ | _(screenshot placeholder)_ |

---

## 📄 Citation

If you use this simulation in research, please cite:

```
@misc{multi-uav-surveillance,
  title  = {Smart City Multi-UAV Crowd Surveillance Simulation},
  year   = {2025},
  url    = {https://github.com/your-username/multi-uav-surveillance}
}
```

---

## 📜 License

MIT — free to use, modify, and distribute for research and education.