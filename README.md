# Decentralized Multi-UAV Surveillance Swarm — STIRS-2025

> [!NOTE]
> **Research Context:** Aligning with Advanced Multi-Agent DRL and Graph Attention (ADGAT) research standards for the STIRS-2025 framework.

---

## 🧭 Quick Start for Team Members

New to the team? Follow these quick reference guides to get started in under 10 minutes:

*   📖 **[TEAM_ROADMAP.md](file:///d:/IFSP/multi-uav-surveillance/TEAM_ROADMAP.md)**: Project vision, migration history, and the 6 research phases.
*   🤝 **[TEAM_WORKFLOW.md](file:///d:/IFSP/multi-uav-surveillance/TEAM_WORKFLOW.md)**: Git branching policy, 5-step commit workflow, and merge process.
*   🚀 **[RUN_GUIDE.md](file:///d:/IFSP/multi-uav-surveillance/RUN_GUIDE.md)**: Running simulation locally (GUI), in Docker (headless), or starting training.
*   📝 **[UPDATE_RULES.md](file:///d:/IFSP/multi-uav-surveillance/UPDATE_RULES.md)**: Mandatory files to update (`CURRENT_STATUS.md`, `CHANGELOG.md`, `TASKS.md`) when pushing changes.

---

## 🚁 Project Overview

A high-fidelity SITL (Software-in-the-Loop) proof-of-concept where a swarm of **three autonomous quadcopters** navigates a procedurally generated urban concrete canyon to search for and track moving ground targets.

UAV decision-making operates under a **Partially Observable Markov Decision Process (POMDP)** framework. Each drone uses local horizontal raycasting sensors to generate an imperfect **16×16 Local Occupancy Grid** subject to simulated SLAM mapping drift and sensor interference.

---

## 🛠️ Technology Stack

| Component | Technology | Role |
| :--- | :--- | :--- |
| **Physics Engine** | **PyBullet** | 3D rigid-body aerodynamics, collision, batch raycasting |
| **Multi-Agent API** | **PettingZoo (ParallelEnv)** | Multi-agent action/step/observation mapping |
| **Deep RL Framework** | **Ray RLlib** | MADDPG/PPO distributed training *(Python ≤ 3.12 only)* |
| **Deep Learning** | **PyTorch** | ARReSVG + LSTM policy network (CPU-only enforced) |
| **Core Environment** | **Gymnasium** | Base class, spaces, registration utilities |
| **Vector Math** | **NumPy** | Grid transforms, occupancy grid math |
| **Visual Rendering** | **OpenGL (PyBullet GUI)** | Real-time 3D lighting, shadows, holographic HUD text |

---

## 📂 Repository Structure

```
multi-uav-surveillance/
├── main_selector.py            # PRIMARY: GUI selector + --scenario / --algorithm CLI flags
├── envs/
│   ├── drone_env.py            # Gymnasium Multi-Agent Environment (PettingZoo ParallelEnv)
│   ├── environment_assets.py   # Legacy asset spawners (trees, birds, wind, poles)
│   ├── terrain/
│   │   ├── city_layout.py      # Grid-based city layout generator (3×3 block grid)
│   │   └── terrain_generator.py# Road surfaces, sidewalks, park patches, road markings
│   ├── structures/
│   │   ├── buildings.py        # EnhancedBuildingSpawner (5 types, realistic palettes)
│   │   └── street_furniture.py # Phase 2: street lights, traffic signals, cars, benches…
│   └── scenarios/
│       ├── __init__.py         # load_scenario() factory with YAML config merge
│       ├── scenario_base.py    # BaseScenario: layout → terrain → buildings → furniture
│       ├── downtown.py         # Dense commercial, arena 15 m, plaza + fountain extras
│       ├── residential.py      # Low-rise, parks, arena 14 m, playground extras
│       ├── event.py            # Large plaza, arena 15 m, stage + tent extras
│       ├── mixed.py            # Varied heights, arena 14 m, street market extras
│       └── industrial.py       # Warehouses, arena 15 m, containers + crane extras
├── config/
│   └── environment_config.yaml # Per-scenario YAML overrides (terrain, buildings, arena)
├── algorithms/
│   └── obstacle_avoidance/     # PPO baseline, SDDPG-NAV, Attention Distillation
├── scripts/
│   └── capture_screenshots.py  # Headless screenshot + FPS benchmark for all scenarios
├── screenshots/                # Auto-generated PNG renders (6 images)
├── train.py                    # Ray RLlib training scaffold + ARReSVG policy
├── requirements.txt            # Dependency list
└── README.md                   # This file
```

---

## ⚙️ Setup Instructions

### Prerequisites
- **Python 3.13** (for simulation only)
- **Python 3.11 or 3.12** (required for Ray RLlib training — see Known Issues)
- Git, Windows PowerShell

### Step 1 — Clone & Navigate

```powershell
git clone <repo-url>
cd multi-uav-surveillance
```

### Step 2 — Activate the Virtual Environment

```powershell
# Activate (Windows PowerShell)
.\.venv\Scripts\Activate.ps1

# Verify you're in the .venv
python --version    # Should show Python 3.13.x
```

> [!TIP]
> If you see a script execution policy error, run:
> `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

### Step 3 — Install Dependencies

```powershell
# With .venv activated:
pip install numpy pybullet gymnasium torch pettingzoo
```

Or use the requirements file (simulation-only packages, no Ray):

```powershell
pip install -r requirements.txt
```

> [!WARNING]
> **Ray RLlib requires Python ≤ 3.12.** It does NOT install on Python 3.13.
> See the [Training Setup](#training-setup-ray-rllib) section below.

---

## 🚀 Getting Started

We recommend using **Docker** to run this project seamlessly, avoiding Python 3.13 dependency issues with PyBullet and Ray.

### Method 1: Using Docker (Recommended)

1. Ensure [Docker Desktop](https://www.docker.com/products/docker-desktop) is installed and running.
2. Open your terminal in the project directory.
3. Build and start the training process:
   ```bash
   docker-compose up --build training
   ```
4. To run the simulation (headless validation):
   ```bash
   docker-compose up --build simulation
   ```

### Method 2: Local Setup (Python 3.10)

If you prefer to run it locally, follow the instructions in the Troubleshooting section below ("Fix B") to use a Python 3.10 virtual environment.

---

## 🌆 Urban Scenario System (Phase 1 + 2)

The simulation includes a professional-grade procedural urban environment with 5 research-ready city scenarios.

### Scenarios

| Scenario | Description | Arena |
|:---------|:------------|:------|
| `downtown` | Dense commercial high-rises, glass towers, central plaza | 15 m |
| `residential` | Low-rise brick housing, parks, playgrounds | 14 m |
| `event` | Large open plaza with stage, tent canopies, dense crowd | 15 m |
| `mixed` | Varied building heights, street markets | 14 m |
| `industrial` | Warehouses, stacked shipping containers, crane | 15 m |

### Phase 2 Street Furniture (all visual-only, zero physics overhead)

Every scenario automatically receives:
- **Street lights** — 7.5 m LED poles with warm amber glow, both road sides
- **Traffic signals** — 3-light signals (red active) at all 9 grid intersections
- **Utility poles** — Brown cross-arm poles with overhead power lines along roads
- **Parked cars** — Realistic palette (white/silver/black/blue/red) in lots and roadside
- **Benches** — Wood + metal benches in every park and plaza block
- **Bus stops** — Glass-panel shelters at 3 road-edge locations
- **Trash bins** — Dark green cylindrical bins near benches
- **Varied sidewalk trees** — Randomised trunk height (1.8–4.2 m), 5 green shades

### Performance (headless, 3 drones)

| Scenario | FPS | Status |
|:---------|----:|:-------|
| downtown | 101.5 | PASS ≥ 100 |
| residential | 124.7 | PASS ≥ 100 |
| event | 141.2 | PASS ≥ 100 |
| mixed | 121.1 | PASS ≥ 100 |
| industrial | 125.3 | PASS ≥ 100 |
| downtown (5-drone est.) | 95.2 | PASS ≥ 20 |

---

## 🚀 Running the Simulation

### Option A — Interactive GUI Selector (Recommended)

Launches the dark-themed Tkinter control panel. Choose a scenario and algorithm, then click **Launch Simulation**:

```powershell
.\.venv\Scripts\python.exe main_selector.py
```

### Option B — Direct CLI Launch

Skip the GUI and launch a specific scenario + algorithm directly:

```powershell
# Downtown with default algorithm (PPO)
.\.venv\Scripts\python.exe main_selector.py --scenario downtown

# Event scenario with SDDPG-NAV
.\.venv\Scripts\python.exe main_selector.py --scenario event --algorithm ddpg

# Residential with Attention Distillation
.\.venv\Scripts\python.exe main_selector.py --scenario residential --algorithm distill
```

Available `--scenario` values: `downtown`, `residential`, `event`, `mixed`, `industrial`
Available `--algorithm` values: `ppo`, `ddpg`, `distill`

### Option C — Headless Environment Validation (No Window)

Test physics, sub-stepping, occupancy grids, and observations without a GUI:

```powershell
.\.venv\Scripts\python.exe envs/drone_env.py
```

### Option D — Screenshot + FPS Benchmark

Capture high-quality renders of all 5 scenarios and benchmark step rate:

```powershell
.\.venv\Scripts\python.exe scripts/capture_screenshots.py
```

Output saved to `screenshots/` (6 PNG files).

---

## 🚂 Training Setup (Ray RLlib)

> [!IMPORTANT]
> Ray RLlib **does not support Python 3.13** as of May 2026.
> You must use Python 3.11 or 3.12 for training.

### Step 1 — Install Python 3.12

Download from: https://www.python.org/downloads/release/python-3120/

### Step 2 — Create a second venv for training

```powershell
# Create a Python 3.12 venv for training (keep venv/ for simulation)
py -3.12 -m venv venv312
.\venv312\Scripts\Activate.ps1
pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]
```

### Step 3 — Run training validation

```powershell
.\venv312\Scripts\python.exe train.py
```

This validates the ARReSVG policy architecture and ADGAT attention routing — no actual training loop yet.

---

## 🔑 Key Environment Parameters

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `DEMO_MODE` | `True` | `True` = GUI with orbital camera; `False` = headless DIRECT |
| `render_mode` | `"human"` | Passed to `DroneSurveillanceEnv()` |
| `fixed_layout` | `False` | `True` = seed 42 for reproducible building/crowd layout |
| `noise_params` | See below | SLAM noise configuration |

**Default noise params:**
```python
noise = {
    "flip_prob": 0.04,       # Probability of cell flip per step
    "gaussian_std": 0.05,    # Std dev of Gaussian sensor noise
    "drift_offset": (0, 0)   # (x_shift, y_shift) SLAM drift
}
```

---

## 📈 STIRS-2025 Target Benchmarks

| Metric | Definition | Target |
| :--- | :--- | :--- |
| **SSR** — Swarm Success Rate | Collision-free mission ratio | ≥ 85% |
| **DA** — Dynamic Adaptability | POMDP belief state recalculation time | ≤ 0.12 s |
| **PO** — Path Optimality | Actual / shortest path ratio | ≤ 1.1 |
| **MDM** — Min Distance Margin | UAV–building separation maintained | 1.0–5.0 m |
| **Scalability Limit** | Computation time at 5 drones | CT ≤ 0.5 s |
| **TRR Resilience** | Target Recognition Rate under SLAM noise | Minimal decay |

---

## 🐞 Troubleshooting

### "ERROR: Failed building wheel for pybullet" (MOST COMMON)
**Root cause:** pybullet has no pre-built wheel for Python 3.13. It must compile from C++ source.

**Fix A — Install VS C++ Build Tools (stay on Python 3.13):**
```powershell
# 1. Download and install VS Build Tools with "Desktop development with C++"
#    https://visualstudio.microsoft.com/visual-cpp-build-tools/
# 2. After install, restart PowerShell and re-run:
.\.venv\Scripts\python.exe -m pip install pybullet
```

**Fix B — Install Python 3.10 (RECOMMENDED — also fixes Ray/rllib):**
```powershell
# Python 3.10 has pre-built wheels for pybullet AND supports ray[rllib]
# Download Python 3.10: https://www.python.org/downloads/release/python-31011/
py -3.10 -m venv venv310
.\venv310\Scripts\Activate.ps1
pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]
python main_selector.py    # launcher GUI
python train.py            # full training
```

### "No matching distribution found for ray"
Ray does not support Python 3.13. Use Python 3.10/3.11 venv (see Fix B above).

### "pybullet_data: No matching distribution found"
`pybullet_data` is **not** a separate PyPI package — it ships inside `pybullet`. Just `pip install pybullet`.

### PyBullet GUI window doesn't open
Make sure `DEMO_MODE = os.environ.get("HEADLESS", "0") != "1"` and `HEADLESS` environment variable is not set to `1` in `envs/drone_env.py`. Also verify your display drivers support OpenGL.

### Script execution policy error (PowerShell)
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### "ImportError: cannot import name 'DroneEnv' from drone_env"
`train.py` tries to import `DroneEnv` but `envs/drone_env.py` exports `DroneSurveillanceEnv`. This is expected — `train.py` uses its built-in mock `DroneEnv` fallback. The simulation runs correctly via `envs/drone_env.py` or `main_selector.py` directly.

### Drones crash immediately / fall to the floor
Hover thrust is calibrated at `u_thrust ≈ 0.3734`. The demo loop uses thrust ≈ `0.38` with a small random walk. This is intentional — drones hover stably with slight drift.

### Run the diagnostics script first
```powershell
.\.venv\Scripts\python.exe setup_env.py
```
This tells you exactly what's installed, what's missing, and how to fix it.

---

## 💡 Technical Notes

- **GPU Offloading**: All PyTorch tensors are pinned to CPU. No CUDA required — safe for 16GB RAM laptops.
- **Observation Space Keys**: `position`, `velocity`, `lidar`, `occupancy_grid` — compatible with Ray and SB3.
- **Physics Timestep**: PyBullet default (`1/240 s`). Demo loop adds `time.sleep(0.02)` for ~50 Hz visual playback.
- **Battery hover life**: At `u_thrust = 0.3734`, drain ≈ `0.01 + 0.03 × 0.3734 ≈ 0.021%/step` → ~500 steps at full thrust.

---

## 📚 See Also

- [`CURRENT_STATUS.md`](CURRENT_STATUS.md) — What's working, what's blocked, known issues
- [`CHANGELOG.md`](CHANGELOG.md) — What changed, when, and why
- [`TASKS.md`](TASKS.md) — Completed, pending, and research ideas
- [`walkthrough.md`](walkthrough.md) — Deep-dive technical implementation details
- [`project.md`](project.md) — Project outline and research requirements
- [`metrics.md`](metrics.md) — STIRS-2025 evaluation benchmarks
