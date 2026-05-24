# Decentralized Multi-UAV Surveillance Swarm — STIRS-2025

> [!NOTE]
> **Research Context:** Aligning with Advanced Multi-Agent DRL and Graph Attention (ADGAT) research standards for the STIRS-2025 framework.

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
├── venv/                   # Virtual Python environment (Python 3.13, not committed)
├── drone_env.py            # ✅ PRIMARY: PyBullet 3D simulation environment
├── train.py                # Ray RLlib training scaffold + ARReSVG policy
├── requirements.txt        # Accurate dependency list (auto-generated from imports)
├── README.md               # Setup & usage guide (this file)
├── CURRENT_STATUS.md       # Team-readable project state tracker
├── CHANGELOG.md            # Update history (what changed, why, next steps)
├── TASKS.md                # Task tracker (completed / pending / research ideas)
├── project.md              # Project outline & research requirements
├── status.md               # Phase-by-phase completion log (legacy)
├── task.md                 # Phase 3 task checklist (legacy)
├── walkthrough.md          # Technical implementation details
└── metrics.md              # STIRS-2025 target benchmarks
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
.\venv\Scripts\Activate.ps1

# Verify you're in the venv
python --version    # Should show Python 3.13.x
```

> [!TIP]
> If you see a script execution policy error, run:
> `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`

### Step 3 — Install Dependencies

```powershell
# With venv activated:
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

## 🚀 Running the Simulation

### Option A — Interactive GUI Demo (Recommended)

Launches the PyBullet 3D GUI with the cinematic orbital camera, holographic HUD labels, and live terminal dashboard:

```powershell
.\venv\Scripts\python.exe drone_env.py
```

*Press `Ctrl+C` in your terminal to safely stop the demo loop.*

> `DEMO_MODE = True` is set at the top of `drone_env.py` by default — this enables GUI mode.

### Option B — Headless Validation (No Window)

Test the raw physics, occupancy grids, and observations without a GUI window:

1. Open `drone_env.py` and change line 11:
   ```python
   DEMO_MODE = False   # was True
   ```
2. Run:
   ```powershell
   .\venv\Scripts\python.exe drone_env.py
   ```

This runs a **10-step trial** and prints the SLAM occupancy grid as ASCII art in the console.

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
.\venv\Scripts\python.exe -m pip install pybullet
```

**Fix B — Install Python 3.10 (RECOMMENDED — also fixes Ray/rllib):**
```powershell
# Python 3.10 has pre-built wheels for pybullet AND supports ray[rllib]
# Download Python 3.10: https://www.python.org/downloads/release/python-31011/
py -3.10 -m venv venv310
.\venv310\Scripts\Activate.ps1
pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]
python drone_env.py    # simulation
python train.py        # full training
```

### "No matching distribution found for ray"
Ray does not support Python 3.13. Use Python 3.10/3.11 venv (see Fix B above).

### "pybullet_data: No matching distribution found"
`pybullet_data` is **not** a separate PyPI package — it ships inside `pybullet`. Just `pip install pybullet`.

### PyBullet GUI window doesn't open
Make sure `DEMO_MODE = True` at line 11 of `drone_env.py`. Also verify your display drivers support OpenGL.

### Script execution policy error (PowerShell)
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### "ImportError: cannot import name 'DroneEnv' from drone_env"
`train.py` tries to import `DroneEnv` but `drone_env.py` exports `DroneSurveillanceEnv`. This is expected — `train.py` uses its built-in mock `DroneEnv` fallback. The simulation runs correctly via `drone_env.py` directly.

### Drones crash immediately / fall to the floor
Hover thrust is calibrated at `u_thrust ≈ 0.3734`. The demo loop uses thrust ≈ `0.38` with a small random walk. This is intentional — drones hover stably with slight drift.

### Run the diagnostics script first
```powershell
.\venv\Scripts\python.exe setup_env.py
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
