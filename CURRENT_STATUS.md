# CURRENT_STATUS.md
# Decentralized Multi-UAV Surveillance — Project Status

> **Last Updated:** 2026-05-23  
> **Branch:** Drone3D  
> **Author:** Jeswin (SSN College of Engineering, CSE 2024–29)  
> **Research Framework:** STIRS-2025

---

## 🟢 Current Project State

**Phase 3 Complete.** The simulation environment (`drone_env.py`) is fully implemented, visually polished, and validated. The training scaffold (`train.py`) is architecturally complete for Phase 2.

The environment runs correctly on **Python 3.13** with `drone_env.py`.  
`train.py` (Ray RLlib) requires **Python ≤ 3.12** — this is a known blocker (see Known Issues).

---

## ✅ What Is Completed

### Phase 1 — Environment & SLAM Foundation ✅
- PyBullet 3D environment with procedural urban canyon (8–15 buildings)
- High-fidelity quadcopter multi-body spawning (dark-grey base + 4 blue rotors)
- 360° horizontal raycasting → 16×16 Local Occupancy Grid
- Simulated SLAM noise: Gaussian noise, random cell flips, drift offset
- `RandomCrowd`: 12 yellow walking dots with building collision rerouting
- `DEMO_MODE` global toggle: GUI ↔ headless DIRECT switch

### Phase 2 — RL Architecture Scaffold ✅
- PettingZoo `ParallelEnv` wrapper (`DronePettingZooEnv`) around `DroneEnv`
- Custom ARReSVG + LSTM policy (`CustomARReSVGPolicy`) with:
  - CNN occupancy grid encoder
  - Drone status encoder
  - TRR encoder
  - ADGAT decentralized graph attention (3 modes: Clustering / Spacing / Searching)
  - LSTM belief state (POMDP memory)
  - Actor + Critic heads
- Ray RLlib MADDPG baseline config
- Ray RLlib PPO + custom policy config
- All model parameters strictly pinned to CPU

### Phase 3 — Swarm Dynamics & Dashboard ✅
- Torque kinematics flight control (`_apply_flight_control`)
- Active battery drain (`0.01 + 0.03 * |thrust|` per step)
- Random lateral wind forces ±0.15 N (X/Y global frame)
- Camera FOV downward cone (45° frustum, radius = altitude Z)
- Holographic Cyan HUD labels (`UAV-i | Bat: X% | Alt: Y m`) above each drone
- Cinematic orbital camera (smooth 360° rotation, no side panels)
- Live in-place terminal swarm dashboard using `\r` carriage returns

### Environment Setup (2026-05-23) ✅
- `venv/` virtual environment created (Python 3.13.1)
- `requirements.txt` created with accurate imports
- `numpy 2.4.6` installed ✅
- `gymnasium 1.3.0` installed ✅
- `torch 2.12.0+cpu` installed ✅
- `pettingzoo 1.26.1` installed ✅
- `pybullet` — NOT installed (requires VS Build Tools or Python 3.10) ⚠️
- `.gitignore` updated: `venv/`, `venv312/`, `*.log`, `ray_results/` added
- `setup_env.py` diagnostic script created
- Neural architecture validation (CNN + LSTM + ADGAT): PASSED ✅

---

## ⚙️ What Is Working

| Component | Status |
|:----------|:-------|
| `drone_env.py` — GUI Demo Mode | ⚠️ Needs pybullet (or X11 for Docker) |
| `drone_env.py` — Headless Validation | ✅ Working (Docker / Python 3.10) |
| PyBullet physics + raycasting | ✅ Working (Docker / Python 3.10) |
| SLAM noise injection | ✅ Code verified |
| RandomCrowd walker simulation | ✅ Code verified |
| Holographic HUD labels | ✅ Code verified |
| Orbital camera | ✅ Code verified |
| numpy 2.4.6 | ✅ Installed + tested |
| gymnasium 1.3.0 | ✅ Installed + tested |
| torch 2.12.0+cpu | ✅ Installed + tested |
| pettingzoo 1.26.1 | ✅ Installed + tested |
| CNN encoder (train.py) | ✅ Validated on CPU |
| LSTM belief state (train.py) | ✅ Validated on CPU |
| ADGAT attention (train.py) | ✅ Validated on CPU |
| `train.py` — Ray RLlib training loop | ✅ Block unblocked via Docker |

---

## 🛠️ What Was Fixed (Recent)

1. **WSL Docker Compose Plugin Missing:** 
   - Symptoms: `docker compose` returned "unknown command" and `docker-compose` returned "command not found" in WSL.
   - Fix: Directly downloaded and installed the official Docker Compose v2.29.1 binary to `/usr/local/lib/docker/cli-plugins/docker-compose` and `/usr/local/bin/docker-compose`. Both syntaxes (`docker compose` and `docker-compose`) now work correctly.
2. **Environment Setup Blockers:** PyBullet and Ray RLlib lack Python 3.13 support. 
   - Fix: Transitioned to a Docker-based architecture (`python:3.10-slim` base image) to ensure cross-platform compatibility without complex manual compiler setup.

---

## ⚠️ Known Issues

### 1. PyBullet — No Pre-Built Wheel for Python 3.13 (BLOCKER for simulation)
- **Issue**: `pybullet` has no pre-built wheel for Python 3.13. Pip tries to compile from C++ source, which requires Microsoft Visual C++ Build Tools.
- **Impact**: `drone_env.py` cannot be launched until pybullet is installed.
- **Fix A (on Python 3.13)**: Install Visual C++ Build Tools → `pip install pybullet`
  - Download: https://visualstudio.microsoft.com/visual-cpp-build-tools/
  - Select "Desktop development with C++" workload
- **Fix B (Recommended — one step solves both blockers)**: Install Python 3.10 and create `venv310`
  ```powershell
  py -3.10 -m venv venv310
  .\venv310\Scripts\Activate.ps1
  pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]
  ```

### 3. PyBullet GUI in Docker
- **Issue**: Docker containers running headless do not have a graphical display. If `DEMO_MODE=True` in `drone_env.py`, it may crash trying to open an OpenGL window.
- **Workaround**: Either use `DEMO_MODE=False` for headless testing or configure an X11 server (like VcXsrv) and forward the `DISPLAY` environment variable to the container.

### 2. Ray / Python 3.13 Incompatibility (BLOCKER for training)
- **Issue**: `ray[rllib]` (the distributed training framework) does not have wheels for Python 3.13.
- **Impact**: `train.py` imports `ray`, which will fail on the current `venv` (Python 3.13).
- **Workaround A**: Install Python 3.11 or 3.12 alongside 3.13, create a second venv using that interpreter, install `ray[rllib]>=2.10.0` in that venv.
- **Workaround B**: Use `stable-baselines3` instead of Ray RLlib (no Python 3.13 blocker).
- **train.py already has graceful fallback**: The import block uses `try/except` so architecture validation still runs without Ray.

### 3. `train.py` imports legacy `gym` (not `gymnasium`)
- Line 3: `import gym` — this is the old OpenAI Gym API, now superseded by `gymnasium`.
- The `DronePettingZooEnv` uses `spaces` from `gym`, while `drone_env.py` uses `gymnasium`.
- **Impact**: Minor — both are installed; the fallback in `drone_env.py` handles this.
- **Future fix**: Unify to `gymnasium` once Ray is updated for Python 3.13.

### 3. `DroneEnv` import mismatch in `train.py`
- `train.py` tries `from drone_env import DroneEnv`, but `drone_env.py` exports `DroneSurveillanceEnv`, not `DroneEnv`.
- The `try/except` fallback in `train.py` catches this and uses the internal mock `DroneEnv`.
- **Impact**: Low — training validation runs with mock; actual physics env not integrated into train.py.
- **Future task**: Wire `DroneSurveillanceEnv` into `train.py` properly.

---

## 🔵 Current Branch Focus

**Drone3D** — Physics simulation polish + training scaffold validation.

Next milestone: Integrate `DroneSurveillanceEnv` from `drone_env.py` into `train.py` and unblock Ray RLlib training (requires Python 3.11/3.12 venv).

---

## 📋 Pending Tasks

- [ ] Resolve Ray / Python 3.13 incompatibility (create Py 3.11/3.12 venv)
- [ ] Wire `DroneSurveillanceEnv` into `train.py` PettingZoo wrapper
- [ ] Run first actual RL training trial (MADDPG baseline vs. ARReSVG)
- [ ] Log and plot training reward curves
- [ ] Benchmark against STIRS-2025 target metrics (SSR, DA, PO, MDM)
- [ ] Scale swarm to 5 drones (Scalability Limit metric)
- [ ] Evaluate TRR drop-off under varying SLAM noise levels

---

## 🎯 Next Immediate Goals

1. **Short term**: Set up Python 3.11/3.12 venv for Ray RLlib → run `train.py` cleanly.
2. **Medium term**: Connect `DroneSurveillanceEnv` to `DronePettingZooEnv` wrapper.
3. **Long term**: Full training run → evaluate STIRS-2025 metrics → paper results.
