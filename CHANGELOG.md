# CHANGELOG.md
# Decentralized Multi-UAV Surveillance — Change History

> Format: Date | Branch/Person | What Changed | Why | Issues Fixed | Next Step

---

## [2026-05-23] — Setup & Documentation Pass
**Branch:** Drone3D | **By:** Sharruk using Antigravity (AI assistant)

**What changed:**
- Created `requirements.txt` from accurate import analysis of `drone_env.py` and `train.py`
- Installed `numpy`, `pybullet`, `gymnasium` into existing `venv/` (Python 3.13.1)
- Installed `torch` (CPU-only) and `pettingzoo` into `venv/`
- Created `CURRENT_STATUS.md` — team-readable project state tracker
- Created `CHANGELOG.md` (this file) — lightweight update history
- Created `TASKS.md` — task tracker with completed/pending/research sections
- Updated `README.md` — complete setup + run instructions, troubleshooting

**Why:**
- `venv/` existed but had zero packages installed — simulation wouldn't launch
- No documentation existed for team continuity (setup steps, known issues)

**Issues fixed:**
- `pybullet_data` is not a PyPI package (it ships inside `pybullet`) — removed incorrect install attempt
- Documented Ray / Python 3.13 incompatibility so team is not blocked silently

**Next step:**
- Verify simulation launches (`venv\Scripts\python.exe drone_env.py`)
- Resolve Ray blocker (Python 3.11/3.12 venv for training)

---

## [Phase 3 Complete] — Swarm Dynamics & Dashboard Polish
**Branch:** Drone3D | **By:** Jeswin

**What changed:**
- `drone_env.py`: Replaced velocity controller with torque kinematics flight control
- `drone_env.py`: Added battery drain (`0.01 + 0.03 * |thrust|` per step)
- `drone_env.py`: Added random lateral wind forces ±0.15 N (global X/Y)
- `drone_env.py`: Added camera FOV downward cone (45°, radius = altitude Z)
- `drone_env.py`: Added holographic Cyan HUD labels floating above drones
- `drone_env.py`: Added cinematic orbital camera (clean, no side panels)
- `drone_env.py`: Added live in-place terminal swarm dashboard (`\r` carriage return)
- `drone_env.py`: Added `fixed_layout` parameter for reproducible demo layouts
- `drone_env.py`: Removed all debug line draw calls (no more `User debug draw failed` warnings)

**Why:**
- Phase 3 requirement: Physics-realistic flight simulation with battery and wind constraints
- Camera FOV cone replaces simplistic 5m 2D circle — more accurate surveillance model
- Visual cleanup for cinematic demo presentation

**Issues fixed:**
- PyBullet `createVisualShape(GEOM_CYLINDER)` requires `length=`, not `height=` → fixed
- Removed `p.addUserDebugLine` LiDAR lines that caused GPU warnings on headless runs

**Next step:**
- Phase 4: Begin actual RL training runs (requires Python 3.11/3.12 for Ray)

---

## [Phase 2 Complete] — RL Architecture Scaffold
**Branch:** Drone3D | **By:** Jeswin

**What changed:**
- `train.py`: PettingZoo `ParallelEnv` wrapper (`DronePettingZooEnv`)
- `train.py`: Custom ARReSVG + LSTM recurrent policy (`CustomARReSVGPolicy`)
- `train.py`: ADGAT 3-mode attention routing (Clustering / Spacing / Searching)
- `train.py`: Ray RLlib MADDPG baseline config
- `train.py`: Ray RLlib PPO + custom policy fallback config
- `train.py`: CPU-only enforcement for all model parameters

**Why:**
- Implements proposed POMDP belief-state architecture for STIRS-2025 benchmarking
- MADDPG baseline required for comparison against proposed ARReSVG approach

**Issues fixed:**
- RLlib model registry via `ModelCatalog.register_custom_model` — confirmed working
- LSTM state shape handling for `h_in`/`c_in` unsqueeze/squeeze

**Next step:**
- Phase 3: Add physical realism (torque control, battery, wind)

---

## [Phase 1 Complete] — Environment & SLAM Foundation
**Branch:** Drone3D | **By:** Jeswin

**What changed:**
- `drone_env.py`: Full PyBullet 3D environment (`DroneSurveillanceEnv`)
- Procedural urban canyon with 8–15 randomized buildings
- High-fidelity quadcopter multi-body model (dark-grey box + 4 blue cylinder rotors)
- 360° horizontal raycasting → 16×16 Local Occupancy Grid
- `inject_slam_noise` function: Gaussian noise, random flips, drift offset
- `RandomCrowd`: 12 yellow walking dots with building collision rerouting

**Why:**
- Phase 1 requirement: Simulate a realistic urban surveillance environment
- SLAM noise needed to simulate real-world sensor imperfections

**Issues fixed:**
- Building spawn collision checker to prevent spawning on drone positions
- `RandomCrowd` replaced BoidsCrowd (simpler, more predictable linear walkers)

**Next step:**
- Phase 2: Add RL training scaffold (PettingZoo wrapper + Ray RLlib)
