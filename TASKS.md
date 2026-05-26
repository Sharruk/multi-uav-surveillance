# TASKS.md
# Decentralized Multi-UAV Surveillance — Task Tracker

> **Project:** STIRS-2025 | **Branch:** Drone3D
> **Team:** Jeswin + teammates

---

## ✅ Completed

### Environment (Phase 1)
- [x] Initialize PyBullet 3D environment (`DroneSurveillanceEnv`)
- [x] Procedural urban canyon: 8–15 randomized buildings with collision checking
- [x] High-fidelity quadcopter multi-body model (box + 4 cylinder rotors, FIXED joints)
- [x] 360° horizontal raycasting → 16×16 Local Occupancy Grid
- [x] `inject_slam_noise`: Gaussian noise, random cell flips, spatial drift offset
- [x] `RandomCrowd`: 12 yellow ground targets with building avoidance rerouting
- [x] `DEMO_MODE` master toggle: GUI ↔ headless DIRECT
- [x] `fixed_layout` parameter for reproducible/seeded scenes

### RL Scaffold (Phase 2)
- [x] PettingZoo `ParallelEnv` wrapper (`DronePettingZooEnv`)
- [x] Custom ARReSVG + LSTM policy (`CustomARReSVGPolicy`)
  - [x] CNN occupancy grid encoder
  - [x] Drone status encoder (linear 6→32)
  - [x] TRR encoder (linear 1→16)
  - [x] ADGAT graph attention (3 behavioral modes)
  - [x] LSTM belief state (POMDP memory, 128 units)
  - [x] Actor head (LSTM→4 continuous actions)
  - [x] Critic head (LSTM→1 value)
- [x] Ray RLlib MADDPG baseline config scaffold
- [x] Ray RLlib PPO + custom policy fallback config
- [x] CPU-only parameter enforcement

### Swarm Dynamics & UI (Phase 3)
- [x] Torque kinematics flight control (`_apply_flight_control`)
- [x] Battery drain constraint (`0.01 + 0.03 * |thrust|` per step)
- [x] Random lateral wind forces ±0.15 N (global X/Y)
- [x] Camera FOV downward cone (radius = altitude Z)
- [x] Holographic Cyan HUD labels above each drone
- [x] Cinematic orbital camera (auto-rotating, no UI panels)
- [x] Live in-place terminal swarm dashboard (`\r`)
- [x] Fixed PyBullet `GEOM_CYLINDER` visual shape API bug (`height` → `length`)
- [x] Removed all debug line draw calls (no more GPU warnings)

### Project Setup (2026-05-23)
- [x] Create `requirements.txt` from accurate import analysis
- [x] Install `numpy 2.4.6`, `gymnasium 1.3.0`, `torch 2.12.0+cpu`, `pettingzoo 1.26.1` in venv
- [x] Neural architecture validation: CNN + LSTM + ADGAT all pass on CPU
- [x] Create `setup_env.py` diagnostic script
- [x] Create `CURRENT_STATUS.md`
- [x] Create `CHANGELOG.md`
- [x] Create `TASKS.md`
- [x] Update `README.md`
- [x] Fix Docker setup for CPU-only training and Headless simulation (`HEADLESS=1`)

---

## 🔄 In Progress

- [ ] **Install pybullet** (BLOCKER for simulation)
  - Option A: Install VS Build Tools then `pip install pybullet` (on current Python 3.13 venv)
    - Download: https://visualstudio.microsoft.com/visual-cpp-build-tools/
    - Re-run: `.\venv\Scripts\python.exe -m pip install pybullet`
  - Option B (RECOMMENDED): Install Python 3.10 + create `venv310` (unlocks both pybullet AND ray)
    - `py -3.10 -m venv venv310` then `pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]`
- [ ] Verify `drone_env.py` launches cleanly after pybullet install
- [ ] Verify `train.py` architecture validation runs (without Ray)

---

## 📋 Pending

### Critical Path (Training Unblock)
- [ ] Install Python 3.11 or 3.12 (for Ray RLlib compatibility)
- [ ] Create second venv: `venv312/` using Python 3.12 interpreter
- [ ] Install `ray[rllib]>=2.10.0` in Python 3.12 venv
- [ ] Verify `train.py` imports Ray without error

### Integration
- [ ] Wire `DroneSurveillanceEnv` from `drone_env.py` into `train.py`'s `DronePettingZooEnv`
  - Currently `train.py` uses a mock `DroneEnv` fallback
  - Connect the real physics simulation to the PettingZoo wrapper
- [ ] Align observation spaces: `drone_env.py` uses `(position, velocity, lidar, occupancy_grid)` but `train.py` expects `(occupancy_grid, drone_status, neighbor_info, target_trr)`
- [ ] Unify gym API: migrate `train.py` from legacy `gym` to `gymnasium`

### Training Runs
- [ ] Run first MADDPG baseline training trial (10k steps)
- [ ] Run first ARReSVG training trial (10k steps)
- [ ] Save checkpoint models
- [ ] Log episode rewards, tracked targets, collision counts
- [ ] Plot reward curves (matplotlib)

### Evaluation (STIRS-2025 Metrics)
- [ ] **SSR** (Swarm Success Rate): collision-free mission ratio ≥ 85%
- [ ] **DA** (Dynamic Adaptability): POMDP recalculation time ≤ 0.12 s
- [ ] **PO** (Path Optimality): actual/shortest path ratio ≤ 1.1
- [ ] **MDM** (Min Distance Margin): separation 1.0–5.0 m maintained
- [ ] **Scalability**: CT ≤ 0.5 s with 5 drones
- [ ] **TRR resilience**: minimal drop-off under SLAM noise injection

---

## 💡 Research Ideas

- **Noise curriculum**: gradually increase SLAM noise over training episodes
- **Communication failure robustness**: drop neighbor info with probability p and test TRR
- **Energy-optimal hovering**: add altitude regularization reward to minimize battery drain
- **Multi-objective reward shaping**: Pareto front between tracking reward and safety margin
- **Scalability experiment**: 3 → 5 → 8 drone swarms; log CT and SSR degradation
- **Compare ADGAT modes**: ablation study (disable specific modes, measure tracking coverage)
- **Transfer learning**: pre-train on fixed_layout=True, fine-tune on procedural maps
- **Video recording**: headless rendering to MP4 for conference demo
