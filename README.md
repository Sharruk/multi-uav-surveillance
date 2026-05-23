# Decentralized Multi-UAV Surveillance Swarm using POMDP Framework

> [!NOTE]  
> **Author:** Jeswin (CSE Batch 2024-29, SSN College of Engineering)  
> **Research Context:** Aligning with Advanced Multi-Agent DRL and Graph Attention (ADGAT) research standards for the STIRS-2025 framework.

---

## 🚁 Project Overview

This repository hosts a high-fidelity SITL (Software-in-the-Loop) decentralized surveillance proof-of-concept. A swarm of three autonomous quadcopters navigates a procedurally generated "concrete canyon" (urban city blocks) to search for and track moving ground targets. 

UAV decision-making operates under a **Partially Observable Markov Decision Process (POMDP)** framework. Rather than accessing global maps or absolute positions, each drone uses local horizontal raycasting sensors to generate an imperfect **16x16 Local Occupancy Grid** subject to simulated SLAM mapping drift and sensor interference.

---

## 🛠️ Technology Stack

| Component | Technology | Role |
| :--- | :--- | :--- |
| **Physics Engine** | **PyBullet** | Handles 3D rigid-body aerodynamics, link coordinates, collision constraints, and efficient batch raycasting. |
| **Multi-Agent API** | **PettingZoo (ParallelEnv)** | Standardized ecosystem mapping multi-agent actions, step loops, observations, and information dictionaries. |
| **Deep RL Framework** | **Ray RLlib** | Coordinates centralized training and decentralized execution using state-of-the-art DRL algorithms (MADDPG/PPO). |
| **Deep Learning Engine** | **PyTorch** | Powers custom recurrent network definitions (ARReSVG + LSTM memory structures) with strictly enforced CPU mapping. |
| **Core Environment** | **Gymnasium (OpenAI Gym)** | Provides base class environment structures, registration utilities, and standard observation/action `spaces`. |
| **Vector Math & Arrays** | **NumPy** | Performs fast matrix transformations for local occupancy grids and distance math. |
| **Visual Rendering** | **OpenGL (PyBullet GUI)** | Renders real-time lighting, shadows, and interactive holographic Cyan 3D overlay text HUDs. |

---

## 🌟 Key Features

### 🏢 1. Procedural Concrete Canyon Generation
- **Dynamic Obstacles**: Spawns `8-15` buildings with randomized coordinates, widths (1m to 3m), and heights (2m to 8m).
- **Collision Checking**: Employs an active footprint collision-checking loop to guarantee obstacles never spawn directly on top of the drones' initial starting positions.
- **Visual Depth**: Assigns each building block a concrete texture with randomized grey shades (RGB `[0.55, 0.65]`) to create depth and visual realism.

### 🚁 2. High-Fidelity Multi-Body UAV Spawning
- **Rigid Physical Assembly**: Constructed dynamically in PyBullet using `p.createMultiBody`.
- **5-Link Rigid Topology**:
  - **Chassis Base**: A dark-grey box base of `[0.15, 0.15, 0.05]` m (Mass: `1.0 kg`).
  - **Rotors**: 4 bright-blue cylindrical rotors of radius `0.08` m and thickness `0.02` m (Mass: `0.1 kg` each) rigidly attached via fixed joints (`p.JOINT_FIXED`).
- **Total Physical Mass**: `1.4 kg` per UAV.

### ⚙️ 3. Direct Torque Kinematics Control
- **Thrust Force**: Applied vertical thrust force in local `LINK_FRAME`:
  $$\text{thrust\_force} = (u_{\text{thrust}} + 1.0) \times 10.0 \text{ N}$$
  *(Balances the gravitational force of $1.4\text{ kg} \times 9.81\text{ m/s}^2 = 13.734\text{ N}$ at $u_{\text{thrust}} \approx 0.3734$)*.
- **Directional Torques**: Physical local torque vector `[roll_torque, pitch_torque, yaw_torque]` mapped directly from continuous action inputs `[-1.0, 1.0]`:
  $$\tau_{\text{roll}} = u_{\text{roll}} \times 0.5 \quad | \quad \tau_{\text{pitch}} = u_{\text{pitch}} \times 0.5 \quad | \quad \tau_{\text{yaw}} = u_{\text{yaw}} \times 0.5$$
- **Proportional Attitude Stabilization**: Integrates custom stabilization torques ($\tau_{\text{stabilize}} = -2.0 \cdot \theta_{\text{current}}$) to maintain upright stability. This prevents instant flips during training while preserving realistic physical tilting and drift.

### 🔋 4. Physical Constraints: Wind & Battery
- **Wind Gusts**: Random lateral forces in the range `[-0.15, 0.15]` N are applied along the global X and Y axes (`p.WORLD_FRAME`) on every step.
- **Active Battery Drain**: Power drains dynamically based on the vertical thrust effort:
  $$\Delta_{\text{battery}} = 0.01 + 0.03 \times |u_{\text{thrust}}| \text{ % per step}$$
- **Battery Safety Cutoff**: Drones undergo a complete low-power shutdown (thrust and torques drop to `0.0`) when battery hits `0.0%`, leading to gravitational crashes.

### 👁️ 5. Downward Camera FOV & Simulated SLAM
- **45° Cone Frustum**: Computes surveillance coverage using a downward-facing camera cone where the radius of detection is equal to the UAV's altitude ($Z$):
  $$d_{\text{horizontal}} = \sqrt{(x_{\text{uav}} - x_{\text{target}})^2 + (y_{\text{uav}} - y_{\text{target}})^2} \le Z_{\text{uav}}$$
- **SLAM Noisy Occupancy Grid**: Adds configurable spatial coordinate drift, cell flips, and Gaussian noise into the 16x16 grid to simulate real-world SLAM mapping anomalies.

### 📊 6. Swarm Dashboard & Orbital Cinematic UI
- **Cinematic Orbital Tracking**: Beautiful orbital camera rotations and clean viewing styles (no visual debug lines or PyBullet side panels).
- **Holographic 3D Text overlays**: Floating bright Cyan labels (`UAV-i | Bat: {bat}% | Alt: {alt}m`) hover `0.5m` above each UAV. Updates cleanly in-place using PyBullet's `replaceItemUniqueId` to prevent memory leaks.
- **In-Place Terminal Dashboard**: Aggregates real-time battery, altitude, and tracking telemetry into a single Console Status Bar using `\r` carriage returns.

---

## 📂 Repository Structure

```
drone_test/
├── .venv/                  # Local virtual Python environment
├── drone_env.py            # Primary Gym/PettingZoo UAV environment
├── train.py                # Ray RLlib baseline training and memory wrapper
├── status.md               # Phase-by-phase project status log
├── task.md                 # Completion checklist
├── walkthrough.md          # Technical walkthrough & PyBullet API details
├── project.md              # Project outline and requirements
├── metrics.md              # Target evaluation metrics
└── README.md               # Project documentation (this file)
```

---

## 🛠️ Installation & Setup

1. **Prerequisites**: Ensure you have Python 3.10+ installed on your Windows system.
2. **Setup Virtual Environment**:
   ```powershell
   # Activate local environment
   .\.venv\Scripts\Activate.ps1
   ```
3. **Core Dependencies**:
   * PyBullet (physics engine)
   * Gymnasium / PettingZoo (multi-agent RL environments)
   * Ray RLlib (distributed training)
   * PyTorch (tensor computation, CPU-only mapping enforced)
   * NumPy

---

## 🚀 Running the Code

### 🖥️ 1. Interactive Live Swarm Demo (GUI)
Run the script directly with default parameters to visualize the physical multi-body drones, concrete canyons, wind drifts, and the holographic HUD:
```powershell
.\.venv\Scripts\python.exe drone_env.py
```
*Press `Ctrl+C` in your terminal to safely terminate the demo loop.*

### 🧪 2. Quick Headless Validation (Direct Physics)
To test the raw mathematical execution, local SLAM grid processing, and observations without PyBullet's graphical GUI, toggle `DEMO_MODE = False` at the top of `drone_env.py` and run:
```powershell
.\.venv\Scripts\python.exe drone_env.py
```
*This will run a 10-step trial and print a simulated SLAM occupancy grid in ASCII inside your console.*

### 🚂 3. Multi-Agent Training (Ray RLlib)
To start multi-agent training using Ray RLlib (mapping our custom PettingZoo environment wrappers into MADDPG or PPO recurrent baselines with LSTM memory models):
```powershell
.\.venv\Scripts\python.exe train.py
```

---

## 📈 Evaluation & STIRS-2025 Target Benchmarks

The system is designed to evaluate decentralized swarm performance against MADDPG baselines under the following rigorous metrics:

| Metric | Definition | Target Goal |
| :--- | :--- | :--- |
| **Swarm Success Rate (SSR)** | Ratio of collision-free surveillance missions completed | $\ge 85\%$ |
| **Dynamic Adaptability (DA)** | Recalculation time of POMDP belief state during local grid updates | $\le 0.12\text{ s}$ |
| **Path Optimality (PO)** | Ratio of actual flight path length to the absolute shortest path | $\le 1.1$ |
| **Min Distance Margin (MDM)** | Separation distance maintained between UAVs and buildings | $1.0\text{ to }5.0\text{ m}$ |
| **Scalability Limit** | Execution computation time (CT) as swarm scales to 5 drones | $\text{CT} \le 0.5\text{ s}$ |
| **Resilience Log** | Target Recognition Rate (TRR) drop-off curve under injected SLAM noise | High robustness (minimal TRR decay) |

---

## 💡 Technical Reference Notes

* **GPU Offloading**: Headless direct pipelines are fully decoupled from GPU dependencies, ensuring training scaling remains 100% stable on CPU tensors for hardware compatibility (e.g. 16GB RAM laptops).
* **Observation Space Keys**: Keeps observations to standard keys (`position`, `velocity`, `lidar`, `occupancy_grid`) to guarantee compatibility with all standard RL libraries (Ray, StableBaselines3).
