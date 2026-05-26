\# Project: Decentralized Multi-UAV Surveillance using POMDP Framework

\*\*Author:\*\* Jeswin (CSE Batch 2024-29, SSN College of Engineering)



# Project: Decentralized Multi-UAV Surveillance using POMDP Framework

**Author:** Jeswin (CSE Batch 2024-29, SSN College of Engineering)



## 1. System Architecture (Optimized for 16GB RAM / Intel CPU+iGPU)

* **Physics & Environment:** `gym-pybullet-drones` (iGPU handles basic OpenGL rendering; CPU handles physics).

* **Multi-Agent Training:** Ray RLlib + PettingZoo API (Strict CPU-only tensors).

* **Flight Control:** Direct Torque Kinematics control (vertical thrust force in local link frame, roll/pitch/yaw local torques, and passive proportional attitude stabilizer for physics-stable flight).

* **Sensing & Mapping (SLAM Substitute):** 360-degree horizontal raycasting mapped to a 16x16 Local Occupancy Grid with configurable SLAM drift noise.

* **Communication Layer:** Decentralized Graph Attention Network (ADGAT) scaffolded.

* **Crowd Simulation:** `RandomCrowd` linear-walking targets navigating ground planes with active building collision check rerouting (replacing flocking boids).



## 2. Core Objectives

Develop a SITL Proof of Concept demonstrating a physics-stable quadcopter swarm tracking a moving ground target crowd inside concrete canyons. Drones operate under physical wind and battery constraints, relying on a partially observable camera FOV and imperfect Local Occupancy Grids.



## 3. STIRS-2025 Research Alignment

To satisfy STIRS-2025 innovation requirements, the proposed architecture (ARReSVG + ADGAT) must be empirically benchmarked against an industry-standard Multi-Agent Deep Deterministic Policy Gradient (MADDPG) baseline to prove advancements in decentralized coordination and uncertainty handling.



\## 4. Agent Operating Rules

\* \*\*Parallel Execution:\*\* Spawn separate subagents for environment setup and RLlib network definitions.

\* \*\*Hardware Limit:\*\* Do not import PyTorch with CUDA requirements; strictly enforce CPU device mapping.

\* \*\*Artifacts:\*\* Output all training loop scripts and PyBullet configurations as clean, executable Python files.

