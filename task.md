# Task List: Swarm Dynamics & Dashboard Polish

## Objective
Implement a realistic 3D quadcopter multi-body model, physical torque kinematics flight control, wind/battery constraints, camera FOV downward cone, hovering holographic debug labels, and a live in-place terminal status dashboard.

## Phase 3 Task Breakdown

- [x] **Swarm Environment & Action Space Update**:
  - [x] Change action space to `(4,)` inputs in range `[-1.0, 1.0]` representing `[Thrust, Pitch, Roll, Yaw]`.
  - [x] Keep observation spaces completely unmodified for downstream compatibility.

- [x] **Realistic Multi-Body Spawning (`_spawn_drones`)**:
  - [x] Create a dark-grey visual and collision box base of `[0.15, 0.15, 0.05]` and mass 1.0kg.
  - [x] Create 4 bright-blue cylindrical rotors of radius 0.08, height 0.02, and mass 0.1kg each.
  - [x] Offset rotors as links attached via fixed joints at the corners: `[0.15, 0.15, 0.03]`, `[-0.15, 0.15, 0.03]`, `[-0.15, -0.15, 0.03]`, and `[0.15, -0.15, 0.03]`.

- [x] **Torque Kinematics & Physical Forces (`_apply_flight_control`)**:
  - [x] Remove the velocity feedback controller.
  - [x] Map thrust action to vertical force in local LINK frame: `thrust_force = (Thrust + 1.0) * 10.0` N.
  - [x] Map roll, pitch, and yaw actions to local LINK frame torques: `Roll * 0.5`, `Pitch * 0.5`, and `Yaw * 0.5`.
  - [x] Support passive restoring torques for attitude stabilization to prevent instant flipping while preserving physics.
  - [x] Force thrust and torques to `0.0` if battery level drops to `0.0%`.

- [x] **Step Updates & Constraints (`step`)**:
  - [x] Initialize battery states to `100.0%` for all agents in `reset()`.
  - [x] Drain battery by `0.01 + 0.03 * abs(thrust_action)` per step.
  - [x] Apply random global lateral wind forces `[-0.15, 0.15]` in X/Y.
  - [x] Implement downward 45-degree camera FOV (`distance d <= Z`) for counting tracked ground crowd boids.
  - [x] Draw hovering HUD labels above each drone using Cyan `[0, 1, 1]` `p.addUserDebugText` at size 1.2.

- [x] **Swarm Demo & Live Terminal Dashboard**:
  - [x] Instantiate the environment with `fixed_layout=True` inside `DEMO_MODE`.
  - [x] Feed flight-stable erratic actions (thrust near 0.38 + random walk, small roll/pitch perturbations) inside the infinite loop.
  - [x] Print the dynamic live status dashboard in-place using `\r` and `flush=True` in the terminal.
