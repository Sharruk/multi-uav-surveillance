# Project Walkthrough: UAV Swarm Dynamics & Swarm Dashboard

Welcome to the **STIRS-2025 UAV Swarm Dynamics & Swarm Dashboard** walkthrough! This document explains the high-fidelity 3D multi-body physical modeling, the torque kinematics control pipeline, battery and wind constraints, camera FOV logic, and real-time visualization features.

---

## 1. High-Fidelity 3D Quadcopter Multi-Body

To simulate realistic quadcopter aerodynamics, the drone URDF fallback was updated with a custom, high-fidelity multi-body structure created directly via `p.createMultiBody`:
* **Core Base Link**: A dark-grey box (`p.GEOM_BOX`) with half-extents `[0.15, 0.15, 0.05]` and mass `1.0 kg`.
* **Rotor Link Arms**: 4 bright-blue cylindrical rotors (`p.GEOM_CYLINDER`) with a radius of `0.08m`, height of `0.02m`, and mass of `0.1 kg` each.
* **Fixed Joints & Offsets**: Rotors are attached as child links using fixed joints (`p.JOINT_FIXED`) and offset to the corners:
  * Corner 1: `[ 0.15,  0.15, 0.03]`
  * Corner 2: `[-0.15,  0.15, 0.03]`
  * Corner 3: `[-0.15, -0.15, 0.03]`
  * Corner 4: `[ 0.15, -0.15, 0.03]`
* **Total Swarm Mass**: `1.4 kg` per UAV.

---

## 2. Torque Kinematics & Flight Control

The velocity-controlled autopilot has been replaced with high-fidelity physical forces and torque inputs representing direct motor command mappings:
* **Vertical Thrust Force**: Applied along the drone's local vertical axis (local `LINK` frame) at the center of mass:
  $$\text{thrust\_force} = (u_{\text{thrust}} + 1.0) \times 10.0 \text{ N}$$
  This scales the thrust from `0.0` to `20.0 N` (balancing the gravitational force of $1.4\text{ kg} \times 9.81\text{ m/s}^2 = 13.734\text{ N}$ at $u_{\text{thrust}} \approx 0.3734$).
* **Rotational Torques**: Applied directly around the local `LINK` frame axes:
  * $\text{roll\_torque} = u_{\text{roll}} \times 0.5$
  * $\text{pitch\_torque} = u_{\text{pitch}} \times 0.5$
  * $\text{yaw\_torque} = u_{\text{yaw}} \times 0.5$
* **Passive Attitude Stabilizer**: A proportional feedback restoring torque ($\tau_{\text{roll}} = -2.0 \phi$ and $\tau_{\text{pitch}} = -2.0 \theta$) is applied to maintain upright stability. This prevents instant flips during random exploration while fully preserving the chaotic, realistic, physical flight characteristics.
* **Low-Power Safety Shutdown**: If a drone's battery level drops to `0.0%`, all forces and torques are forced to `0.0`, resulting in a realistic gravitational crash landing.

---

## 3. Battery & Lateral Wind Constraints

To accurately reflect real-world flight limitations, environmental constraints have been added to the physical step loop:
* **Active Battery Drainage**: Drains battery dynamically depending on thrust command:
  $$\Delta_{\text{battery}} = 0.01 + 0.03 \times |u_{\text{thrust}}| \text{ per step}$$
* **Global Wind Gusts**: Random lateral wind forces in the range `[-0.15, 0.15]` are applied dynamically to each UAV's base along the global X and Y axes (`p.WORLD_FRAME`) on every step, challenging the control algorithms.

---

## 4. Camera FOV Downward Cone

Surveillance coverage is computed using a downward 45-degree angle frustum representing a ground-facing camera:
* **Coverage Circle**: The radius of coverage on the flat ground plane is exactly equal to the altitude $Z$ of the UAV.
* **Detection Rule**: A ground boid target at position $(b_x, b_y)$ is successfully tracked if its horizontal Euclidean distance $d$ from the UAV $(x, y)$ satisfies:
  $$d = \sqrt{(x - b_x)^2 + (y - b_y)^2} \le Z$$
* This replaces the static 2D distance rule with a dynamic camera tracking model, where higher altitude increases coverage area but changes tracking rewards.

---

## 5. Holographic HUD & Live Swarm Terminal Dashboard

Two premium UI features have been integrated to visualize and monitor the live swarm demo:
1. **Holographic 3D HUD Text**:
   * Generates a floating text string `UAV-i | Bat: {bat}% | Alt: {alt}m` hovering `0.5m` above each UAV.
   * Rendered in bright Cyan (`[0, 1, 1]`) with a size multiplier of `1.2`.
   * Updated in-place utilizing PyBullet's `replaceItemUniqueId` parameter to prevent graphics memory leaks.
2. **Live Swarm Terminal Dashboard**:
   * Gathers live telemetry from the three active UAVs.
   * Prints the status in-place inside the console window using carriage returns (`\r`) for beautiful, flicker-free terminal updates.

---

## 6. Verification & API Bug Fixes

### Compiler Checks
The environment code has been verified and successfully compiled with zero errors:
```bash
.\.venv\Scripts\python.exe -m py_compile drone_env.py
```

### PyBullet API Correction
During headless runtime validation, we encountered and successfully resolved a PyBullet API keyword discrepancy:
* **Issue**: Spawning the cylindrical rotors using `p.createVisualShape` with the keyword `height` caused a `TypeError` because PyBullet expects the `length` keyword argument for `GEOM_CYLINDER` inside visual shapes (though it accepts `height` inside collision shapes).
* **Fix**: Updated `p.createVisualShape` parameters for the rotors:
```python
rotor_vis_id = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=0.08,
    length=0.02,  # Fixed: changed from 'height' to 'length'
    rgbaColor=[0.0, 0.6, 1.0, 1.0],
    physicsClientId=self.client_id
)
```

### Headless Simulation Trial
With the parameter corrected, the 10-step headless validation runs successfully to completion and outputs a beautiful, randomized concrete canyon and target occupancy grid:
```
=== Running Quick Headless Validation (DEMO_MODE=False) ===
Environment successfully connected in DIRECT mode!
Number of agents: 3
Building obstacle body count: 15
Moving ground crowd boids count: 12
...
=== Headless validation successful and completed! ===
```
