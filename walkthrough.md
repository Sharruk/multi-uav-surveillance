# Project Walkthrough: UAV Swarm Dynamics, Environmental Assets, & Multi-View Dashboard

Welcome to the **STIRS-2025 UAV Swarm Dynamics & Swarm Dashboard** walkthrough! This document explains the high-fidelity 3D multi-body physical modeling, torque kinematics control pipeline, battery and wind constraints, camera FOV logic, rich visual environmental spawner assets, dynamic bird flocking, wind physics, and real-time visualization features.

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
* **Global Wind Gusts**: Random lateral wind forces are applied dynamically to each UAV's base along the global X and Y axes (`p.WORLD_FRAME`) on every step, challenging the control algorithms.

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

## 6. Premium Environmental Assets & Spawners

A modular set of environment features has been introduced in [`envs/environment_assets.py`](file:///c:/Users/Sundareswaran/ifsp/multi-uav-surveillance/envs/environment_assets.py) using pure PyBullet primitives (requiring zero new package dependencies).

### 6.1 Static Obstacles & Urban Clusters
* **Trees (`TreeSpawner`)**: Procedural clusters containing realistic brown trunks (cylinders) topped by vibrant green foliage spheres. Trees act as 3D physical obstructions, blocking drone path sweeps and returning valid LiDAR collision boundaries.
* **Buildings (`BuildingSpawner`)**: A set of tall landmarks (ranging from 5m up to 20m in height) arranged in dense, urban canyons. Features distinct high-fidelity colors (e.g. glass blue, terracotta, slate grey) and serves as physical boundaries.
* **Houses (`HouseSpawner`)**: Smaller 3m–5m residential structures adorned with sloped color roofs, placed in clean clusters to represent low-altitude residential surroundings.
* **Electric Utility Poles (`ElectricPoleSpawner`)**: 6m tall wooden poles with double cross-arms aligned in clean rows alongside streets (GUI-only).

### 6.2 Dynamic Elements & Forces
* **Wind System (`WindSystem`)**: Directional wind featuring sinusoidal gusting and Gaussian turbulence, which scales physically with altitude:
  $$\text{Wind Force } F_w = F_{\text{base}} \times \left(1 + 0.08 \times (z - 0.5)\right) \times \text{gust\_mult} + \text{Turbulence}$$
  The global wind direction slowly rotates at a rate of $\approx 1.7^\circ$ per step. Drones must dynamically adjust pitch and roll thrust vectors to prevent lateral drift!
* **Bird Flocks (`BirdFlock`)**: A flock of 5–10 kinematic bird spheres mimicking real-world behavior. Birds soar (climb), glide (hold altitude), and dive (descend) inside randomized personal altitude bands (1.5m to 8m). Equipped with separation forces and collision-aware physics, bird strikes apply a direct negative penalty (`-3.0`) to nearby drones.

### 6.3 Ground Terrain Assets
* **Roads and Parks (`RoadAndParkSpawner`)**: To elevate visual aesthetics, the ground now displays a grey asphalt street layout with white lane markings alongside four lush green park quadrants (visible in GUI/Demo mode).

---

## 7. Configuration Customization & Headless Training Modes

To accommodate both beautiful GUI demonstrations and high-performance headless reinforcement learning, a centralized config schema is integrated:

```python
DEFAULT_ENV_CONFIG = {
    "enable_trees":        True,   # LiDAR-relevant
    "enable_houses":       True,   # LiDAR + collision
    "enable_poles":        True,   # GUI only
    "enable_birds":        True,   # collision-enabled
    "enable_wind_physics": True,   # replaces old random wind
    "enable_roads":        True,   # GUI only
    "num_trees":           12,
    "num_birds":           8,
    "wind_base_speed":     0.4,    # m/s
    "num_houses":          6,
    "num_tall_buildings":  3,
}
```

To maximize step rate during training, developers can easily initialize a stripped-down environment:
```python
env = DroneSurveillanceEnv(
    render_mode="headless",
    env_config={
        "enable_poles": False,
        "enable_roads": False,
        "num_trees": 5,
        "num_birds": 3
    }
)
```

---

## 8. Multi-View Camera System & Keyboard Controls

When running any evaluation demo (PPO, SDDPG, or Attention Distillation) in GUI mode, you can change the view dynamically using your keyboard:

| Key | View Mode | Details |
|-----|-----------|---------|
| `1` | **ORBITAL** | Sweeps a continuous $360^\circ$ cinematic orbit around the swarm centroid. |
| `2` | **TOP-DOWN** | Locks in a strict overhead bird's-eye perspective tracking the swarm. |
| `3` | **CINEMATIC** | Pulls close with dramatic tilt angles and rotation for maximum graphic showcase. |
| `4` | **CLOSE-UP** | Focuses directly on the drone swarm with narrow distance and static tracking. |

An active text overlay shows the current view mode and drone altitudes on the fly.

---

## 9. Verification & Compatibility Results

Headless and backward compatibility suites are in place to ensure zero breaking changes to existing model training runs:

* **PPO Shared Policy Baseline**: **PASSED** (all attributes, rewards, and constraints fully compatible).
* **SDDPG-NAV split-actor**: **PASSED** (active target guidance and LiDAR boundaries preserved).
* **Attention Policy Distillation**: **PASSED** (occupancy grids and decentralised structures intact).
* **Configuration Customization**: **PASSED** (birds, trees, buildings dynamically toggleable).

---

## 10. Advanced Environment Polish & Optimization

To elevate visual authenticity and operational robustness, three major improvements have been made:
1. **High-Accuracy AABB Bird Avoidance**: Kinematic birds now query static obstacle bounding boxes (`p.getAABB`) on each step to calculate a multi-dimensional closest-point vector. This guarantees that birds steer naturally around and over structures of any geometry or scale—from dense 20m office towers to residential roofs and green canopies—completely eliminating building/tree penetration.
2. **Smooth Multi-View Camera Transition**: Keyboard event polling was optimized to check if the user is switching to a *new* view mode before resetting camera state. This prevents visual camera resetting/locking during continuous or rapid keyboard presses, ensuring smooth orbital panning and cinematic rotation.
3. **Swarm Mutual Drone-to-Drone Avoidance**: A real-world-inspired swarm potential field (repulsion force) has been integrated into the flight control loops of all three swarm architectures (PPO, SDDPG, and Attention Distillation). When drones come within a `2.5m` radius of one another, they generate a mutual horizontal separation vector scaled inversely by the square of their distance (`1 / d^2`). This ensures they actively steer away from each other and never overlap, collide, or get stuck during multi-drone target chases.
