# 🚀 Run Commands Guide

This file contains the exact commands you need to run the Drone3D project in various modes.

## 🖥️ Local Run (GUI Visualization)

To run the simulation locally with the 3D PyBullet GUI and holographic HUD:

```powershell
# Windows PowerShell
.\venv\Scripts\Activate.ps1
python drone_env.py
```

*Note: You must have PyBullet installed in your local environment. If you want to force headless mode locally, set `HEADLESS=1` before running.*

## 🐳 Docker Simulation (Headless Mode)

To run the simulation validation in Docker without opening a GUI window (CPU-only):

```bash
docker-compose up --build simulation
```

*This will output the 16x16 Local Occupancy Grid as an ASCII printout in your terminal.*

## 🏋️ Docker Training (CPU-only)

To start the Ray RLlib / PyTorch training loop on CPU:

```bash
docker-compose up --build training
```

*Note: The environment is configured strictly for CPU-only execution to ensure compatibility across all team members' machines. No NVIDIA GPU is required.*

## 🛠️ Troubleshooting Common Issues

1. **`cannot connect to X server` when running Docker**
   * **Cause:** The simulation is trying to open a PyBullet GUI window inside Docker without an X11 server forwarded.
   * **Fix:** The `docker-compose.yml` is now updated with `HEADLESS=1` for the `simulation` service, which automatically runs `drone_env.py` in headless mode. No further action needed!

2. **`could not select device driver "nvidia" with capabilities: [[gpu]]`**
   * **Cause:** Docker Compose was requesting NVIDIA GPU access on a machine without the NVIDIA Container Toolkit.
   * **Fix:** This has been resolved. The `docker-compose.yml` has been updated to remove the `deploy` block, forcing purely CPU execution for both simulation and training.

3. **High CPU Usage During Training**
   * **Cause:** By design, all tensors and models are pinned to the CPU to guarantee team-wide compatibility.
   * **Fix:** This is normal. Let the container run in the background.
