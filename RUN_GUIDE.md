# 🚀 Running the Multi-UAV Simulation & Training

This guide provides simple instructions on how to run our multi-UAV surveillance environment using local Python or Docker.

---

## 🖥️ Option 1: Local GUI Run (For Active Development & Debugging)

To view the 3D physics environment, hover thrust metrics, battery status, and drone camera cones in real-time, run the simulation locally using your Python environment.

### Command
```powershell
# Make sure your virtual environment is active
# (e.g. .\venv\Scripts\Activate.ps1 or .\venv310\Scripts\Activate.ps1)
python drone_env.py
```

### Details
*   **Pros:**
    *   **Real Visualization:** Opens a PyBullet GUI window showing the city, buildings, quadcopters, and targets.
    *   **Easier Debugging:** Inspect drone behavior, wind effects, collisions, and sensor raycasts visually.
*   **Cons:**
    *   **Higher CPU Usage:** Rendering the 3D graphics window consumes more processor cycles.
    *   **Slower Execution:** Capped at real-time speeds (~50 Hz) for visual playback.

> [!NOTE]
> Make sure `DEMO_MODE = True` at the top of [drone_env.py](file:///d:/IFSP/multi-uav-surveillance/drone_env.py) to enable the GUI window.

---

## 🐳 Option 2: Docker Headless Simulation (For Quick Checking & Testing)

To verify that the code compiles, physics calculations work, and occupancy grids are generated correctly without needing visual displays or compiler setups.

### Command
```bash
docker-compose up simulation
```

### Details
*   **Pros:**
    *   **Lightweight:** No overhead from rendering graphics.
    *   **Fast Checking:** Quickly verifies code sanity and environment states.
    *   **Cross-platform:** Works immediately regardless of local system drivers or Python versions.
*   **Cons:**
    *   **No GUI Visualization:** The environment runs headlessly (outputs text-based logs and ASCII occupancy grids).

---

## 🏋️ Option 3: Reinforcement Learning Training

To run the MADDPG / PPO training loop with our custom CNN + LSTM + ADGAT policy models.

### Command
```bash
docker-compose up training
```

### Details
*   **What it does:** Starts the decentralized reinforcement learning agent training using Ray RLlib inside a Python 3.10 Docker container.
*   **CPU Usage & Waiting Time:**
    *   **⚠️ High CPU Consumption:** All neural network parameter calculations and policy optimizations are pinned to CPU in our configuration. Expect CPU utilization to spike to near 100% on the container cores.
    *   **🕒 Extended Run Times:** RL training requires millions of steps to converge. Expected training runs can range from a few hours for small tests to several days for a complete, published research run. Let it run in the background.
