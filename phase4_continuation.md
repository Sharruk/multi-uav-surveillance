# Phase 4 Continuation File — STIRS-2025

This file exists so a new Claude session can resume Phase 4 without re-reading the conversation.
Read this entire file before doing anything. Then check git status and task list.

---

## Current State (as of last write)

**Branch:** `environment`
**Last commit before Phase 4:** `a40cab3` (assets: add Phase 3 scenario screenshots)

**Phase 4A — DONE (committed separately below after docs are reviewed):**
- [x] `README_ENVIRONMENT.md` — complete environment reference manual
- [x] `SCENARIO_DETAILS.md` — technical per-scenario specs from actual source code
- [x] `CONFIGURATION_GUIDE.md` — YAML reference, tuning guide, custom scenario walkthrough

**Phase 4B — TODO:**
- [ ] `envs/metrics/__init__.py` (empty package init)
- [ ] `envs/metrics/metrics_dashboard.py` — FPS monitor, drone status, crowd analytics, PyBullet debug text overlay
- [ ] `envs/metrics/data_logger.py` — CSV logger: metrics.csv, collisions.csv, trajectories.csv
- [ ] Wire `MetricsDashboard` into `drone_env.py` step() (optional, low-priority)

**Phase 4C — TODO:**
- [ ] `utils/__init__.py` (empty)
- [ ] `utils/video_recorder.py` — capture PyBullet frames, save as MP4 (use imageio or opencv)
- [ ] `utils/screenshot_tool.py` — 4-angle standardised screenshot capture function
- [ ] Add `--record` / `--screenshot` / `--output-dir` flags to `main_selector.py`

**Phase 4D — TODO:**
- [ ] `experiments/__init__.py` (empty)
- [ ] `experiments/experiment_presets.py` — 5 experiment configs (occlusion_stress_test, dense_crowd_tracking, wind_resilience_test, collision_avoidance_gauntlet, baseline_comparison)
- [ ] `experiments/batch_runner.py` — CLI runner: --experiments, --algorithms, --runs, --headless

**Phase 4E — TODO:**
- [ ] `utils/figure_generator.py` — 4-panel scenario overview PNG, FPS bar chart PNG, LaTeX tables

---

## Key Facts for Code Generation

### Project root
`C:\Users\Sundareswaran\ifsp\multi-uav-surveillance\`

### Environment class
`envs/drone_env.py` — `DroneSurveillanceEnv(render_mode, fixed_layout, env_config)`

### Phase 3 FPS (actual benchmarked, 5-drone load)
- downtown: 108.9 FPS
- residential: 134.7 FPS
- event: 118.1 FPS
- mixed: 117.2 FPS
- industrial: 133.5 FPS

### Crowd agent counts
- downtown: 15 (10 sidewalk + 5 bus_stop)
- residential: 8 (5 park + 3 sidewalk)
- event: 70 (50 plaza_dense + 20 sidewalk)
- mixed: 35 (25 sidewalk + 10 park)
- industrial: 10 (8 sidewalk + 2 park)

### Key env attributes set after reset()
```python
env.crowd_sim        # CrowdSimulator | None
env.boid_birds       # BoidBirds | None
env.layout           # CityLayout | None
env.client_id        # PyBullet physics client int
env.agents           # list of drone_id strings e.g. ["drone_0", "drone_1", "drone_2"]
env.action_spaces    # dict[drone_id → Box(4)]
env.crowd_sim.boid_positions  # list of np.array([x,y]) — one per crowd agent
env.boid_birds.positions      # np.ndarray shape (N,3) — one row per bird
```

### Observation keys per drone
```python
obs[drone_id] = {
    "position": np.array([x,y,z]),
    "velocity": np.array([vx,vy,vz]),
    "lidar": np.array(shape=(36,)),
    "occupancy_grid": np.array(shape=(16,16)),
}
```

### Screenshot pattern (no PIL — pure Python PNG)
See `scripts/phase3_showcase.py` for the `write_png()` and `capture()` helpers.
They work in headless mode. Reuse them in utils/screenshot_tool.py rather than reinventing.

### Main selector CLI
`main_selector.py` already has `--scenario` and `--algorithm` argparse flags.
Add `--record`, `--screenshot`, `--output-dir` to the existing parser.

### Algorithm names (CLI values)
- `ppo` → PPO (Multi-Agent)
- `ddpg` → SDDPG-NAV
- `distill` → Attention Distillation

### Existing utility scripts
- `scripts/phase3_showcase.py` — screenshot + benchmark (reference for new utils)
- `scripts/capture_screenshots.py` — legacy per-scenario screenshots

### Video encoding note
PyBullet returns RGBA uint8 numpy arrays from `getCameraImage()`. For MP4:
- Prefer `imageio` with `ffmpeg` backend: `imageio.get_writer(path, fps=30, codec='libx264')`
- Fallback: `cv2.VideoWriter` if OpenCV is available
- Last resort: save frame PNGs and note that ffmpeg CLI can stitch them

---

## Commit Strategy for Phase 4

One commit per logical unit:
1. `docs: add Phase 4A documentation (README_ENVIRONMENT, SCENARIO_DETAILS, CONFIGURATION_GUIDE)`
2. `feat(metrics): add metrics_dashboard and data_logger`
3. `feat(utils): add video_recorder and screenshot_tool, add CLI flags to main_selector`
4. `feat(experiments): add experiment_presets and batch_runner`
5. `feat(utils): add figure_generator with LaTeX table output`

No Claude co-authorship in any commit message.

---

## User Instructions & Style Preferences

- No Claude co-authorship in commits
- Separate commits per logical change (best version control)
- No trailing summaries in responses ("I did X and Y")
- Show each document / module as it's completed for review
- No emojis in files unless asked
- No unnecessary comments in code (only non-obvious WHY comments)

---

## Phase 5 (Not Yet Scoped)

Mentioned in project memory as "Phase 4 (lighting improvements, skybox, weather visual effects)" — that is the *original* roadmap Phase 4. The current Phase 4 is the research-ready finalization requested explicitly. Confirm with user before starting Phase 5.
