"""
STIRS-2025 | Scenario Screenshot & FPS Benchmark Tool
Captures a high-quality render of each scenario using PyBullet's
software ray-tracer (ER_TINY_RENDERER), then benchmarks step rate.
No PIL / external image libraries required.
"""
# Force UTF-8 output on Windows so special chars don't crash
import sys, os
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import time, struct, zlib
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
os.environ["HEADLESS"] = "1"   # use DIRECT physics client

import numpy as np
import pybullet as p
import pybullet_data

from envs.drone_env import DroneSurveillanceEnv


# ── PNG writer (no PIL) ───────────────────────────────────────────────────────
def write_png(path, rgba_hwc):
    """Save HxWx4 uint8 RGBA numpy array as PNG using only stdlib."""
    h, w = rgba_hwc.shape[:2]
    rgb = rgba_hwc[:, :, :3].astype(np.uint8)

    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    out  = b"\x89PNG\r\n\x1a\n"
    out += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    out += chunk(b"IDAT", zlib.compress(raw, 6))
    out += chunk(b"IEND", b"")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(out)


def capture(client_id, dist, pitch, yaw, target=(0, 0, 2),
            width=1280, height=720):
    """Render the scene from the given camera pose."""
    vm = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=list(target),
        distance=dist, yaw=yaw, pitch=pitch, roll=0, upAxisIndex=2,
        physicsClientId=client_id,
    )
    pm = p.computeProjectionMatrixFOV(
        fov=58, aspect=width / height,
        nearVal=0.2, farVal=120,
        physicsClientId=client_id,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width=width, height=height,
        viewMatrix=vm, projectionMatrix=pm,
        renderer=p.ER_TINY_RENDERER,
        lightDirection=[1.5, 2.5, 4.0],
        lightColor=[1.0, 0.97, 0.92],
        lightDistance=120,
        lightAmbientCoeff=0.45,
        lightDiffuseCoeff=0.85,
        lightSpecularCoeff=0.3,
        physicsClientId=client_id,
    )
    return np.array(rgba, dtype=np.uint8).reshape(height, width, 4)


# ── Scenario definitions ──────────────────────────────────────────────────────
SCENARIOS = [
    # (name,           cam_dist, pitch, yaw)
    ("downtown",        22,      -50,   42),
    ("residential",     20,      -44,   38),
    ("event",           22,      -48,   50),
    ("mixed",           20,      -44,   42),
    ("industrial",      24,      -44,   38),
]

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "screenshots")
os.makedirs(OUT_DIR, exist_ok=True)

BASE_CFG = dict(
    enable_birds=True,
    enable_trees=True,
    enable_poles=True,
    enable_houses=False,
    num_trees=12,
    num_birds=8,
)

print("\n" + "=" * 65)
print("  STIRS-2025 | Scenario Screenshots + FPS Benchmark")
print("=" * 65)

fps_table = {}

for scn, dist, pitch, yaw in SCENARIOS:
    print(f"\n[{scn.upper()}] --------------------------------------------------------")
    cfg = {**BASE_CFG, "use_scenario_system": True, "scenario": scn}

    env = DroneSurveillanceEnv(render_mode="headless", fixed_layout=True,
                               env_config=cfg)
    obs, _ = env.reset()

    # Hover drones at 3.5 m altitude for 60 steps to let them settle
    for _ in range(60):
        actions = {}
        for aid in env.agents:
            pos  = obs[aid]["position"]
            z_e  = 3.5 - pos[2]
            thr  = float(np.clip(0.3734 + z_e * 0.4, 0.1, 0.7))
            actions[aid] = np.array([thr, 0, 0, 0], dtype=np.float32)
        obs, _, _, _, _ = env.step(actions)

    # Screenshot
    rgba = capture(env.client_id, dist, pitch, yaw)
    out_path = os.path.join(OUT_DIR, f"{scn}.png")
    write_png(out_path, rgba)
    print(f"  Screenshot saved: screenshots/{scn}.png")

    # FPS benchmark: 150 steps with random actions
    for _ in range(20):   # warm-up
        env.step({aid: env.action_spaces[aid].sample() for aid in env.agents})

    N = 150
    t0 = time.perf_counter()
    for _ in range(N):
        env.step({aid: env.action_spaces[aid].sample() for aid in env.agents})
    elapsed = time.perf_counter() - t0
    fps = N / elapsed
    fps_table[scn] = fps
    print(f"  FPS (3 drones, headless, full env): {fps:.1f}")
    env.close()


# ── Downtown with drones actively flying ─────────────────────────────────────
print("\n[DOWNTOWN + DRONES FLYING] -------------------------------------------")
cfg = {**BASE_CFG, "use_scenario_system": True, "scenario": "downtown"}
env = DroneSurveillanceEnv(render_mode="headless", fixed_layout=False,
                            env_config=cfg)
obs, _ = env.reset()

# Fly drones in a spread formation at ~4 m altitude
for step in range(120):
    actions = {}
    t = step * 0.05
    for i, aid in enumerate(env.agents):
        pos = obs[aid]["position"]
        z_e = 4.5 - pos[2]
        thr = float(np.clip(0.3734 + z_e * 0.38, 0.1, 0.75))
        pitch = float(0.15 * np.sin(t + i * 2.09))
        roll  = float(0.10 * np.cos(t + i * 2.09))
        actions[aid] = np.array([thr, pitch, roll, 0], dtype=np.float32)
    obs, _, _, _, _ = env.step(actions)

# Dramatic lower angle with drones visible in scene
rgba = capture(env.client_id,
               dist=18, pitch=-32, yaw=55, target=(0, 0, 3.5),
               width=1280, height=720)
write_png(os.path.join(OUT_DIR, "downtown_drones_flying.png"), rgba)
print("  Screenshot saved: screenshots/downtown_drones_flying.png")
env.close()


# ── 5-drone load estimate ─────────────────────────────────────────────────────
print("\n[5-DRONE LOAD ESTIMATE] ----------------------------------------------")
cfg5 = {**BASE_CFG, "use_scenario_system": True, "scenario": "downtown"}
env5 = DroneSurveillanceEnv(render_mode="headless", fixed_layout=True,
                             env_config=cfg5)
obs5, _ = env5.reset()

# Warm-up
for _ in range(20):
    env5.step({aid: env5.action_spaces[aid].sample() for aid in env5.agents})

N = 150
t0 = time.perf_counter()
for _ in range(N):
    actions5 = {aid: env5.action_spaces[aid].sample() for aid in env5.agents}
    obs5, _, _, _, _ = env5.step(actions5)
    # Simulate 2 extra LiDAR scans (for 2 additional drones beyond the 3)
    env5._get_drone_sensors("drone_0")
    env5._get_drone_sensors("drone_1")
elapsed5 = time.perf_counter() - t0
fps5 = N / elapsed5
fps_table["downtown_5drones_est"] = fps5
print(f"  FPS (5-drone load estimate, downtown): {fps5:.1f}")
env5.close()


# ── Summary ───────────────────────────────────────────────────────────────────
print("\n" + "=" * 65)
print("  FPS BENCHMARK SUMMARY")
print("=" * 65)
print(f"  {'Scenario':<24}  {'FPS':>7}  {'Target':>8}  Status")
print(f"  {'-'*24}  {'-'*7}  {'-'*8}  ------")
for scn, fps in fps_table.items():
    target = 20 if "5drone" in scn else 30
    label  = ">= 20" if "5drone" in scn else ">= 30"
    flag   = "PASS" if fps >= target else "WARN"
    print(f"  {scn:<24}  {fps:7.1f}  {label:>8}  {flag}")

print("")
print("  Targets: >=30 FPS for 3 drones | >=20 FPS for 5 drones")
print("  All screenshots saved to: screenshots/")
print("=" * 65)
