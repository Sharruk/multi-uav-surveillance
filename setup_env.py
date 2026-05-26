# -*- coding: utf-8 -*-
"""
setup_env.py - Environment setup checker for multi-uav-surveillance

Run this script to diagnose your environment and get exact fix instructions.
Usage: python setup_env.py
"""

import sys
import os
import importlib.util

# Force UTF-8 output on Windows to avoid cp1252 emoji errors
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

OK  = "[OK] "
FAIL = "[FAIL]"
WARN = "[WARN]"

print("=" * 65)
print("  Multi-UAV Surveillance -- Environment Diagnostics")
print("=" * 65)
print(f"\nPython version: {sys.version}")
print(f"Platform: {sys.platform}")

PACKAGES = {
    "numpy":      "numpy",
    "gymnasium":  "gymnasium",
    "pybullet":   "pybullet",
    "torch":      "torch",
    "pettingzoo": "pettingzoo",
}

TRAINING_PACKAGES = {
    "ray":  "ray",
    "gym":  "gym",
}

print("\n--- Core Simulation Packages ---")
sim_ok = True
for pkg_name, import_name in PACKAGES.items():
    spec = importlib.util.find_spec(import_name)
    if spec is not None:
        try:
            mod = importlib.import_module(import_name)
            ver = getattr(mod, "__version__", "unknown")
            print(f"  {OK} {pkg_name:<15} version: {ver}")
        except Exception as e:
            print(f"  {FAIL} {pkg_name:<15} import error: {e}")
            sim_ok = False
    else:
        print(f"  {FAIL} {pkg_name:<15} NOT INSTALLED")
        sim_ok = False

print("\n--- Training Packages (Ray RLlib) ---")
for pkg_name, import_name in TRAINING_PACKAGES.items():
    spec = importlib.util.find_spec(import_name)
    if spec is not None:
        try:
            mod = importlib.import_module(import_name)
            ver = getattr(mod, "__version__", "unknown")
            print(f"  {OK} {pkg_name:<15} version: {ver}")
        except Exception as e:
            print(f"  {WARN} {pkg_name:<15} import error: {e}")
    else:
        print(f"  {WARN} {pkg_name:<15} not installed (training only)")

print("\n--- Diagnosis ---")

py_major, py_minor = sys.version_info[:2]

if py_major == 3 and py_minor >= 13:
    print(f"""
  {WARN} Python {py_major}.{py_minor} detected.

  BLOCKER 1 - PyBullet (no pre-built wheel for Python 3.13):
    pybullet must be compiled from C++ source, which requires:
    Microsoft Visual C++ Build Tools 14.0+

    FIX A (Recommended for simulation on Py 3.13):
      1. Download VS Build Tools:
         https://visualstudio.microsoft.com/visual-cpp-build-tools/
      2. Install "Desktop development with C++" workload
      3. Re-run: pip install pybullet

    FIX B (Recommended for ALL: simulation + training):
      Use Python 3.10 which has pre-built wheels for pybullet + ray[rllib]
      1. Download: https://www.python.org/downloads/release/python-31011/
      2. py -3.10 -m venv venv310
      3. .\\venv310\\Scripts\\Activate.ps1
      4. pip install numpy pybullet gymnasium torch pettingzoo ray[rllib]
      5. python drone_env.py  (simulation)
      6. python train.py      (training)

  BLOCKER 2 - Ray RLlib (no Python 3.13 support):
    Ray stable supports Python 3.9-3.12 only.
    Use Python 3.10 or 3.11 venv for training (see FIX B above).
""")
else:
    print(f"  {OK} Python {py_major}.{py_minor} is compatible with all packages.")

if sim_ok:
    print(f"  {OK} All core simulation packages installed. drone_env.py can run.")
else:
    print(f"  {FAIL} Some packages missing. See FIX instructions above.")

print("\n--- Quick Functional Tests ---")

try:
    import numpy as np
    arr = np.array([1.0, 2.0, 3.0])
    print(f"  {OK} numpy    : {arr}")
except Exception as e:
    print(f"  {FAIL} numpy    : {e}")

try:
    import gymnasium as gym
    env = gym.make("CartPole-v1")
    obs, _ = env.reset()
    env.close()
    print(f"  {OK} gymnasium: CartPole-v1 obs shape = {obs.shape}")
except Exception as e:
    print(f"  {FAIL} gymnasium: {e}")

try:
    import pybullet as p
    client = p.connect(p.DIRECT)
    p.disconnect(client)
    print(f"  {OK} pybullet : DIRECT mode connected and disconnected cleanly")
except ImportError:
    print(f"  {FAIL} pybullet : Not installed - see BLOCKER 1 above")
except Exception as e:
    print(f"  {FAIL} pybullet : {e}")

try:
    import torch
    x = torch.tensor([1.0, 2.0])
    print(f"  {OK} torch    : tensor on {x.device}, version {torch.__version__}")
except Exception as e:
    print(f"  {FAIL} torch    : {e}")

try:
    from pettingzoo.utils.env import ParallelEnv
    print(f"  {OK} pettingzoo: ParallelEnv importable")
except Exception as e:
    print(f"  {FAIL} pettingzoo: {e}")

print("\n" + "=" * 65)
print("  Run commands:")
print("  Diagnostics: .\\venv\\Scripts\\python.exe setup_env.py")
print("  Simulation:  .\\venv\\Scripts\\python.exe drone_env.py")
print("  Training:    .\\venv\\Scripts\\python.exe train.py  (needs ray)")
print("=" * 65)
