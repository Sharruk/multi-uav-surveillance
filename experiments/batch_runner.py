"""
BatchRunner — systematic multi-experiment, multi-algorithm, multi-run testing.

Runs every (experiment × algorithm × run) combination and writes structured
output to results/<experiment>_<algorithm>_run<N>/:

    metrics.csv        — per-step FPS, crowd, bird, collision data
    trajectories.csv   — per-drone-per-step position and action
    collisions.csv     — collision events
    summary.csv        — episode totals
    config.yaml        — exact config used for this run
    screenshot_*.png   — 4-angle captures at step 0 and final step

Usage:
    python experiments/batch_runner.py \\
        --experiments occlusion_stress_test dense_crowd_tracking \\
        --algorithms ppo ddpg distill \\
        --runs 5 \\
        --headless \\
        --output-dir results/

    # Run all experiments with default settings:
    python experiments/batch_runner.py --all
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from experiments.experiment_presets import (
    EXPERIMENTS, get_experiment, list_experiments, print_experiment_table)


ALGORITHM_RUNNERS = {
    'ppo': ('algorithms.obstacle_avoidance.ppo_baseline',
            'run_ppo_demo'),
    'ddpg': ('algorithms.obstacle_avoidance.state_decomp_ddpg',
             'run_ddpg_demo'),
    'distill': ('algorithms.obstacle_avoidance.attention_distill',
                'run_distill_demo'),
}


def run_single(exp_name: str, algo_name: str, run_id: int,
               output_dir: str, headless: bool, screenshot: bool) -> dict:
    """
    Execute one (experiment, algorithm, run) triple.

    Returns a result dict with keys: success, fps_avg, collisions, path.
    """
    import numpy as np
    from envs.drone_env import DroneSurveillanceEnv
    from envs.metrics.data_logger import DataLogger
    from utils.screenshot_tool import ScreenshotTool

    exp  = get_experiment(exp_name)
    cfg  = dict(exp['env_config'])
    steps = exp['duration_steps']

    run_label = f"{exp_name}_{algo_name}_run{run_id:02d}"
    run_dir   = os.path.join(output_dir, run_label)
    os.makedirs(run_dir, exist_ok=True)

    if headless:
        os.environ['HEADLESS'] = '1'

    render = 'headless' if headless else 'human'
    env = DroneSurveillanceEnv(render_mode=render, fixed_layout=True,
                                env_config=cfg)
    obs, _ = env.reset()

    logger = DataLogger(run_label, output_dir=output_dir)

    # Initial screenshots
    if screenshot:
        tool = ScreenshotTool(output_dir=run_dir)
        tool.capture_scenario(env.client_id, exp['scenario'])

    # Write config snapshot
    _write_config_snapshot(run_dir, exp_name, algo_name, run_id, cfg, steps)

    fps_samples = []
    for step in range(steps):
        actions = {aid: env.action_spaces[aid].sample() for aid in env.agents}
        t0 = time.perf_counter()
        obs, rews, terms, truncs, info = env.step(actions)
        fps = 1.0 / max(time.perf_counter() - t0, 1e-9)
        fps_samples.append(fps)
        logger.log_step(env, obs, actions, fps)

        if all(terms.values()) or all(truncs.values()):
            break

    logger.finalize()
    env.close()

    result = {
        'success':    True,
        'fps_avg':    round(sum(fps_samples) / max(len(fps_samples), 1), 2),
        'collisions': logger._collision_count,
        'path':       run_dir,
    }
    return result


def run_suite(experiments: list, algorithms: list, num_runs: int,
              output_dir: str, headless: bool, screenshot: bool):
    """Run the full (experiment × algorithm × run) matrix."""
    total = len(experiments) * len(algorithms) * num_runs
    done  = 0

    print(f"\n{'='*62}")
    print(f"  STIRS-2025 Batch Runner")
    print(f"  Experiments: {experiments}")
    print(f"  Algorithms:  {algorithms}")
    print(f"  Runs each:   {num_runs}   Total: {total}")
    print(f"{'='*62}\n")

    results = {}
    for exp_name in experiments:
        for algo_name in algorithms:
            for run_id in range(1, num_runs + 1):
                done += 1
                label = f"{exp_name} + {algo_name} (run {run_id}/{num_runs})"
                print(f"[{done:>3}/{total}] {label}")
                t_start = time.perf_counter()
                try:
                    r = run_single(exp_name, algo_name, run_id,
                                   output_dir, headless, screenshot)
                    elapsed = time.perf_counter() - t_start
                    print(f"         FPS avg={r['fps_avg']}  "
                          f"collisions={r['collisions']}  "
                          f"wall={elapsed:.1f}s  → {r['path']}")
                    results[f"{exp_name}_{algo_name}_{run_id}"] = r
                except Exception as e:
                    print(f"         FAILED: {e}")
                    results[f"{exp_name}_{algo_name}_{run_id}"] = {
                        'success': False, 'error': str(e)}

    _print_summary(results, experiments, algorithms, num_runs)
    return results


def _print_summary(results: dict, experiments, algorithms, num_runs):
    print(f"\n{'='*62}")
    print("  BATCH SUMMARY")
    print(f"{'='*62}")
    for exp_name in experiments:
        for algo_name in algorithms:
            fps_vals = []
            failures = 0
            for run_id in range(1, num_runs + 1):
                r = results.get(f"{exp_name}_{algo_name}_{run_id}", {})
                if r.get('success'):
                    fps_vals.append(r['fps_avg'])
                else:
                    failures += 1
            if fps_vals:
                avg_fps = round(sum(fps_vals) / len(fps_vals), 1)
                status  = 'PASS' if avg_fps >= 100 else 'WARN'
                print(f"  {exp_name:<30} {algo_name:<8} "
                      f"fps={avg_fps}  failures={failures}  [{status}]")
            else:
                print(f"  {exp_name:<30} {algo_name:<8} ALL FAILED")
    print(f"{'='*62}\n")


def _write_config_snapshot(run_dir, exp_name, algo_name, run_id, cfg, steps):
    """Write a minimal YAML-style config record for reproducibility."""
    path = os.path.join(run_dir, "config.yaml")
    with open(path, 'w') as f:
        f.write(f"experiment: {exp_name}\n")
        f.write(f"algorithm: {algo_name}\n")
        f.write(f"run_id: {run_id}\n")
        f.write(f"duration_steps: {steps}\n")
        f.write("env_config:\n")
        for k, v in cfg.items():
            f.write(f"  {k}: {v}\n")


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(
        description='STIRS-2025 Batch Experiment Runner')
    p.add_argument('--experiments', nargs='+', default=None,
                   help='Experiment names (default: all)')
    p.add_argument('--all', action='store_true',
                   help='Run all experiments')
    p.add_argument('--algorithms', nargs='+',
                   default=['ppo', 'ddpg', 'distill'],
                   choices=['ppo', 'ddpg', 'distill'],
                   help='Algorithms to test (default: all three)')
    p.add_argument('--runs', type=int, default=None,
                   help='Override default num_runs for all experiments')
    p.add_argument('--headless', action='store_true',
                   help='Run without PyBullet GUI window')
    p.add_argument('--screenshot', action='store_true',
                   help='Capture 4-angle screenshots per run')
    p.add_argument('--output-dir', default='results/',
                   help='Root output directory (default: results/)')
    p.add_argument('--list', action='store_true',
                   help='List available experiments and exit')
    return p.parse_args()


if __name__ == '__main__':
    args = _parse_args()

    if args.list:
        print_experiment_table()
        sys.exit(0)

    if args.all or args.experiments is None:
        exps = list_experiments()
    else:
        exps = args.experiments

    # Validate experiment names
    for name in exps:
        if name not in EXPERIMENTS:
            print(f"[ERROR] Unknown experiment: '{name}'")
            print(f"Available: {list_experiments()}")
            sys.exit(1)

    # Determine run counts
    if args.runs is not None:
        for name in exps:
            EXPERIMENTS[name]['num_runs'] = args.runs
    num_runs = args.runs or max(EXPERIMENTS[n]['num_runs'] for n in exps)

    run_suite(
        experiments=exps,
        algorithms=args.algorithms,
        num_runs=num_runs,
        output_dir=args.output_dir,
        headless=args.headless,
        screenshot=args.screenshot,
    )
