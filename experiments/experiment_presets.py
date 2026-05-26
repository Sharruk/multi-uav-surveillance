"""
Predefined research experiment configurations for STIRS-2025.

Each entry in EXPERIMENTS is a self-contained experiment specification that
can be passed to batch_runner.run_single_experiment() or used standalone.

Keys in each config:
  scenario       — one of the five standard scenario names
  description    — one-line summary of the experiment's research goal
  env_config     — dict passed directly to DroneSurveillanceEnv(env_config=...)
  duration_steps — number of env.step() calls per run
  num_runs       — default repetition count for statistical significance

Usage:
    from experiments.experiment_presets import EXPERIMENTS, get_experiment

    cfg = get_experiment("baseline_comparison")
    print(cfg["description"])
    print(cfg["env_config"])
"""

# Shared base config applied to all experiments
_BASE = dict(
    use_scenario_system=True,
    enable_trees=True,
    enable_poles=True,
    enable_houses=False,
    num_trees=12,
)


def _cfg(scenario, birds, bird_count, wind, wind_speed, crowd_override=None,
         trees=True, poles=True):
    """Build an env_config dict for an experiment."""
    c = dict(_BASE)
    c['scenario']           = scenario
    c['enable_birds']       = birds
    c['num_birds']          = bird_count
    c['enable_wind_physics'] = wind
    c['wind_base_speed']    = wind_speed
    c['enable_trees']       = trees
    c['enable_poles']       = poles
    if crowd_override is not None:
        c['crowd_override'] = crowd_override   # picked up by drone_env if wired
    return c


EXPERIMENTS = {

    # ── 1. Occlusion Stress Test ──────────────────────────────────────────────
    'occlusion_stress_test': {
        'scenario':       'downtown',
        'description':    ('Test UAV coordination in a high-occlusion environment '
                           'with tall buildings and street canyons. No birds or '
                           'wind so only architectural occlusion matters.'),
        'env_config':     _cfg('downtown', birds=False, bird_count=0,
                               wind=False, wind_speed=0.0),
        'duration_steps': 1000,
        'num_runs':       5,
        'tags':           ['occlusion', 'downtown', 'navigation'],
    },

    # ── 2. Dense Crowd Tracking ───────────────────────────────────────────────
    'dense_crowd_tracking': {
        'scenario':       'event',
        'description':    ('Track a 70-agent dense gathering in the event plaza '
                           'with wind and birds active. Tests crowd density '
                           'estimation and multi-target tracking under maximum '
                           'environmental challenge.'),
        'env_config':     _cfg('event', birds=True, bird_count=10,
                               wind=True, wind_speed=2.0),
        'duration_steps': 2000,
        'num_runs':       5,
        'tags':           ['crowd', 'event', 'tracking', 'full-challenge'],
    },

    # ── 3. Wind Resilience Test ───────────────────────────────────────────────
    'wind_resilience_test': {
        'scenario':       'mixed',
        'description':    ('Test drone formation stability and path tracking '
                           'under strong wind (4.5 m/s). No birds to isolate '
                           'the wind disturbance effect on control policies.'),
        'env_config':     _cfg('mixed', birds=False, bird_count=0,
                               wind=True, wind_speed=4.5),
        'duration_steps': 1500,
        'num_runs':       5,
        'tags':           ['wind', 'mixed', 'formation', 'resilience'],
    },

    # ── 4. Collision Avoidance Gauntlet ───────────────────────────────────────
    'collision_avoidance_gauntlet': {
        'scenario':       'industrial',
        'description':    ('Maximum obstacle density: 15 birds, strong wind, '
                           'and industrial container stacks. Tests collision '
                           'avoidance at the upper limit of obstacle count.'),
        'env_config':     _cfg('industrial', birds=True, bird_count=15,
                               wind=True, wind_speed=3.0),
        'duration_steps': 2000,
        'num_runs':       3,
        'tags':           ['collision', 'industrial', 'birds', 'stress'],
    },

    # ── 5. Baseline Comparison ────────────────────────────────────────────────
    'baseline_comparison': {
        'scenario':       'residential',
        'description':    ('Clean baseline for algorithm benchmarking. Sparse '
                           'crowd, no birds, no wind. Minimum environmental '
                           'noise so algorithm differences dominate results.'),
        'env_config':     _cfg('residential', birds=False, bird_count=0,
                               wind=False, wind_speed=0.0,
                               trees=False, poles=False),
        'duration_steps': 1000,
        'num_runs':       10,
        'tags':           ['baseline', 'residential', 'clean'],
    },
}


def get_experiment(name: str) -> dict:
    """Return experiment config by name. Raises KeyError if not found."""
    if name not in EXPERIMENTS:
        available = ', '.join(EXPERIMENTS.keys())
        raise KeyError(f"Unknown experiment '{name}'. Available: {available}")
    return EXPERIMENTS[name]


def list_experiments() -> list:
    """Return all experiment names."""
    return list(EXPERIMENTS.keys())


def print_experiment_table():
    """Print a human-readable summary table of all experiments."""
    header = f"{'Name':<35} {'Scenario':<14} {'Steps':>6} {'Runs':>5}  Tags"
    print(header)
    print('-' * len(header))
    for name, cfg in EXPERIMENTS.items():
        tags = ', '.join(cfg.get('tags', []))
        print(f"{name:<35} {cfg['scenario']:<14} "
              f"{cfg['duration_steps']:>6} {cfg['num_runs']:>5}  {tags}")
