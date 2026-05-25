"""
Scenario loading system for STIRS-2025 Multi-UAV Surveillance.

Usage:
    from envs.scenarios import load_scenario
    scn = load_scenario('downtown', physics_client_id, config_dict, seed=42)
    result = scn.spawn()
    building_ids = result['building_ids']
"""

import os
from typing import Optional


def _load_yaml_config(yaml_path: str) -> dict:
    """Load YAML config (graceful fallback if pyyaml not installed)."""
    try:
        import yaml
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except ImportError:
        return {}
    except FileNotFoundError:
        return {}


def load_scenario(scenario_name: str,
                  physics_client: int,
                  config: Optional[dict] = None,
                  seed: int = 42):
    """
    Instantiate and return a scenario object.

    If config is None the system tries to load config/environment_config.yaml
    relative to the project root; falls back to an empty dict on failure.
    """
    if config is None:
        root = os.path.join(os.path.dirname(__file__), '..', '..', 'config',
                            'environment_config.yaml')
        config = _load_yaml_config(os.path.abspath(root))

    # Merge per-scenario overrides from the YAML 'scenarios' key
    scenario_overrides = config.get('scenarios', {}).get(scenario_name, {})
    merged = dict(config)
    for key, val in scenario_overrides.items():
        if isinstance(val, dict) and key in merged and isinstance(merged[key], dict):
            merged[key] = {**merged[key], **val}
        else:
            merged[key] = val

    # Late import to keep module load fast
    from .downtown     import DowntownScenario
    from .residential  import ResidentialScenario
    from .event        import EventScenario
    from .mixed        import MixedUrbanScenario
    from .industrial   import IndustrialScenario

    _MAP = {
        'downtown':    DowntownScenario,
        'residential': ResidentialScenario,
        'event':       EventScenario,
        'mixed':       MixedUrbanScenario,
        'industrial':  IndustrialScenario,
    }

    cls = _MAP.get(scenario_name, DowntownScenario)
    return cls(physics_client, merged, seed)


def available_scenarios():
    return ['downtown', 'residential', 'event', 'mixed', 'industrial']
