import os
import sys
import argparse
import tkinter as tk
from tkinter import ttk

# ── CLI flags ─────────────────────────────────────────────────────────────────
_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument('--scenario', default=None,
                     choices=['downtown', 'residential', 'event', 'mixed', 'industrial'],
                     help='Directly launch a specific urban scenario (skips GUI)')
_parser.add_argument('--algorithm', default=None,
                     choices=['ppo', 'ddpg', 'distill'],
                     help='Algorithm shorthand when used with --scenario')
_CLI, _ = _parser.parse_known_args()


SCENARIO_LABELS = {
    'downtown':    'Downtown Surveillance  (high-rise, dense crowd)',
    'residential': 'Residential Monitoring  (parks, low-rise, parking)',
    'event':       'Event Crowd Control  (plaza, very dense gathering)',
    'mixed':       'Mixed Urban Area  (varied heights, general testing)',
    'industrial':  'Industrial / Port Zone  (warehouses, sparse crowd)',
}
SCENARIO_KEYS = list(SCENARIO_LABELS.keys())


def center_window(window, width=500, height=320):
    """Centers the Tkinter window on the screen."""
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()
    x = (screen_width // 2) - (width // 2)
    y = (screen_height // 2) - (height // 2)
    window.geometry(f"{width}x{height}+{x}+{y}")

def run_selector():
    # Setup Tkinter root window
    root = tk.Tk()
    root.title("STIRS-2025: Swarm Policy Selector")
    center_window(root, 560, 420)
    
    # Premium Dark Palette Colors
    BG_DARK = "#1E1E24"
    CARD_BG = "#2A2B36"
    TEXT_LIGHT = "#F4F7F6"
    ACCENT_CYAN = "#00F0FF"
    ACCENT_HOVER = "#00C2CF"
    TEXT_MUTED = "#8E92A2"
    
    root.configure(bg=BG_DARK)
    root.resizable(False, False)
    
    # Custom protocol to exit completely if user closes the window
    def on_close():
        root.quit()
        root.destroy()
        sys.exit(0)
    root.protocol("WM_DELETE_WINDOW", on_close)
    
    # Header Frame
    header_frame = tk.Frame(root, bg=BG_DARK, pady=15)
    header_frame.pack(fill=tk.X)
    
    title_label = tk.Label(
        header_frame, 
        text="STIRS-2025 MULTI-UAV TESTBED", 
        font=("Helvetica", 14, "bold"), 
        bg=BG_DARK, 
        fg=ACCENT_CYAN
    )
    title_label.pack()
    
    subtitle_label = tk.Label(
        header_frame, 
        text="Decentralized Surveillance & Swarm Coordination Testbed", 
        font=("Helvetica", 9, "italic"), 
        bg=BG_DARK, 
        fg=TEXT_MUTED
    )
    subtitle_label.pack(pady=2)
    
    # Main Selector Frame (Card Style)
    card_frame = tk.Frame(root, bg=CARD_BG, bd=1, relief=tk.FLAT, padx=25, pady=20)
    card_frame.pack(padx=20, pady=10, fill=tk.BOTH, expand=True)
    
    # ── ttk style ──────────────────────────────────────────────────────────
    style = ttk.Style()
    style.theme_use('default')
    style.configure("TCombobox", fieldbackground=BG_DARK, background=CARD_BG,
                    foreground="black", font=("Helvetica", 10))

    # ── Algorithm selector ──────────────────────────────────────────────────
    algo_label = tk.Label(card_frame, text="Select Swarm Control Architecture:",
                          font=("Helvetica", 11, "bold"), bg=CARD_BG, fg=TEXT_LIGHT)
    algo_label.pack(anchor=tk.W, pady=(0, 4))

    algo_options = [
        "Multi-Agent PPO (Shared Policy Baseline)",
        "State-Decomposition DDPG (SDDPG-NAV)",
        "Attention-Based Policy Distillation"
    ]
    combobox = ttk.Combobox(card_frame, values=algo_options, state="readonly",
                             width=50, font=("Helvetica", 10))
    combobox.set(algo_options[0])
    combobox.pack(pady=(0, 12), fill=tk.X)

    # ── Scenario selector ───────────────────────────────────────────────────
    scn_label = tk.Label(card_frame, text="Select Urban Scenario Environment:",
                         font=("Helvetica", 11, "bold"), bg=CARD_BG, fg=TEXT_LIGHT)
    scn_label.pack(anchor=tk.W, pady=(0, 4))

    scn_options = [SCENARIO_LABELS[k] for k in SCENARIO_KEYS]
    scn_combo = ttk.Combobox(card_frame, values=scn_options, state="readonly",
                              width=50, font=("Helvetica", 10))
    scn_combo.set(scn_options[0])   # Default: Downtown
    scn_combo.pack(pady=(0, 4), fill=tk.X)

    # Scenario description hint
    scn_hint = tk.Label(card_frame,
                        text="Tip: Each scenario uses a procedurally generated city with realistic buildings.",
                        font=("Helvetica", 8, "italic"), bg=CARD_BG, fg=TEXT_MUTED)
    scn_hint.pack(anchor=tk.W, pady=(0, 6))

    selected_option = {"value": None, "scenario": SCENARIO_KEYS[0]}
    
    # Button callback
    def on_deploy():
        selected_option["value"] = combobox.get()
        # Map scenario label back to key
        scn_lbl = scn_combo.get()
        for k, lbl in SCENARIO_LABELS.items():
            if lbl == scn_lbl:
                selected_option["scenario"] = k
                break
        root.quit()
        root.destroy()
        
    # Premium styled Deploy Button
    deploy_btn = tk.Button(
        card_frame,
        text="DEPLOY SWARM SIMULATION",
        font=("Helvetica", 11, "bold"),
        bg=ACCENT_CYAN,
        fg=BG_DARK,
        activebackground=ACCENT_HOVER,
        activeforeground=BG_DARK,
        relief=tk.FLAT,
        cursor="hand2",
        bd=0,
        pady=8,
        command=on_deploy
    )
    deploy_btn.pack(fill=tk.X, pady=(15, 0))
    
    # Add simple button hover effect
    def on_enter(e):
        deploy_btn.configure(bg=ACCENT_HOVER)
    def on_leave(e):
        deploy_btn.configure(bg=ACCENT_CYAN)
        
    deploy_btn.bind("<Enter>", on_enter)
    deploy_btn.bind("<Leave>", on_leave)
    
    # Start blocking GUI loop
    root.mainloop()
    
    # Execute the selected simulation
    choice   = selected_option["value"]
    scenario = selected_option["scenario"]
    _inject_scenario(scenario)   # set env var before algorithm imports env

    if choice == "Multi-Agent PPO (Shared Policy Baseline)":
        from algorithms.obstacle_avoidance.ppo_baseline import run_ppo_demo
        run_ppo_demo()
    elif choice == "State-Decomposition DDPG (SDDPG-NAV)":
        from algorithms.obstacle_avoidance.state_decomp_ddpg import run_ddpg_demo
        run_ddpg_demo()
    elif choice == "Attention-Based Policy Distillation":
        from algorithms.obstacle_avoidance.attention_distill import run_distill_demo
        run_distill_demo()
    else:
        print("[ERROR] No valid selection made. Exiting.")

def _inject_scenario(scenario_name: str):
    """
    Pass the chosen scenario to the algorithm demo environments via env var.
    Each algorithm's run_*_demo() calls DroneSurveillanceEnv internally;
    the env var is read in _build_demo_env_config() below, which algorithms
    can call, or directly via env_config when they construct the environment.
    """
    os.environ['STIRS_SCENARIO'] = scenario_name
    os.environ['STIRS_USE_SCENARIO_SYSTEM'] = '1'


def build_scenario_env_config(base_config: dict = None) -> dict:
    """
    Helper for algorithm modules to pick up the scenario chosen in the GUI.
    Merges scenario settings into a drone_env env_config dict.

    Usage in ppo_baseline.py / state_decomp_ddpg.py / attention_distill.py:
        from main_selector import build_scenario_env_config
        env_cfg = build_scenario_env_config()
        env = DroneSurveillanceEnv(..., env_config=env_cfg)
    """
    cfg = dict(base_config) if base_config else {}
    scenario = os.environ.get('STIRS_SCENARIO', 'downtown')
    use_sys  = os.environ.get('STIRS_USE_SCENARIO_SYSTEM', '0') == '1'
    cfg['use_scenario_system'] = use_sys
    cfg['scenario']            = scenario
    return cfg


if __name__ == "__main__":
    # Direct --scenario launch (skip GUI)
    if _CLI.scenario:
        algo = _CLI.algorithm or 'ppo'
        _inject_scenario(_CLI.scenario)
        print(f"[STIRS] Launching scenario={_CLI.scenario}  algorithm={algo}")
        if algo == 'ddpg':
            from algorithms.obstacle_avoidance.state_decomp_ddpg import run_ddpg_demo
            run_ddpg_demo()
        elif algo == 'distill':
            from algorithms.obstacle_avoidance.attention_distill import run_distill_demo
            run_distill_demo()
        else:
            from algorithms.obstacle_avoidance.ppo_baseline import run_ppo_demo
            run_ppo_demo()
    else:
        run_selector()
