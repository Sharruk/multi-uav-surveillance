import os
import sys
import tkinter as tk
from tkinter import ttk

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
    center_window(root, 520, 320)
    
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
    
    prompt_label = tk.Label(
        card_frame, 
        text="Select Swarm Control Architecture:", 
        font=("Helvetica", 11, "bold"), 
        bg=CARD_BG, 
        fg=TEXT_LIGHT
    )
    prompt_label.pack(anchor=tk.W, pady=(0, 5))
    
    # Dropdown combobox choices
    options = [
        "Multi-Agent PPO (Shared Policy Baseline)",
        "State-Decomposition DDPG (SDDPG-NAV)",
        "Attention-Based Policy Distillation"
    ]
    
    # ttk styling for combobox to match dark theme partially
    style = ttk.Style()
    style.theme_use('default')
    style.configure(
        "TCombobox", 
        fieldbackground=BG_DARK, 
        background=CARD_BG, 
        foreground="black",
        font=("Helvetica", 10)
    )
    
    combobox = ttk.Combobox(
        card_frame, 
        values=options, 
        state="readonly", 
        width=45, 
        font=("Helvetica", 11)
    )
    combobox.set(options[0]) # Default to PPO Baseline
    combobox.pack(pady=10, fill=tk.X)
    
    selected_option = {"value": None}
    
    # Button callback
    def on_deploy():
        selected_option["value"] = combobox.get()
        # Cleanly stop and destroy the Tkinter GUI to prevent any PyBullet context locks
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
    choice = selected_option["value"]
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

if __name__ == "__main__":
    run_selector()
