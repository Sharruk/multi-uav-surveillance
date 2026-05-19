"""
main.py — Smart City Multi-UAV Crowd Surveillance
===================================================
Primary entry point for the research simulation.

Usage:
    python main.py

Controls:
    SPACE  → Reset simulation
    P      → Pause / Resume
    ESC    → Quit
"""

from simulation.runner import SimulationRunner


def main() -> None:
    runner = SimulationRunner()
    runner.run()


if __name__ == "__main__":
    main()
