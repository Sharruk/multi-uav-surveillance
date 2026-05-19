"""
drone_simulation.py — backward-compatibility shim
===================================================
This file is kept so that anyone who runs the original command
    python drone_simulation.py
still gets the full simulation.

All logic has been moved to the `simulation/` package.
The new canonical entry point is:
    python main.py
"""

from simulation.runner import SimulationRunner


def main() -> None:
    runner = SimulationRunner()
    runner.run()


if __name__ == "__main__":
    main()
