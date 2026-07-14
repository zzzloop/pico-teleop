#!/usr/bin/env python3
"""Isaac Lab launcher that makes the workspace package importable."""

from __future__ import annotations

import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from pico_isaaclab.sim_app import main, simulation_app  # noqa: E402


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

