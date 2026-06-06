"""Dextrah Kuka-Allegro task registration.

Env implementation, assets, distillation, and deploy live in ``dextrah_rgb/``.
This module only registers the gym env so HumanoidRL ``train.py`` / ``play.py`` work
without wrapper scripts.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SIMULATION_ROOT = Path(__file__).resolve().parents[6]

if _SIMULATION_ROOT.is_dir():
    simulation_path = str(_SIMULATION_ROOT)
    if simulation_path not in sys.path:
        sys.path.insert(0, simulation_path)

import dextrah_rgb.tasks.dextrah_kuka_allegro  # noqa: F401
