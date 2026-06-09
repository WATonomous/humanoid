"""Paths to SO101 vial-to-rack task USD assets under assets/lerobot/."""

import os

_TELEOP_DIR = os.path.abspath(os.path.dirname(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_TELEOP_DIR, "..", "..", "..", ".."))
VIAL_TASK_USD_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task", "usd")

LIGHTBOX_USD = os.path.join(VIAL_TASK_USD_DIR, "lightbox-simple.usd")
MAT_USDA = os.path.join(VIAL_TASK_USD_DIR, "mat.usda")
VIAL_OPAQUE_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_opaque.usda")
VIAL_RACK_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_rack_simple.usda")

VIAL_SPAWN_Z = 0.05
VIAL_SPAWNS = (
    (0.23, -0.08, VIAL_SPAWN_Z),
    (0.23, 0.0, VIAL_SPAWN_Z),
    (0.23, -0.16, VIAL_SPAWN_Z),
)
RACK_SPAWN = (0.18, 0.08, 0.06)
MAT_SPAWN_POS = (0.22, 0.0, 0.032)
LIGHTBOX_SPAWN_POS = (-0.1, 0.0, 0.0257)
