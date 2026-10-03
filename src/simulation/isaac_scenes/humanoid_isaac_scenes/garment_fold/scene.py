"""Teleop registration for the garment-fold scene.

The scene (worksurface + cameras) and the garment's own post-init builder
live with the task (`humanoid_garment_fold.tasks.teleop_scene`) -- single
source of truth, same pattern as `push_block`. Here we just register it
under the ``garment_fold`` name so ``keyboard_teleop --scene garment_fold``
can pull it in with the arm plugged into its ``MISSING`` robot slot.

Requires the ``humanoid_garment_fold`` package installed (``pip install -e
src/simulation/garment_fold_task``) -- if it isn't, this import fails and
``_discover()`` just skips registering ``garment_fold`` (every other scene
still works, see ``humanoid_isaac_scenes/_register.py``).
"""
from __future__ import annotations

from humanoid_isaac_scenes import scene
from humanoid_garment_fold.tasks.teleop_scene import (
    GarmentFoldSceneCfg,
    ROBOT_BASE_POS,
    ROBOT_BASE_ROT,
    post_init,
)

# Same base pose GarmentPioneerEnvCfg uses (reach-tested, see that file).
scene(
    "garment_fold",
    robot_pos=ROBOT_BASE_POS,
    robot_rot=ROBOT_BASE_ROT,
    camera=([1.8, -2.0, 1.3], [0.0, -0.1, 0.55]),
    post_init=post_init,
)(GarmentFoldSceneCfg)
