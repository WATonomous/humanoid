"""Robot-config + datagen re-exports for the pick_place task.

`pioneer_humanoid` is editable-installed in the isaac_lab image; `datagen/` is a
sub-package here. No `sys.path` juggling needed.
"""
# The pick task drives the L-suffixed chain (physical LEFT arm), which the canonical config
# names LEFT_*; aliased to RIGHT_* here so downstream "RIGHT_* = the picking arm" is unchanged.
from pioneer_humanoid.bimanual_arm import (  # noqa: F401
    BIMANUAL_ARM_CFG,
    LEFT_GRIPPER_CLOSED as GRIPPER_CLOSED,
    LEFT_GRIPPER_OPEN as GRIPPER_OPEN,
    LEFT_ARM_JOINTS as RIGHT_ARM_JOINTS,
    LEFT_EE_BODY as RIGHT_EE_BODY,
    LEFT_FINGER_TIP_BODIES as RIGHT_FINGER_TIP_BODIES,
    LEFT_GRIPPER_JOINTS as RIGHT_GRIPPER_JOINTS,
    compute_gripper_tip_pose_b,
    compute_gripper_tip_pose_w,
    compute_tip_ik_jacobian,
    resolve_body_ids,
)

from .datagen import task_geometry  # noqa: F401
from .datagen.task_params import PickPlaceTaskParams  # noqa: F401

# Recorded joint order for the dataset (arm 6 + gripper pair).
RIGHT_JOINTS_ALL = RIGHT_ARM_JOINTS + RIGHT_GRIPPER_JOINTS

# pick_place_env_cfg.py refers to this by its physical-arm name; the rest of the
# task uses the RIGHT_* alias (see the picking-arm note above). Keep both.
LEFT_JOINTS_ALL = RIGHT_JOINTS_ALL
