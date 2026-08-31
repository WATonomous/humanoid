"""Import bridge for the pioneer_bimanual_arm config and pick_place_gen modules.

The authoritative robot config lives outside this task package's import tree
("Teleop/keyboard_based_teleoperation"), so it is imported as a top-level
module by putting that directory on sys.path — the same pattern as
quest_isaac_teleop/run_quest_bimanual_teleop.py. This module is the ONLY place
in the task package that touches those paths.
"""
import sys
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parents[6]  # .../autonomy/simulation
assert (_SIM_DIR / "Humanoid_Wato").is_dir(), f"unexpected repo layout at {_SIM_DIR}"

for _p in (
    _SIM_DIR / "Teleop" / "keyboard_based_teleoperation",
    _SIM_DIR / "pick_place_gen",
):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

# The pick task drives the L-suffixed chain (physical LEFT arm), which the canonical config
# names LEFT_*; aliased to RIGHT_* here so downstream "RIGHT_* = the picking arm" is unchanged.
from pioneer_bimanual_arm_cfg import (  # noqa: E402,F401
    PIONEER_BIMANUAL_ARM_CFG as BIMANUAL_ARM_CFG,
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
import task_geometry  # noqa: E402,F401
from task_params import PickPlaceTaskParams  # noqa: E402,F401

# Recorded joint order for the dataset (arm 6 + gripper pair).
RIGHT_JOINTS_ALL = RIGHT_ARM_JOINTS + RIGHT_GRIPPER_JOINTS
