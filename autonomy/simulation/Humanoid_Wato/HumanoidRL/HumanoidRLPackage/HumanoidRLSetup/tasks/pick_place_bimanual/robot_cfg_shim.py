"""Import bridge for the wato_bimanual_arm configs and pick_place_gen modules.

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

from bimanual_arm_cfg import (  # noqa: E402,F401
    BIMANUAL_ARM_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_FINGER_TIP_BODIES,
    LEFT_GRIPPER_JOINTS,
    compute_gripper_tip_pose_b,
    compute_gripper_tip_pose_w,
    resolve_body_ids,
)
import wato_constants  # noqa: E402,F401
from task_params import PickPlaceTaskParams  # noqa: E402,F401

# Recorded joint order for the dataset (arm 6 + gripper pair).
LEFT_JOINTS_ALL = LEFT_ARM_JOINTS + LEFT_GRIPPER_JOINTS


def check_constants_consistency() -> None:
    """Assert the Isaac-free wato_constants mirror matches bimanual_arm_cfg.

    Gripper open/closed and gripper defaults are intentionally EXCLUDED:
    measured finger STL geometry shows bimanual_arm_cfg's labels are inverted
    (its GRIPPER_OPEN crosses the finger meshes) — see wato_constants.py.
    """
    import math

    import bimanual_arm_cfg as bac

    assert wato_constants.LEFT_EE_BODY == bac.LEFT_EE_BODY
    assert wato_constants.LEFT_FINGER_DISTAL_TIP_LOCAL == bac.LEFT_FINGER_DISTAL_TIP_LOCAL
    # the upstream dicts are the inversion of ours (both files agree on values)
    assert wato_constants.GRIPPER_OPEN == bac.GRIPPER_CLOSED
    for name, (lo, hi) in bac.JOINT_POS_LIMITS.items():
        wlo, whi = wato_constants.JOINT_POS_LIMITS[name]
        assert math.isclose(lo, wlo) and math.isclose(hi, whi), name
    gripper = {"joint7", "joint8", "joint7l", "joint8l"}
    for name, val in bac._DEFAULT_JOINT_POS.items():
        if name not in gripper:
            assert math.isclose(val, wato_constants.DEFAULT_JOINT_POS[name]), name
