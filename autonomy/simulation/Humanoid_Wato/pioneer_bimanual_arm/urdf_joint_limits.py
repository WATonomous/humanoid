"""Joint position limits for pioneer_bimanual_arm, parsed from the URDF.

The URDF is the SINGLE SOURCE OF TRUTH for joint limits. Every consumer
(Teleop/keyboard_based_teleoperation/armWithStand_v2_cfg.py, that directory's
bimanual_arm_cfg.py, and pick_place_gen/wato_constants.py) imports from here
instead of carrying its own copy, so editing urdf/pioneer_bimanual_arm.urdf
changes the limits everywhere at once. Before this module those three files each
hardcoded the same +/-2pi placeholder, which silently overrode the real limits.

Why a shared module rather than each config parsing the URDF itself:
wato_constants.py runs OUTSIDE Isaac Sim (it feeds the cuRobo pipeline and says
so in its own docstring), so anything shared with it must import nothing beyond
the standard library. This module does exactly that -- no isaaclab, no numpy.

IMPORTANT -- how a URDF edit actually reaches the simulation. The USD under
usd/ is a separate, pre-converted artifact; nothing regenerates it when the URDF
changes, and Isaac Sim loads the USD, never the URDF. The bridge is
armWithStand_v2_cfg.patch_joint_pos_limits_on_prim() (at spawn) and
apply_joint_limits() (post-init), which write these values over the spawned USD
prims. That is what makes editing the URDF take effect without re-running
UrdfConverter -- and it is why those two functions must keep being called.
"""
import os
import xml.etree.ElementTree as ET

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
URDF_PATH = os.path.join(_THIS_DIR, "urdf", "pioneer_bimanual_arm.urdf")

# Joint types whose <limit lower/upper> describe a real position range. A
# "continuous" joint has no limits by definition and "fixed" has no DOF.
_LIMITED_JOINT_TYPES = ("revolute", "prismatic")

# Prismatic gripper travel is deliberately NOT taken from the URDF. GRIPPER_OPEN /
# GRIPPER_CLOSED in the arm configs, and the empirically tuned grasp that depends on
# them, are built around this exact stroke; the URDF says joint7/joint7l [-0.05, 0.02]
# and joint8/joint8l [-0.01, 0.05], and widening the jaw changes what the fingers do
# to the box. Delete this dict to make the URDF authoritative for the grippers too.
GRIPPER_LIMIT_OVERRIDES = {
    "joint7": (-0.05, 0.0),
    "joint8": (0.0, 0.05),
    "joint7l": (-0.05, 0.0),
    "joint8l": (0.0, 0.05),
}


def load_joint_limits(
    urdf_path: str | None = None,
    joint_types: tuple[str, ...] = _LIMITED_JOINT_TYPES,
) -> dict[str, tuple[float, float]]:
    """{joint_name: (lower, upper)} read straight from the URDF.

    Units follow URDF convention: radians for revolute, metres for prismatic.

    Joints whose limit is degenerate (lower >= upper) are SKIPPED rather than
    returned. The SolidWorks exporter writes lower=upper=0 for any joint whose
    limits were never filled in, and PhysX reads lower >= upper as "no limit",
    so propagating such a pair would silently unlock the joint instead of
    constraining it. Skipping means the caller keeps whatever the asset already
    had, and a missing key is visible rather than quietly wrong.
    """
    root = ET.parse(urdf_path or URDF_PATH).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") not in joint_types:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        if lower >= upper:
            continue
        limits[joint.get("name")] = (lower, upper)
    return limits


# The table every consumer imports: URDF limits, with the gripper stroke pinned.
JOINT_POS_LIMITS = {**load_joint_limits(), **GRIPPER_LIMIT_OVERRIDES}
