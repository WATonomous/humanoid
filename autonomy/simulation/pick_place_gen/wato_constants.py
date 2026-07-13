"""Isaac-free constants for the wato_bimanual_arm cuRobo pipeline.

Values mirror `Teleop/keyboard_based_teleoperation/bimanual_arm_cfg.py`
(Physics Inspector limits/poses, SolidWorks URDF fingertip geometry). That
file hard-imports isaaclab, so cuRobo-side scripts that run outside Isaac
Sim use this module instead. `generate_demos.py` cross-checks these values
against bimanual_arm_cfg at runtime, where both are importable.
"""
import math
import os

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SIM_DIR = os.path.join(REPO_ROOT, "autonomy", "simulation")
BIMANUAL_ROOT = os.path.join(SIM_DIR, "Humanoid_Wato", "wato_bimanual_arm")
URDF_PATH = os.path.join(BIMANUAL_ROOT, "urdf", "bimanual_arm.urdf")
# Patched copy (real joint limits) written next to the original so its
# `../meshes/...` references still resolve.
CUROBO_URDF_PATH = os.path.join(BIMANUAL_ROOT, "urdf", "bimanual_arm_curobo.urdf")
CUROBO_ROBOT_YML = os.path.join(os.path.dirname(__file__), "curobo_cfg", "wato_bimanual_left.yml")

_REVOLUTE_LIMIT = (-2 * math.pi, 2 * math.pi)
_JOINT7_LIMIT = (-0.05, 0.0)
_JOINT8_LIMIT = (0.0, 0.05)

JOINT_POS_LIMITS = {
    "joint1": _REVOLUTE_LIMIT,
    "joint2": _REVOLUTE_LIMIT,
    "joint3": _REVOLUTE_LIMIT,
    "joint4": _REVOLUTE_LIMIT,
    "joint5": _REVOLUTE_LIMIT,
    "joint6": _REVOLUTE_LIMIT,
    "joint7": _JOINT7_LIMIT,
    "joint8": _JOINT8_LIMIT,
    "joint1L": _REVOLUTE_LIMIT,
    "joint2l": _REVOLUTE_LIMIT,
    "joint3l": _REVOLUTE_LIMIT,
    "joint4l": _REVOLUTE_LIMIT,
    "joint5l": _REVOLUTE_LIMIT,
    "joint6l": _REVOLUTE_LIMIT,
    "joint7l": _JOINT7_LIMIT,
    "joint8l": _JOINT8_LIMIT,
}


def _deg(degrees: float) -> float:
    return degrees * math.pi / 180.0


DEFAULT_JOINT_POS = {
    "joint1": _deg(-140.8),
    "joint2": _deg(55.7),
    "joint3": _deg(-66.0),
    "joint4": _deg(111.4),
    "joint5": _deg(34.8),
    "joint6": _deg(3.5),
    "joint7": 0.0,
    "joint8": 0.0,
    "joint1L": _deg(139.2),
    "joint2l": _deg(66.1),
    "joint3l": _deg(147.9),
    "joint4l": _deg(-76.5),
    "joint5l": _deg(-76.5),
    "joint6l": _deg(-22.6),
    # Grippers default to OPEN=(0,0) — the shared cfg's (-0.05, +0.05)
    # crosses the finger meshes (see GRIPPER_OPEN note below).
    "joint7l": 0.0,
    "joint8l": 0.0,
}

LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
LEFT_GRIPPER_JOINTS = ["joint7l", "joint8l"]
RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]

# Joints frozen for planning: right arm + both grippers (grippers are driven
# directly as a synchronized pair, outside the planner).
LOCKED_JOINTS = {
    name: DEFAULT_JOINT_POS[name]
    for name in RIGHT_ARM_JOINTS + RIGHT_GRIPPER_JOINTS + LEFT_GRIPPER_JOINTS
}

# NOTE: intentionally INVERTED relative to bimanual_arm_cfg.py. Measured from
# the STL meshes: at (0, 0) the finger pads are 9.5 cm apart (open); at
# (-0.05, +0.05) they cross and interpenetrate. The keyboard-teleop dicts have
# the labels backwards (the pink_controller_cfg version was correct) — flagged
# upstream rather than silently edited there.
GRIPPER_OPEN = {"joint7l": 0.0, "joint8l": 0.0}
GRIPPER_CLOSED = {"joint7l": -0.05, "joint8l": 0.05}

# Finger pad geometry in the wrist (link6l) frame, from link7l/link8l STLs:
# pads hang 4.5 cm below the fingertip-center reference point, and the pad
# inner faces at OPEN are ~9.5 cm apart.
FINGER_PAD_BELOW_TIP = 0.0445
GRIPPER_OPEN_GAP = 0.095

LEFT_EE_BODY = "link6l"
LEFT_FINGER_TIP_BODIES = ("link7l", "link8l")
LEFT_FINGER_DISTAL_TIP_LOCAL = {
    "link7l": (0.13211595, -0.04057075, -0.00434997),
    "link8l": (-0.13211595, -0.04057075, -0.00435003),
}

# Fingertip-center offset expressed in the link6l (wrist) frame. Constant for
# a synchronized gripper pair (q7l = -q8l): derived from joint7l/joint8l URDF
# origins + LEFT_FINGER_DISTAL_TIP_LOCAL. Fingers extend along wrist -Y.
#   x: ((-0.016558 + 0.13211595) + (0.085558 - 0.13211595)) / 2
#   y: -0.10361 - 0.04057075
#   z: 0.004349 - 0.00434997
FINGERTIP_OFFSET_IN_WRIST = (0.0345, -0.14418075, 0.0)

# Tool approach axis in the wrist frame (wrist -> fingertips).
APPROACH_AXIS_IN_WRIST = (0.0, -1.0, 0.0)

# Reachable workspace for top-down grasps, validated by validate_curobo_plan.py
# (93.8% plan success over this box with 8 yaw candidates, table top at 0.05).
# All values in the robot base frame (== env-local frame; robot at env origin).
TABLE_TOP_Z = 0.05
WORKSPACE_X = (0.28, 0.43)
WORKSPACE_Y = (-0.32, -0.10)
HOVER_Z = (0.12, 0.22)
# Table slab (matches the cuRobo world model and the Isaac scene): front edge
# clear of the robot column at x~0.
TABLE_X_MIN = 0.18
TABLE_DIMS = (0.9, 1.2, 0.05)  # top surface at TABLE_TOP_Z
NUM_GRASP_YAWS = 8

# Tray asset (place-into-tray demo mode). tray.usda is an open-top box; its
# local origin sits at a CORNER (mesh spans local x[0,0.20] y[0,0.16] z[0,0.03]),
# so the interior centre is offset from the prim origin by TRAY_LOCAL_CENTER.
# Geometry read from the asset; scaled uniformly by PlaceParams.tray_scale.
TRAY_USDA = os.path.join(REPO_ROOT, "assets", "lerobot", "so101_vial_task", "usd", "tray.usda")
TRAY_FOOTPRINT = (0.20, 0.16)        # local x,y extent at scale 1.0 [m]
TRAY_LOCAL_CENTER = (0.10, 0.08)     # interior centre in tray-local frame [m]
TRAY_INTERIOR_HALF = (0.075, 0.055)  # interior half-extent at scale 1.0 [m]
TRAY_FLOOR_LOCAL_Z = 0.005           # inner floor height above the prim origin [m]
TRAY_WALL_HEIGHT = 0.03              # rim height at scale 1.0 [m]
TRAY_WALL_THICK = 0.004              # wall thickness at scale 1.0 [m]
