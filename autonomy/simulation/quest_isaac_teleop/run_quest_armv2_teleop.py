"""Quest armWithStand (wato_arm_v2) teleoperation — runs inside the
simulation_isaac container.

This is run_quest_bimanual_teleop.py with the config import repointed at
armWithStand_v2_cfg.ARM_V2_CFG (armWithStand.usd) instead of
bimanual_arm_cfg.BIMANUAL_ARM_CFG (bimanual_arm.usd). Camera setup: kept
bimanual's dynamic head-tracked stereo RSD455 pair (real depth via a 63mm
IPD baseline, not a mirrored monocular feed) for the teleop headset display,
per explicit request -- data-collection cameras (ego_cam/wrist_cam baked
into armWithStand.usd's sensor layer) are a SEPARATE concern, not routed to
the headset here.

_HEAD_VIEWPOINT_HOME_POS/QUAT below are RESET to bimanual's original values
(see the constant's own comment) -- multiple independent attempts at an
armWithStand-specific head pose were each visibly wrong when tested live,
so this resets to a known baseline for direct user observation/feedback
instead of another guess.

See run_quest_bimanual_teleop.py's own module docstring below for the full
IK/coordinate-mapping explanation, all of which applies unchanged here.

Both arms are controlled via Quest hand tracking. Hand pose data arrives via
ROS 2 /quest_teleop topic published by quest_teleop_node (also in this
container). The left Quest wrist drives the left arm, the right Quest wrist
drives the right arm. Pinching thumb + index closes the corresponding
gripper.

IK solver: isaaclab.controllers.DifferentialIKController (ik_method="dls",
damped least squares), tracking each gripper's fingertip-tip-center frame —
the same controller/target used by
Task_space_controller/robot_arm_controllers/task_space_test.py, just fed
from the Quest wrists instead of a draggable cube.

Each arm is fingertip-tip referenced (LEFT_/RIGHT_FINGER_TIP_BODIES in
bimanual_arm_cfg.py: midpoint of each gripper's two finger distal tips), not
the raw wrist link — see compute_gripper_tip_pose_b / compute_tip_ik_jacobian
there. RIGHT_FINGER_DISTAL_TIP_LOCAL was measured directly from the link7/
link8 STL vertex data (mesh outward-X extreme, Y/Z bounding-box center) using
the same method as the pre-existing LEFT_FINGER_DISTAL_TIP_LOCAL constants —
link7/link8 are NOT a simple mirror of link7l/link8l (separate STL files,
swapped X-extent — see bimanual_arm_cfg.py for the measurement).

Orientation of each tip frame is defined as its own wrist link's orientation
(see compute_gripper_tip_pose_w); only the tracked *position* is offset to
each fingertip midpoint instead of the wrist origin.

lambda_val=0.2 (both arms) was tuned live on the left arm: at the DLS
default (0.01), a ~0.1m commanded tip displacement pushed the Jacobian
condition number from ~60 to 4000+ and some axes moved AWAY from target
instead of converging. 0.2 keeps the condition number bounded (~100) and
converges to a stable ~0.05m residual instead. Not yet validated on the
right arm specifically (mechanically mirrors the left, so 0.2 is a
reasonable starting point) — retune per-arm live if needed.

Coordinate mapping
------------------
WebXR uses a Y-up frame (X-right, Y-up, -Z-forward). The robot base is in
a Z-up world frame and is rotated 180° about Z, so the mapping is applied in
two stages (see _QUEST_TO_WORLD and the per-arm delta computation below).

Usage
-----
Inside the simulation_isaac container:
    ISAAC_LAB=/workspace/isaaclab \\
    PYTHONPATH=/workspace/humanoid/autonomy/simulation/quest_isaac_teleop:$PYTHONPATH \\
    /workspace/isaaclab/isaaclab.sh -p \\
        /workspace/humanoid/autonomy/simulation/quest_isaac_teleop/run_quest_armv2_teleop.py \\
        --device cpu

Or via the helper script:
    ./run_quest_armv2_teleop.sh
"""

import argparse
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

from isaaclab.app import AppLauncher

# ── path setup (must be before AppLauncher so PYTHONPATH is correct) ─────────
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parent
_KEYBOARD_TELEOP_DIR = _SIM_DIR / "Teleop" / "keyboard_based_teleoperation"
sys.path.insert(0, str(_KEYBOARD_TELEOP_DIR))
_IL_PKG = _SIM_DIR.parent / "il"


def _ensure_il_on_path() -> None:
    if str(_IL_PKG) not in sys.path:
        sys.path.insert(0, str(_IL_PKG))


# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Quest armWithStand teleop (both arms: DLS fingertip IK)")
parser.add_argument("--gain", type=float, default=1.0,
                    help="Motion gain: metres of EE motion per metre of real wrist motion")
parser.add_argument("--record", action="store_true",
                    help="Record demonstrations (requires: pip install -e autonomy/il[record]). Records the "
                         "L-suffixed (link6l) arm only -- that's the arm ego_cam/wrist_cam are mounted for.")
parser.add_argument("--schema", type=str,
                    default=str(_IL_PKG / "config" / "dataset_schema_wato_arm_v2_push_box.yaml"),
                    help="Path to dataset_schema.yaml (only used with --record)")
parser.add_argument("--sink", type=str, default="lerobot",
                    help="Output sinks when --record: lerobot, hdf5, or lerobot,hdf5")
parser.add_argument("--dataset_root", type=str, default=None,
                    help="Override record.root from schema")
parser.add_argument("--num_episodes", type=int, default=10)
parser.add_argument("--task_description", type=str, default="pick up box and place it in container")
parser.add_argument("--publish-real-left-arm", action="store_true",
                    help="Also publish the left arm's DLS-solved joint targets to /behaviour/arm_pose for "
                         "joint_command_node to drive the REAL physical left arm over CAN. Off by default -- "
                         "sim-only unless explicitly requested. RIGHT ARM HAS NO REAL-HARDWARE CAN MAPPING YET "
                         "(hardware_mapping.yaml only has a left: section) -- this flag does not and cannot "
                         "touch the right arm. Requires can_node + joint_command_node already running, the "
                         "CANable adapter connected, e-stop armed, and the real arm manually positioned near "
                         "the sim's rest pose (see PUBLISH_START_DELAY below).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── post-launch imports (require Omniverse runtime) ───────────────────────────
import rclpy  # noqa: E402
import torch  # noqa: E402
from rclpy.node import Node  # noqa: E402
from common_msgs.msg import QuestHandPose  # noqa: E402

import carb  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import CameraCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
    quat_apply,
    quat_apply_inverse,
    quat_from_matrix,
    quat_inv,
    quat_mul,
    subtract_frame_transforms,
)

# NOTE on the aliasing below: bimanual_arm_cfg.py's LEFT_*/RIGHT_* constants were
# corrected (the "L"-suffixed URDF chain is physically the RIGHT arm, not left --
# the mechanical/CAD naming had it backwards). This file's own left_*/right_* local
# variables mean something different: "the arm driven by the LEFT/RIGHT Quest
# wrist" (msg.left_wrist / msg.right_wrist), a Quest-hand-relative label that is
# NOT part of that correction. Aliasing here keeps every local left_*/right_*
# variable below pointing at the SAME physical URDF chain (and therefore the same
# real hardware) it always did -- only the upstream symbol names changed.
from armWithStand_v2_cfg import (  # noqa: E402
    ARM_V2_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    RIGHT_ARM_JOINTS as LEFT_ARM_JOINTS,
    RIGHT_EE_BODY as LEFT_EE_BODY,
    RIGHT_FINGER_TIP_BODIES as LEFT_FINGER_TIP_BODIES,
    RIGHT_FINGER_DISTAL_TIP_LOCAL as LEFT_FINGER_DISTAL_TIP_LOCAL,
    RIGHT_GRIPPER_JOINTS as LEFT_GRIPPER_JOINTS,
    LEFT_EE_BODY as RIGHT_EE_BODY,
    LEFT_FINGER_TIP_BODIES as RIGHT_FINGER_TIP_BODIES,
    LEFT_FINGER_DISTAL_TIP_LOCAL as RIGHT_FINGER_DISTAL_TIP_LOCAL,
    apply_joint_limits,
    compute_gripper_tip_pose_b,
    compute_tip_ik_jacobian,
    resolve_body_ids,
    resolve_joint_name,
)

# ── constants ─────────────────────────────────────────────────────────────────

_RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
# VERIFIED empirically (headless sim, fingertip-gap measurement) -- see
# armWithStand_v2_cfg.py's GRIPPER_OPEN/CLOSED comment for the same swap on
# joint7l/8l. (joint7=0, joint8=0) -> gap=0.161m (open);
# (joint7=-0.05, joint8=0.05) -> closed. Opposite of bimanual's labeling for
# joint7/8 (different prismatic axis, see module docstring).
_RIGHT_GRIPPER_OPEN = {"joint7": 0.0, "joint8": 0.0}
_RIGHT_GRIPPER_CLOSED = {"joint7": -0.05, "joint8": 0.05}

# RETIRED for orientation (still referenced by name in old comments/history) -- WebXR (Y-up:
# X-right, Y-up, -Z-forward) -> simulation world frame (Z-up), as a FIXED world-frame rotation.
# This was proven wrong for hand POSITION (see below: raw X drove forward/back instead of left/
# right) and fixed there by switching to a camera-relative basis -- but that same fix was never
# applied to wrist ORIENTATION, which kept using this exact matrix (see git history / the old
# dq_left_world computation). That's the near-certain root cause of "can't orient the gripper to
# match the cube" -- a live-reported symptom entirely consistent with rotating your wrist about
# what feels like a natural axis (e.g. rolling your forearm) mapping to some unrelated, unintuitive
# world axis instead of the camera/gripper's own corresponding axis. Superseded by
# _QUEST_TO_CAM_LOCAL below, which applies the SAME camera-relative philosophy already verified
# correct for position.
_QUEST_TO_WORLD = torch.tensor(
    [[-1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, 1.0, 0.0]],
    dtype=torch.float32,
)
# Hand POSITION deltas are mapped camera-relative instead (see
# _CAM_LOCAL_FORWARD/UP/RIGHT and their use in the main loop below) --
# verified via synthetic single-axis ros2 topic pub tests (isolate raw
# x/y/z, read the [axisdbg] world_delta) that the straight fixed-world-frame
# mapping (formerly also used for orientation, see _QUEST_TO_WORLD's comment above) puts raw X
# into the world forward/back row and raw Z into the lateral row -- i.e. physically moving
# your hand right/left drove the arm forward/back and vice versa. Rather
# than patch that swap with more sign flips, position deltas are now built
# directly from verified WebXR semantics (X=right, Y=up, -Z=forward) applied
# in the camera's own basis, then rotated into world by the camera's fixed
# mount tilt -- so "push away from you, into the view" tracks the camera's
# actual boresight (partially world -Z at this ~60deg downward tilt) instead
# of raw world +X. Flip an entry here only if a live headset test shows a
# genuine handedness/chirality issue, not for tilt -- tilt is now handled by
# the camera-relative basis itself.
# Same idea, now also applied to wrist ORIENTATION (see _QUEST_TO_WORLD's comment above for why
# the old approach was wrong): maps a rotation expressed in WebXR's local device frame (X=right,
# Y=up, -Z=forward -- SAME convention as _CAM_LOCAL_FORWARD/UP/RIGHT's physical meaning, just
# expressed in the operator's controller/hand frame instead of the camera mount's local frame)
# into the camera MOUNT's own local frame, via the natural ergonomic correspondence: quest
# forward (-Z) -> camera forward (_CAM_LOCAL_FORWARD, local +X); quest up (+Y) -> camera up
# (_CAM_LOCAL_UP, local +Z); quest right (+X) -> camera right (_CAM_LOCAL_RIGHT, local -Y).
# Row i = camera-local axis i's coefficients on (quest_x, quest_y, quest_z):
#   cam_local_X (forward) = -quest_z
#   cam_local_Y (left; camera "right" is -Y per _CAM_LOCAL_RIGHT)  = -quest_x
#   cam_local_Z (up)      =  quest_y
# The resulting camera-local delta is then rotated into world by camera_tilt_quat (the camera
# mount's actual current world orientation) -- the SAME two-stage camera-relative-then-tilted
# construction already verified for position, just for a rotation instead of a translation.
_QUEST_TO_CAM_LOCAL = torch.tensor(
    [[0.0, 0.0, -1.0],
     [-1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0]],
    dtype=torch.float32,
)
_AXIS_SIGN_LEFT = torch.tensor([1.0, 1.0, 1.0])  # [right, up, forward]
_AXIS_SIGN_RIGHT = torch.tensor([1.0, 1.0, 1.0])  # [right, up, forward]
_CAM_LOCAL_FORWARD = torch.tensor([1.0, 0.0, 0.0])  # rsd455 local +X = boresight
_CAM_LOCAL_UP = torch.tensor([0.0, 0.0, 1.0])  # rsd455 local +Z = up
_CAM_LOCAL_RIGHT = torch.tensor([0.0, -1.0, 0.0])  # matches _EYE_LOCAL_RIGHT below

_GAIN_FAR = 1.0  # reverted to match run_quest_bimanual_teleop.py's original 1:1 tracking precision
# (was bumped to 2.0 for more reach per live feedback, but that made the target markers feel
# less precisely tied to real hand movement compared to Allen/Wilson's original -- reverted)
_GAIN_RAMP_START_M = 0.15
_GAIN_RAMP_END_M = 0.35

# Was 0.28, bumped to 0.35, then to 0.5 per live feedback. Shoulder-to-wrist
# chain length in the home config (joint1L..joint6l origins,
# armWithStand.urdf) is ~0.61m, and shoulder-to-fingertip is roughly
# ~0.70-0.75m (link-length-sum estimate, not a verified FK sweep) -- 0.5m
# stays under that ceiling but is a much bigger fraction of it than before,
# so watch for the right arm's DLS conditioning degrading near the new
# limit (see _DLS_LAMBDA_RIGHT) -- retune live if it locks up.
_MAX_REACH_M = 0.5

# Home/rest IK target X offset. Keep at 0 -- the arm should run at its
# natural rest position; fix camera/framing issues on the camera/enclosure
# side, not by dragging the arm's target away from where it actually rests.
_HOME_TIP_X_OFFSET = 0.0
# Base-frame +Z = up (rest tip Z is negative, e.g. ~-0.666, since base_link
# sits at the TOP of the stand and the arm hangs down from it). Rest tip was
# sitting ~0.145m BELOW the table top (table top ~0.679m world Z, rest tip
# ~0.534m world Z) -- +0.2 lifts the starting pose to comfortably clear the
# table surface ("slightly above the table"), per explicit request.
_HOME_TIP_Z_OFFSET = 0.2

# Real-hardware bridge timing (see --publish-real-left-arm). Mirrors
# Task_space_controller/robot_arm_controllers/task_space_real.py's
# PUBLISH_PERIOD/PUBLISH_START_DELAY exactly -- same joint_command_node on
# the other end, which applies NO velocity/delta rate-limiting to the very
# first ArmPose message it receives after startup. An un-delayed first
# publish could snap the real arm hard from wherever it physically is to the
# sim's current target. The delay exists so a human can manually position
# the real arm near the sim pose first -- don't shorten it.
_REAL_ARM_PUBLISH_PERIOD_S = 0.02  # 20ms = 50Hz, matches joint_command_node's control_rate_hz
_REAL_ARM_PUBLISH_START_DELAY_S = 5.0

# Both reset to identity (bimanual_arm's non-identity _WRIST_ORIENT_OFFSET_RIGHT
# was empirically tuned against ITS OWN joint6l axis -- armWithStand's joint6l
# rotates about a different local axis (Y here vs X in bimanual, confirmed by
# diffing both URDFs' <joint><axis> tags), so that offset does not transfer.
# If wrist rotation feels like it's driving the EE the wrong way, retune this
# live per the README's "Wrist orientation alignment" section, same procedure
# used to derive bimanual's original values.
_WRIST_ORIENT_OFFSET_LEFT = torch.tensor([1.0, 0.0, 0.0, 0.0])
_WRIST_ORIENT_OFFSET_RIGHT = torch.tensor([1.0, 0.0, 0.0, 0.0])
# Per-arm rotation-offset conjugation applied to the wrist orientation delta.
# Identity = no correction. If rotating your wrist drives the EE the wrong
# way, retune per the README's "Wrist orientation alignment" table -- but if
# a fixed-angle offset doesn't fix a "rotate one way, EE goes the opposite
# way" feel, it's likely a chirality/mirror issue (WebXR can report
# left/right hand joints in mirrored local conventions), which needs a sign
# flip on the raw wrist quaternion components instead, not a rotation offset
# here.

_THUMB_TIP_IDX = 4
_INDEX_TIP_IDX = 9
_PINCH_CLOSE_M = 0.035  # was 0.030, nudged up per live feedback (easier to trigger close)
_PINCH_OPEN_M = 0.050

_DLS_LAMBDA = 0.2
# Right arm gets extra damping (not a smaller _MAX_REACH_M): near full
# extension its Jacobian conditioning is worse than the left arm's (the two
# arms' default joint poses were measured independently, not mirrored), and
# it would freeze mid-reach with target_err stuck near the clamp. Heavier
# damping keeps the full reach distance but makes the solver more
# conservative exactly where conditioning gets bad. Retune live if it's
# sluggish well before full extension, or still locks up at the extreme.
_DLS_LAMBDA_RIGHT = 0.35
# _DLS_LAMBDA/_DLS_LAMBDA_RIGHT above are now MINIMUM (floor) damping values, not fixed --
# per DLS/singularity-avoidance literature (Chiaverini et al.), a constant lambda either damps
# too little right at a singularity (this is a plausible root cause of the joint-jitter flagged
# earlier and never fully explained -- single-frame jumps ~9x over the velocity clamp) or damps
# too much everywhere else (unnecessary tracking-accuracy/responsiveness loss). _DLS_LAMBDA_RIGHT
# being manually raised above was already a static, hand-tuned approximation of exactly what
# adaptive damping does automatically for the region where conditioning is worst -- this
# generalizes that fix to ramp up smoothly and automatically as manipulability drops, instead of
# a flat value everywhere. lambda_max is the ceiling applied only very close to a singularity;
# _DLS_MANIPULABILITY_EPSILON is the manipulability threshold below which damping starts ramping
# up at all (at/above it, lambda stays at the existing floor value, unchanged from before).
# UNVERIFIED live -- unlike this session's other changes, this one has real risk if
# mistuned (too-low epsilon = damping never kicks in when it should; too-high lambda_max =
# solver gets sluggish/inaccurate even in comfortable poses). A [Quest][ikdiag] print is added
# below specifically to sanity-check live what manipulability/lambda values actually occur
# during normal use, since no live data on this arm's own conditioning was available to tune
# epsilon against beforehand.
_DLS_LAMBDA_MAX = 0.6
_DLS_LAMBDA_MAX_RIGHT = 0.9
_DLS_MANIPULABILITY_EPSILON = 0.01

# Joint-space output smoothing -- mirrors joint_command_core.cpp's actual
# ACTIVE default on the interfacing-fixes branch: plain velocity clamp +
# delta clamp (clampStep), NOT the trapezoidal ramp (enable_trapezoidal_limit
# defaults to false there -- untested on hardware, so not imitated here
# either) and NOT a low-pass filter (removed entirely on that branch --
# see commit 6fe705a5 -- it silently discounts steady-state speed).
# Values start from safety_limits.yaml's `global` block (velocity_max: 20
# deg/s, delta_max: 2.4 deg), converted to radians -- velocity_max bumped to
# 40deg/s per live feedback (20 was hardware's conservative bench-testing
# speed and visibly lagged behind the target during fast VR hand motion; not
# read from the yaml automatically, see this constant's own definition).
# delta_max left at the original 2.4deg. The real config also has tighter
# PER-JOINT overrides (shoulder/elbow/wrist), not applied here --
# armWithStand_v2_cfg.py's joint grouping (2-DOF shoulder/3-DOF elbow)
# doesn't match hardware_mapping.yaml's ArmPose split (3-DOF shoulder/2-DOF
# elbow, already flagged elsewhere as unverified), so mapping per-joint
# values onto this file's joint order isn't trustworthy yet -- the uniform
# global value is applied to all 6 joints instead.
_JOINT_VELOCITY_MAX_RAD_S = 1.7453292519943295  # 100 deg/s -- still the enforced speed ceiling, now applied via _smooth_damp instead of a hard clamp (see solve_and_apply)
_JOINT_DELTA_MAX_RAD = 0.2617993877991494  # 15 deg per control step -- UNUSED, kept for reference (superseded by _smooth_damp)
# "Time to close ~90% of the gap to target" for the critically-damped smoothing filter (see
# _smooth_damp) -- smaller = snappier/more responsive, larger = smoother but laggier. 0.08s
# chosen as a starting point: tight enough to stay responsive for precise grasping, loose enough
# to meaningfully round off the bang-bang motion the previous hard clamp produced. Retune live if
# it feels sluggish (lower it) or still jerky (raise it).
_SMOOTH_TIME_S = 0.08

_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))
# Back wall shifted to clear the mount/stand structural bar. Front boundary
# extended to 0.75m so the enclosure actually contains the arms' full reach
# (rest EE X~0.42-0.43 + _MAX_REACH_M=0.28 = ~0.71m) instead of the arms
# sticking out past it.
_ENCLOSURE_BACK_X = -0.45
_ENCLOSURE_FRONT_X = 0.75  # open front (arm reach direction) -- extended to actually contain the arms, see above
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
# armWithStand's stand is ~1.1997m tall below the robot's own origin (vs.
# bimanual's much shorter stand, which is what the original 0.9m height was
# sized for). After lifting the robot so the stand's base rests on the table
# (see _ARM_TABLE_LIFT_Z below), the arm's highest point sits at
# ~1.1997 + 0.257 = 1.457m above the table. 1.8m leaves ~0.34m of headroom
# for reach motion.
_ENCLOSURE_TOP_Z = 1.8
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_FRONT_X - _ENCLOSURE_BACK_X
_ENCLOSURE_X_CTR = (_ENCLOSURE_BACK_X + _ENCLOSURE_FRONT_X) / 2


# Real table asset (converted from the user's Table.STEP CAD file -- see
# Table/table.usd). Source units are inches (metersPerUnit=0.0254 on the
# converted USD), hence the scale correction below. Its default prim's
# origin sits at the table-TOP surface (bbox top face at local Z=0, verified
# via direct USD query), same convention the old flat cuboid table used, so
# no Z offset is needed here -- just place it so the top surface is at
# world Z=0. Real-world size: ~1.39m (X) x 0.75m (Y) x 0.62m tall.
_TABLE_USD_PATH = str(
    _SIM_DIR / "Humanoid_Wato" / "Table" / "table.usd"
)
_TABLE_SCALE = (0.0254, 0.0254, 0.0254)
# Table pose -- user GUI-verified transform: Translate=(0.84149, -0.13977,
# -0.0), Orient XYZ=(90, 90, 0) degrees (quat computed via the same
# Rx*Ry*Rz USD rotateXYZ convention used for wrist_cam elsewhere in this
# file) -- the STEP conversion's native axes don't come out Z-up-front by
# default, hence the rotation.
_TABLE_POS = (0.69, 0.00612, 0.33)  # raised 10, 15, 20cm, lowered 10, 5cm, raised 3cm, per live feedback
_TABLE_ROT = (0.5000000000000001, 0.5, 0.5, 0.49999999999999994)  # wxyz

# Box (to grasp) and container (to place it in). Box now placed INSIDE the
# container (same X/Y/Z as the container's own origin -- both assets have
# their local origin at a bottom corner, so this rests the box flush on the
# container's floor) per live feedback, rather than off to the side.
_BOX_USD_PATH = str(
    _SIM_DIR / "Humanoid_Wato" / "UsdModelAssets" / "block.usd"
)  # 5.08cm cube -- box.usd (25x25x3cm flat pad) is too flat/wide for this gripper to grasp
_CONTAINER_USD_PATH = str(
    _SIM_DIR.parent.parent / "assets" / "lerobot" / "so101_vial_task" / "usd" / "tray.usda"
)
_CONTAINER_POS = (0.3, -0.07041, 0.70917)  # +33cm net with the table
_CONTAINER_ROT = (0.7071067811865476, 0.0, 0.0, 0.7071067811865475)  # wxyz
_BOX_POS = (0.3, 0.2, 0.70917)  # pushed back, per live feedback

# Stereo camera pair (real depth via two eye textures, not a flat
# single-camera POV): two RealSense D455s mounted on base_link, fixed at
# _HEAD_VIEWPOINT_HOME_POS/QUAT (head tracking is currently disabled --
# head_pose is still read from QuestHandPose.msg but unused for positioning).
#
# rsd455's local axes: +X = lens boresight (forward), +Z = up. That makes
# local -Y "right" (forward x up = X x Z = -Y) -- IPD offset is applied
# along that axis, rotated by the viewpoint's current orientation.
_RSD455_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
# Forward-facing head cam per explicit request: aiming precisely at the
# rest-pose fingertips (previous version) forced a ~92deg near-vertical
# pitch, because the zero-pose hands droop well below shoulder height --
# that's not what a head cam should track anyway, since during actual
# teleop the hands move wherever the user directs them, not the idle rest
# spot. This is a clean, level, mild ~25deg downward glance facing +X (the
# confirmed "front" direction from earlier live testing: the ego_cam-matched
# orientation was reported facing left, a 90deg turn right landed on +X).
# Built directly from forward/up vectors (not by composing incremental
# rotations, which likely introduced unintended roll in an earlier attempt)
# -- verified the up vector has zero roll component. Orientation confirmed
# correct live; position then nudged 5mm along the gaze direction
# (head_pos + 0.005*forward) since the camera mount stand itself was
# blocking the bottom of frame at the original position.
# Reverted per live feedback ("the other side of the table" viewpoint wasn't working) -- back to
# the original near-side view. The far-side/180deg-yaw attempt is preserved here, commented, in
# case it's worth revisiting: pos=(0.5094397380234622, 0.04, 0.23809789390566405),
# quat=(0.0, -0.42261826174069944, 0.0, 0.9063077870366499) (old pitch quat + 180deg yaw about Z,
# position mirrored across the box/container's X=0.3).
_HEAD_VIEWPOINT_HOME_POS = (0.0905602619765378, 0.04, 0.23809789390566405)  # translate xyz, relative to base_link -- nudged along the gaze direction (5,10,20,20,10,20,15,5,+10mm) and left (20,20mm) per live feedback
_HEAD_VIEWPOINT_HOME_QUAT = (0.9063077870366499, 0.0, 0.42261826174069944, 0.0)  # ~50deg downward pitch, facing +X, no roll (was ~40deg, tilted 10deg more/downward per live feedback)
_EYE_LOCAL_RIGHT = torch.tensor([0.0, -1.0, 0.0])
_EYE_IPD_M = 0.063
_EYE_LOCAL_FORWARD = torch.tensor([1.0, 0.0, 0.0])
# ego_cam is pinned to the left eye's exact pose (see the per-frame
# _set_mount_pose call below) but nudged forward by this much along the
# gaze direction -- exact co-location put its lens inside the RSD455
# payload's own visible housing mesh, blacking out half the frame. Retune
# live if the housing is still in view or the vantage drifts too far from
# the actual left-eye position.
_EGO_CAM_FORWARD_OFFSET_M = 0.25  # nudged back slightly from 0.3 per live feedback, now that the
# real bug (clippingRange) is fixed and this value is no longer chasing a phantom position issue.
# User identified the actual fix live: a straight +Z (base-frame, not gaze-relative) offset --
# the forward-offset direction alone wasn't enough because the camera needed to clear something
# above/below it, not in front of it.
_EGO_CAM_UP_OFFSET_M = 0.1
# Debug-only: when True, skips the per-frame _set_mount_pose call on ego_cam
# so a manual GUI drag/rotate sticks instead of being overwritten every
# frame -- set back to False once done manually probing for a good pose,
# since with this on ego_cam no longer follows the head viewpoint at all.
_EGO_CAM_FREE_MOVE_DEBUG = False  # real bug found (clippingRange), no longer needed
# ego_cam is a bare USD Camera prim (standard convention: looks down local
# -Z, up +Y), NOT the rsd455 payload (local +X=forward, +Z=up, -Y=right,
# established earlier this session). head_orient_b/_HEAD_VIEWPOINT_HOME_QUAT
# are calibrated for the rsd455 convention, so applying them to ego_cam
# directly points it the wrong way even though the quaternion *value* is
# identical to the left eye's. This is the basis-change rotation (USD-camera
# local axes -> rsd455 local axes) that corrects for it -- verified
# numerically: maps (0,0,-1)->(1,0,0), (0,1,0)->(0,0,1), (1,0,0)->(0,-1,0).
# Compose as quat_mul(head_orient_b, this), NOT the other way around.
_EGO_CAM_CONVENTION_FIX_QUAT = (-0.5, -0.5, 0.5, 0.5)
# Extra downward pitch applied to ego_cam ONLY, on top of the left-eye-
# matching baseline (left eye POV keeps its own orientation, untouched).
# Composed as the OUTERMOST rotation (quat_mul(this, head_orient_b *
# convention_fix)) -- verified numerically before first applying: baseline
# pitch is exactly 50deg (matches _HEAD_VIEWPOINT_HOME_QUAT), same
# base-frame-Y-axis convention (positive sin term = downward). Net +20deg
# from baseline (70deg total pitch) -- went 50->60->70 across two rounds,
# corrected back down to 60 per live feedback ("wrong to orient it down"),
# then +10 more per live feedback (back up to 70).
_EGO_CAM_EXTRA_TILT_QUAT = (0.984807753012208, 0.0, 0.17364817766693033, 0.0)
# Sub-path to the actual renderable Camera prim inside the rsd455 payload --
# same as the SO101 vial task's camera_external_D455 (task_env_cfg.py).
_RSD455_CAMERA_SUBPATH = "rsd455/RSD455/Camera_OmniVision_OV9782_Right"
# Widen the RSD455's baked-in FOV (~90.5deg horizontal by default, measured
# live: focalLength=1.93 at horizontalAperture=3.896) to ~120deg -- the head
# is too close to the arm to see its full range of motion by moving the
# camera further back (the stand/mount geometry blocks the view first), so
# widen the lens instead. focalLength solved from the standard
# fov = 2*atan(aperture / (2*focalLength)) relation, aperture unchanged.
_RSD455_WIDENED_FOCAL_LENGTH = 1.1246782979523935  # ~120deg horizontal FOV


def _widen_camera_fov(camera_prim_path: str, focal_length: float) -> None:
    """Override a (possibly not-yet-loaded) Camera prim's focalLength. The
    rsd455 payload can take a few frames to compose after AddPayload(), so
    this retries for a short while rather than assuming the prim is already
    valid -- setting the attribute before the payload loads would silently
    no-op (or fail) instead of taking effect."""
    stage = omni.usd.get_context().get_stage()
    app = omni.kit.app.get_app_interface()
    for _ in range(100):
        cam_prim = stage.GetPrimAtPath(camera_prim_path)
        if cam_prim.IsValid() and cam_prim.IsA(UsdGeom.Camera):
            UsdGeom.Camera(cam_prim).GetFocalLengthAttr().Set(focal_length)
            print(f"[Quest] Widened FOV on {camera_prim_path} (focalLength={focal_length})", flush=True)
            return
        app.update()
    print(f"[Quest] WARNING: could not widen FOV on {camera_prim_path} (camera prim never became valid)", flush=True)


def _read_camera_fov(camera_prim_path: str) -> tuple[float, float, float] | None:
    """Read a (possibly not-yet-loaded) Camera prim's focalLength/
    horizontal/verticalAperture -- same load-wait pattern as
    _widen_camera_fov, but reading instead of writing. Used to copy the
    RSD455's real, native (unwidened) FOV onto ego_cam so recorded frames
    match what an actual D455 would see -- read this BEFORE
    _widen_camera_fov changes the value for the teleop display."""
    stage = omni.usd.get_context().get_stage()
    app = omni.kit.app.get_app_interface()
    for _ in range(100):
        cam_prim = stage.GetPrimAtPath(camera_prim_path)
        if cam_prim.IsValid() and cam_prim.IsA(UsdGeom.Camera):
            cam = UsdGeom.Camera(cam_prim)
            return (
                cam.GetFocalLengthAttr().Get(),
                cam.GetHorizontalApertureAttr().Get(),
                cam.GetVerticalApertureAttr().Get(),
            )
        app.update()
    print(f"[Quest] WARNING: could not read FOV on {camera_prim_path} (camera prim never became valid)", flush=True)
    return None


def _set_camera_fov(camera_prim_path: str, focal_length: float, h_aperture: float, v_aperture: float) -> None:
    """Set a Camera prim's focalLength/horizontal/verticalAperture directly
    (prim is assumed already valid/loaded -- ego_cam is baked into the USD,
    not a late-composing payload like rsd455)."""
    stage = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera(stage.GetPrimAtPath(camera_prim_path))
    cam.GetFocalLengthAttr().Set(focal_length)
    cam.GetHorizontalApertureAttr().Set(h_aperture)
    cam.GetVerticalApertureAttr().Set(v_aperture)


def _project_world_point_to_uv(camera_prim: Usd.Prim, world_xyz: tuple) -> tuple[float, float, bool]:
    """Projects a world-space point into the given USD Camera prim's normalized image UV (0..1,
    top-left origin -- matching how the captured PNG's rows are addressed), using the camera's
    REAL resolved focalLength/aperture/world-transform (UsdGeom.Camera.GetCamera() composes the
    full prim hierarchy, including the rsd455 payload's internal camera-to-mount offset -- no
    need to know that offset here). This is the same camera capture_viewport_to_file renders
    pov_left.png/pov_right.png from, so a point projected this way lands at the same pixel it
    appears at in the image, unlike drawing at the viewer's own raw WebXR hand-tracking position
    (which is a different coordinate space entirely -- see live feedback: "the blue ball is not
    at the same position it usually is... it should be like before").
    Returns (u, v, visible); visible is False when the point is behind the camera or outside its
    frustum, in which case the caller should skip drawing rather than trust u/v."""
    cam = UsdGeom.Camera(camera_prim).GetCamera(Usd.TimeCode.Default())
    view = cam.frustum.ComputeViewMatrix()
    proj = cam.frustum.ComputeProjectionMatrix()
    clip = Gf.Vec4d(world_xyz[0], world_xyz[1], world_xyz[2], 1.0) * view * proj
    if clip[3] <= 1e-6:
        return 0.5, 0.5, False
    ndc_x = clip[0] / clip[3]
    ndc_y = clip[1] / clip[3]
    u = (ndc_x + 1.0) / 2.0
    v = (1.0 - ndc_y) / 2.0
    visible = -1.0 <= ndc_x <= 1.0 and -1.0 <= ndc_y <= 1.0
    return u, v, visible


def _attach_rsd455_camera(parent_prim_path: str, mount_name: str, translate: tuple, orient_wxyz: tuple) -> str:
    """Attach an rsd455 payload as a `mount_name` child Xform of any prim.
    Returns the mount prim path. Safe to call _set_mount_pose(..., create=
    False) on the returned path repeatedly afterward -- see that function's
    docstring for why create=True is only used once, here."""
    mount_path = f"{parent_prim_path}/{mount_name}"
    _set_mount_pose(mount_path, translate, orient_wxyz, create=True)
    stage = omni.usd.get_context().get_stage()
    rsd_prim = stage.DefinePrim(f"{mount_path}/rsd455")
    rsd_prim.GetPayloads().AddPayload(_RSD455_USD_URL)
    return mount_path


def _set_mount_pose(mount_path: str, translate: tuple, orient_wxyz: tuple, create: bool = False) -> None:
    """Set (or create) a camera mount Xform's translate/orient. Called every
    frame for the stereo eye mounts to follow head tracking.

    NOTE: AddTranslateOp()/AddOrientOp() are NOT idempotent in this USD
    version -- calling them again on a prim that already has that op raises
    'xformOp already exists in xformOpOrder' (confirmed live, this used to
    assume otherwise and crashed on the second frame). So ops are added ONCE
    at create=True, and every subsequent call fetches the existing ops via
    GetOrderedXformOps() and .Set()s them directly instead."""
    stage = omni.usd.get_context().get_stage()
    if create:
        prim = UsdGeom.Xform.Define(stage, mount_path)
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    else:
        prim = stage.GetPrimAtPath(mount_path)
        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        translate_op = next(op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
        orient_op = next(op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient)

    translate_op.Set(Gf.Vec3d(*translate))
    w, x, y, z = orient_wxyz
    orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


def _open_pov_viewport(camera_prim_path: str, window_name: str, width: int = 480, height: int = 360):
    """Best-effort: open a Kit viewport window locked to a camera prim, for a
    live POV feed on the host monitor AND the source for the periodic
    capture_viewport_to_file calls that feed the Quest browser (see
    _POV_FRAME_PATH_LEFT/RIGHT in run_simulator). Returns the viewport_api or
    None if the viewport extension isn't available in this Kit experience --
    a missing viewport degrades the feed, it doesn't crash teleop.

    Default resolution lowered from 640x480 (per live feedback: recording
    still felt slow on this GPU) -- these viewport windows are NOT what
    recording reads (that's the separate ego_cam/wrist_cam CameraCfg sensor
    tensors, at their own fixed resolution), so shrinking them only affects
    the headset's own eye-texture visual fidelity and the host-side preview
    windows, never recorded data quality."""
    try:
        from omni.kit.viewport.utility import create_viewport_window

        viewport_window = create_viewport_window(window_name, width=width, height=height)
        viewport_window.viewport_api.camera_path = camera_prim_path
        return viewport_window.viewport_api
    except Exception as exc:  # noqa: BLE001 -- best-effort convenience feature
        print(f"[Quest] Could not open POV viewport '{window_name}': {exc}", flush=True)
        return None


# Frames the eye viewports get captured to, for the Quest browser to fetch as
# payload.head_pose-tracked stereo textures (see index.html's
# leftEyeTexture/rightEyeTexture). Live in the WebXR static dir
# (autonomy/teleop/quest_teleop/static/) so webxr_server.py's existing
# SimpleHTTPRequestHandler serves them as plain static files -- no server
# code changes needed. Captured every _POV_CAPTURE_EVERY_N_STEPS sim steps,
# aligned with render_interval below (see main()'s SimulationCfg) so every
# capture lands on an actually-rendered frame instead of re-capturing a
# stale buffer -- capturing more often than the app renders wastes I/O for
# no new pixels. At 100Hz physics / render_interval=3 that's ~33Hz -- raised
# back from render_interval=5 (~20Hz) once the REAL headset-fps bottleneck
# was found and fixed: index.html's browser-side POV poll interval was
# hardcoded to 200ms (5fps), a hard cap totally independent of this backend
# rate -- laptop-side render was already fine, the headset specifically was
# never going to see more than 5fps no matter how fast the sim rendered.
# The camera mounts themselves are repositioned every physics step
# regardless (cheap USD attribute writes) so head tracking stays responsive
# even between rendered frames.
_POV_STATIC_DIR = _SIM_DIR.parent / "teleop" / "quest_teleop" / "static"
_POV_FRAME_PATH_LEFT = _POV_STATIC_DIR / "pov_left.png"
_POV_FRAME_PATH_RIGHT = _POV_STATIC_DIR / "pov_right.png"
_POV_CAPTURE_EVERY_N_STEPS = 3
# Small wrist_cam HUD overlay in the headset, per live feedback. Written directly from the
# existing wrist_cam CameraCfg SENSOR tensor (the same one recording reads) rather than
# capture_viewport_to_file + a dedicated viewport window -- that window was deliberately removed
# earlier for render performance (see the removed _open_pov_viewport("Wrist Cam Preview") call),
# so this avoids re-adding that cost while still getting the image into the headset.
_WRIST_CAM_FRAME_PATH = _POV_STATIC_DIR / "wrist_cam.png"
# IK-target marker screen position, per eye, projected through the REAL eye cameras (see
# _project_world_point_to_uv) -- polled by index.html alongside pov_left.png/pov_right.png so it
# can draw the marker at the position it actually appears in the rendered image, instead of at
# the browser's own raw WebXR hand-tracking position (a different coordinate space -- that's what
# made the client-side marker appear at the physical wrist instead of "in front of, teleoped by"
# the operator per live feedback). Written with a temp-file-then-rename so the browser never
# reads a half-written JSON file.
_MARKER_UV_PATH = _POV_STATIC_DIR / "marker_uv.json"


# ── helpers ───────────────────────────────────────────────────────────────────

@configclass
class ArmV2SceneCfg(InteractiveSceneCfg):
    """Bare arm stand + the white lightbox enclosure walls/table (visual only, no collision on the
    walls) -- same geometry as BimanualPushBlockSceneCfg in
    HumanoidRLSetup/tasks/push/bimanual_env_cfg.py. Always on (no CLI flag)."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    # armWithStand's stand is much taller than bimanual's -- its bottom sits
    # ~1.1997m below the robot's own origin (base_link). The surface the
    # robot should rest on is the TABLE's top (z=0, see the "table" asset
    # below: pos.z=-_TABLE_THICKNESS/2, thickness=_TABLE_THICKNESS -> spans
    # -0.05 to 0), NOT the separate collision-safety ground plane far below
    # at z=-1.05 -- lifting to match the ground plane (an earlier mistake)
    # left the stand still buried under the table. Lift by
    # (table_top_z - stand_bottom_z) = 0 - (-1.1997) = 1.1997m, same value
    # used for the arm_usd/armWithStand.usd scene's table-rest fix.
    robot = ARM_V2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ARM_V2_CFG.init_state.replace(pos=(0.0, 0.0, 1.1997)),
    )

    # Data-collection cameras -- NOT the teleop headset display (that's the
    # separate RSD455 stereo pair attached at runtime below). These wrap the
    # Camera prims already baked into armWithStand.usd's sensor layer
    # (spawn=None -- point at the existing prim, don't create a new one) so
    # recording code can read scene["ego_cam"]/scene["wrist_cam"].data.output
    # ["rgb"] each frame. wrist_cam is on link6l, the "right" arm's wrist
    # per this file's naming (RIGHT_EE_BODY in armWithStand_v2_cfg.py).
    ego_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/ego_cam",
        spawn=None,
        height=480,
        width=640,
        update_period=0.0,  # render_interval=3 (SimulationCfg) already throttles the app's overall
        # render rate to ~33Hz -- an ADDITIONAL per-camera update_period on top of that was redundant
        # and, per live feedback, caused ego_cam's feed to render a stale/frozen scene state (table
        # showing its old orientation) while the live main viewport was correct -- reverted to 0.0
        # (render every time the app renders, i.e. effectively ~33Hz via render_interval alone).
        data_types=["rgb"],
    )
    wrist_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link6l/wrist_cam",
        spawn=None,
        height=480,
        width=640,
        update_period=0.0,  # render_interval=3 (SimulationCfg) already throttles the app's overall
        # render rate to ~33Hz -- an ADDITIONAL per-camera update_period on top of that was redundant
        # and, per live feedback, caused ego_cam's feed to render a stale/frozen scene state (table
        # showing its old orientation) while the live main viewport was correct -- reverted to 0.0
        # (render every time the app renders, i.e. effectively ~33Hz via render_interval alone).
        data_types=["rgb"],
    )

    enclosure_back: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureBack",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_BACK_X, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(0.003, _ENCLOSURE_Y_SPAN, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_left: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureLeft",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MIN, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_right: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureRight",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MAX, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_top: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureTop",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, _ENCLOSURE_Y_SPAN, 0.003), visual_material=_ENCLOSURE_MATERIAL),
    )
    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_POS, rot=_TABLE_ROT),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_TABLE_USD_PATH,
            scale=_TABLE_SCALE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    # Graspable box -- dynamic rigid body, same rigid/mass/collision pattern
    # as pick_place_env_cfg.py's _cuboid_object_cfg, but referencing the
    # imported block.usd mesh instead of a procedural CuboidCfg.
    box: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_BOX_POS),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_BOX_USD_PATH,
            scale=(0.9, 0.9, 0.9),  # slightly smaller than the native 5.08cm cube, per live feedback
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    # Container -- kinematic (doesn't get knocked around, just needs to
    # collide with the box), same pattern as pick_place_env_cfg.py's
    # _tray_cfg.
    container: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Container",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_CONTAINER_POS, rot=_CONTAINER_ROT),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_CONTAINER_USD_PATH,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )


class QuestRosReceiver(Node):
    """ROS 2 subscriber that caches the latest /quest_teleop message."""

    def __init__(self) -> None:
        super().__init__("quest_ik_listener")
        self._latest: QuestHandPose | None = None
        self._lock = threading.Lock()
        self.create_subscription(QuestHandPose, "/quest_teleop", self._cb, 1)
        self.get_logger().info("Subscribed to /quest_teleop")

    def _cb(self, msg: QuestHandPose) -> None:
        with self._lock:
            self._latest = msg

    def poll(self) -> QuestHandPose | None:
        with self._lock:
            return self._latest


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {n: i for i, n in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, n)] for n in names]


def _make_entity_cfg(scene, joint_names: list[str], ee_body: str) -> SceneEntityCfg:
    cfg = SceneEntityCfg("robot", joint_names=joint_names, body_names=[ee_body])
    cfg.resolve(scene)
    return cfg


def _wrist_xyz(wrist) -> torch.Tensor:
    p = wrist.position
    return torch.tensor([p.x, p.y, p.z], dtype=torch.float32)


def _wrist_quat_wxyz(wrist) -> torch.Tensor:
    o = wrist.orientation
    return torch.tensor([o.w, o.x, o.y, o.z], dtype=torch.float32)


def _ramped_gain(disp_m: torch.Tensor, gain_near: float) -> torch.Tensor:
    t = ((disp_m - _GAIN_RAMP_START_M) / (_GAIN_RAMP_END_M - _GAIN_RAMP_START_M)).clamp(0.0, 1.0)
    t = t * t * (3.0 - 2.0 * t)
    return gain_near + (_GAIN_FAR - gain_near) * t


def _is_tracked(pos: torch.Tensor, quat_wxyz: torch.Tensor, eps: float = 1e-6) -> bool:
    """True unless this is WebXR's "untracked hand" sentinel: zero position
    + identity orientation."""
    return not (
        torch.allclose(pos, torch.zeros_like(pos), atol=eps)
        and torch.allclose(quat_wxyz, torch.tensor([1.0, 0.0, 0.0, 0.0], device=quat_wxyz.device), atol=eps)
    )


def _pinch_dist(hand_joints: list) -> float:
    if len(hand_joints) != 75:
        return float("inf")
    ti, ii = _THUMB_TIP_IDX * 3, _INDEX_TIP_IDX * 3
    thumb = torch.tensor(hand_joints[ti:ti + 3])
    index = torch.tensor(hand_joints[ii:ii + 3])
    return (thumb - index).norm().item()


def _publish_real_left_arm_pose(pub, clock_node, joint_pos_des_rad) -> None:
    """Build and publish an ArmPose for the real left arm from a DLS
    solution. Field layout/units copied EXACTLY from
    Task_space_controller/robot_arm_controllers/task_space_real.py's
    publish_joint_pos -- same LEFT_ARM_JOINTS order (joint1L, joint2l,
    joint3l, joint4l, joint5l, joint6l) maps 1:1 to
    shoulder(flexion,abduction,rotation) / elbow(flexion,forearm_rotation) /
    wrist(extension), degrees (hardware_mapping.yaml + the CAN PositionDeg
    signal expect degrees, joint_pos_des is radians)."""
    import math

    from common_msgs.msg import ArmPose as WatoArmPose, JointState as WatoJointState

    q = [math.degrees(v) for v in joint_pos_des_rad[0].tolist()]

    msg = WatoArmPose()
    msg.header.stamp = clock_node.get_clock().now().to_msg()

    shoulder = WatoJointState()
    shoulder.position = [q[0], q[1], q[2]]
    msg.shoulder = shoulder

    elbow = WatoJointState()
    elbow.position = [q[3], q[4]]
    msg.elbow = elbow

    wrist = WatoJointState()
    wrist.position = [q[5]]
    msg.wrist = wrist

    pub.publish(msg)


def _clamp_step(target: torch.Tensor, previous: torch.Tensor, delta_max) -> torch.Tensor:
    """Direct port of joint_command_core.cpp's clampStep -- a generic
    rate-limit-toward-target. delta_max may be a scalar or a tensor matching
    target's shape."""
    return previous + torch.clamp(target - previous, -delta_max, delta_max)


def _adaptive_dls_lambda(
    jacobian: torch.Tensor, lambda_min: float, lambda_max: float, epsilon: float,
) -> tuple[float, float]:
    """Chiaverini adaptive damping for DLS IK -- see _DLS_LAMBDA_MAX's comment for why. Uses the
    SAME jacobian_b already computed in solve_and_apply, no extra IK-relevant computation.
    manipulability = sqrt(det(J J^T)) -- the standard scalar measure of how far the current pose
    is from a kinematic singularity (0 = exactly singular, larger = better-conditioned).
    Continuous ramp (not a hard threshold switch) from lambda_min at/above epsilon to lambda_max
    as manipulability -> 0, so damping changes smoothly frame-to-frame rather than jumping.
    Returns (lambda_val, manipulability)."""
    jjt = jacobian @ jacobian.transpose(-2, -1)
    manipulability = torch.sqrt(torch.clamp(torch.linalg.det(jjt), min=0.0)).item()
    if manipulability >= epsilon:
        return lambda_min, manipulability
    ratio = manipulability / epsilon
    lambda_val = math.sqrt(lambda_min**2 + (1.0 - ratio**2) * (lambda_max**2 - lambda_min**2))
    return lambda_val, manipulability


class _OneEuroFilter:
    """Adaptive low-pass filter for noisy human-input tracking data (Casiez, Roussel, Vogel,
    "1 Euro Filter", CHI 2012) -- standard technique for exactly this problem in production VR/AR
    systems (Kinect, VR controller hand tracking, etc.): smooths heavily when the signal is
    nearly still (kills tracking jitter) but relaxes to almost no smoothing during fast motion
    (adds ~no lag to real, deliberate movement) -- something a fixed-cutoff low-pass can't do,
    since it has to pick one fixed tradeoff between jitter and lag.

    This filters the RAW Quest wrist position BEFORE it drives the IK target, complementing
    _smooth_damp (which smooths the resulting joint COMMANDS after the IK solve). The two
    address different noise sources -- tracking-hardware jitter vs. actuator/solve smoothness --
    and using both together is standard practice, not redundant.
    """

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.5, d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff  # Hz -- baseline smoothing strength when nearly still
        self.beta = beta  # how aggressively speed relaxes the cutoff (higher = less lag, more residual jitter during fast motion)
        self.d_cutoff = d_cutoff  # Hz -- smooths the velocity estimate itself, so cutoff adaptation isn't itself noisy
        self.x_prev: torch.Tensor | None = None
        self.dx_prev: torch.Tensor | None = None

    @staticmethod
    def _alpha(cutoff, dt: float):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def filter(self, x: torch.Tensor, dt: float) -> torch.Tensor:
        if self.x_prev is None:
            self.x_prev = x.clone()
            self.dx_prev = torch.zeros_like(x)
            return x.clone()
        dt = max(dt, 1e-4)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        edx = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * edx.abs()
        a = self._alpha(cutoff, dt)
        x_filtered = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_filtered
        self.dx_prev = edx
        return x_filtered

    def reset(self) -> None:
        """Called on recalibration (R key) / scene reset (T key) so a teleport-style jump in
        raw position doesn't get smoothed into a slow drift -- the next filter() call reseeds
        state exactly like the first-ever call."""
        self.x_prev = None
        self.dx_prev = None


def _smooth_damp(
    current: torch.Tensor, current_vel: torch.Tensor, target: torch.Tensor,
    smooth_time: float, max_speed: float, dt: float,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Critically-damped spring smoothing (the same algorithm as Unity's Mathf.SmoothDamp,
    originally from Game Programming Gems 4's "Critically Damped Ease-In/Ease-Out Smoothing") --
    replaces the hard position/velocity _clamp_step with a continuous, physically-motivated
    filter. A hard clamp produces a bang-bang motion profile: full speed right up to the limit,
    then an instantaneous plateau -- exactly the kind of jerky motion professional VR teleop
    write-ups (ALOHA/GELLO-style analyses) flag as avoidable. This tracks both position AND
    velocity state so deceleration happens smoothly as the target is approached, rather than
    slamming into a rate ceiling every frame. max_speed is still enforced (same safety role
    _JOINT_VELOCITY_MAX_RAD_S always had), just as a smooth cap via this filter instead of a
    hard clamp.
    Returns (new_position, new_velocity) -- caller is responsible for carrying new_velocity
    into the next call (see _ArmDlsController.smoothed_vel)."""
    smooth_time = max(1e-4, smooth_time)
    omega = 2.0 / smooth_time

    x = omega * dt
    exp_val = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)  # stable exp(-omega*dt) approx, same as Unity's impl

    max_change = abs(max_speed) * smooth_time
    change = torch.clamp(current - target, -max_change, max_change)
    clamped_target = current - change

    temp = (current_vel + omega * change) * dt
    new_vel = (current_vel - omega * temp) * exp_val
    new_pos = clamped_target + (change + temp) * exp_val

    # Prevent overshoot: if we'd cross past the (pre-speed-clamp) target, snap to it instead.
    orig_to_current = target - current
    out_to_orig = new_pos - target
    overshot = (orig_to_current * out_to_orig) > 0
    new_pos = torch.where(overshot, target, new_pos)
    new_vel = torch.where(overshot, (target - current) / dt, new_vel)

    return new_pos, new_vel


def _step_velocity_and_delta_clamp(
    target: torch.Tensor, prev_pos: torch.Tensor, velocity_max: float, delta_max: float, dt: float,
) -> torch.Tensor:
    """Mirrors joint_command_core.cpp's non-trapezoidal moderation path:
    velocity-limit clamp (velocity_max converted to a per-frame step via dt)
    then a fixed delta-limit clamp, applied in that order."""
    velocity_step = abs(velocity_max) * dt
    stepped = _clamp_step(target, prev_pos, velocity_step)
    stepped = _clamp_step(stepped, prev_pos, abs(delta_max))
    return stepped


def _ee_pose_in_base(robot, body_id: int):
    root_pose_w = robot.data.root_state_w[:, 0:7]
    ee_pose_w = robot.data.body_state_w[:, body_id, 0:7]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )


class _ArmDlsController:
    """Bundles a DifferentialIKController with the per-arm state needed to
    drive it from Quest wrist data: entity/jacobian indices, fingertip
    geometry, homing state, and the current target. One instance per arm —
    kept separate (rather than sharing one controller with num_envs=2) so
    neither arm's homing/recalibration state can leak into the other's.
    """

    def __init__(self, scene, robot, device, arm_joint_names, ee_body, finger_tip_bodies, finger_distal_local,
                 lambda_val=_DLS_LAMBDA, lambda_max=_DLS_LAMBDA_MAX):
        self.finger_tip_bodies = finger_tip_bodies
        self.finger_distal_local = finger_distal_local

        entity_cfg = _make_entity_cfg(scene, arm_joint_names, ee_body)
        self.body_id = entity_cfg.body_ids[0]
        self.arm_ids = entity_cfg.joint_ids
        self.ee_jacobi_idx = self.body_id - 1 if robot.is_fixed_base else self.body_id
        self.finger_body_ids = resolve_body_ids(robot, finger_tip_bodies)

        cfg = DifferentialIKControllerCfg(
            command_type="pose", use_relative_mode=False, ik_method="dls",
            ik_params={"lambda_val": lambda_val},
        )
        self.controller = DifferentialIKController(cfg, num_envs=scene.num_envs, device=device)
        self.controller.reset(env_ids=torch.arange(scene.num_envs, device=device))
        # lambda_val above is now the MINIMUM (floor) damping -- see _adaptive_dls_lambda, called
        # each solve_and_apply to raise self.controller.cfg.ik_params["lambda_val"] toward
        # lambda_max as manipulability drops near a singularity.
        self.lambda_min = lambda_val
        self.lambda_max = lambda_max
        self.last_manipulability: float | None = None

        self.quest_home_xyz: torch.Tensor | None = None
        self.quest_home_quat: torch.Tensor | None = None
        self.home_tip_pos_b: torch.Tensor | None = None
        self.home_tip_quat_b: torch.Tensor | None = None
        self.wrist_orient_offset: torch.Tensor | None = None
        # Latest DLS solution in arm_joint_names order, radians -- read by the
        # optional real-hardware bridge (see --publish-real-left-arm) to build
        # ArmPose messages. Deliberately the RAW (pre-ramp) solution, not the
        # trapezoidal-smoothed one applied to the sim robot below -- the real
        # arm runs its own independent smoothing in joint_command_core.cpp,
        # so double-smoothing here would just add extra bridge latency.
        self.last_joint_pos_des: torch.Tensor | None = None
        # Smoothing state (see _smooth_damp) -- None until the first solve_and_apply call,
        # which seeds smoothed_pos from the robot's actual current joint pose and
        # smoothed_vel to zero.
        self.smoothed_pos: torch.Tensor | None = None
        self.smoothed_vel: torch.Tensor | None = None
        # Filters the RAW Quest wrist position before it's used for homing/displacement -- see
        # _OneEuroFilter's docstring for why this is separate from smoothed_pos/smoothed_vel
        # above (input-tracking-jitter filtering vs. output-joint-command smoothing).
        self.pos_filter = _OneEuroFilter()

    def tip_pose_b(self, robot, root_pose_w):
        return compute_gripper_tip_pose_b(
            robot, root_pose_w, self.body_id, self.finger_body_ids,
            self.finger_tip_bodies, self.finger_distal_local,
        )

    def solve_and_apply(self, robot, device, target_pos_b, target_quat_b, dt: float):
        root_pose_w = robot.data.root_state_w[:, 0:7]
        wrist_pos_b, _ = _ee_pose_in_base(robot, self.body_id)
        tip_pos_b, tip_quat_b = self.tip_pose_b(robot, root_pose_w)
        jacobian_w = robot.root_physx_view.get_jacobians()[:, self.ee_jacobi_idx, :, self.arm_ids]
        jacobian_b = compute_tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b)

        # Adaptive DLS damping (see _adaptive_dls_lambda) -- raises the controller's damping
        # only when this frame's pose is actually close to a singularity, instead of always
        # applying the fixed floor value. Mutates the SAME cfg object compute() reads below
        # (DifferentialIKController re-reads cfg.ik_params["lambda_val"] every call, confirmed
        # by reading its source -- no need to reconstruct the controller).
        adaptive_lambda, self.last_manipulability = _adaptive_dls_lambda(
            jacobian_b, self.lambda_min, self.lambda_max, _DLS_MANIPULABILITY_EPSILON,
        )
        self.controller.cfg.ik_params["lambda_val"] = adaptive_lambda

        self.controller.set_command(torch.cat([target_pos_b, target_quat_b], dim=1))
        joint_pos = robot.data.joint_pos[:, self.arm_ids]
        joint_pos_des = self.controller.compute(tip_pos_b, tip_quat_b, jacobian_b, joint_pos)
        self.last_joint_pos_des = joint_pos_des

        # Critically-damped smoothing (see _smooth_damp) instead of a hard clamp -- per live
        # request, after researching how professional VR teleop systems (ALOHA/GELLO-style
        # analyses) achieve smooth motion: a hard clamp produces bang-bang motion (full speed
        # until the limit, then an instant plateau), which reads as jerky and is a plausible
        # contributor to the joint-jitter previously seen in recorded demos. _JOINT_VELOCITY_MAX_RAD_S
        # (100deg/s) is still the enforced speed ceiling, just applied smoothly now.
        if self.smoothed_pos is None:
            self.smoothed_pos = joint_pos.clone()
            self.smoothed_vel = torch.zeros_like(joint_pos)
        self.smoothed_pos, self.smoothed_vel = _smooth_damp(
            self.smoothed_pos, self.smoothed_vel, joint_pos_des,
            _SMOOTH_TIME_S, abs(_JOINT_VELOCITY_MAX_RAD_S), dt,
        )
        robot.set_joint_position_target(self.smoothed_pos, joint_ids=self.arm_ids)
        robot.write_joint_state_to_sim(
            self.smoothed_pos,
            torch.zeros_like(self.smoothed_pos),
            joint_ids=self.arm_ids,
        )
        return tip_pos_b, tip_quat_b


# ── recording (see dataset_schema_wato_arm_v2_push_box.yaml) ─────────────────

def _init_recorder(device: str):
    """Mirrors so101_leader_teleop.py's _init_recorder. Returns (recorder, schema_cfg)
    or (None, None) if --record wasn't passed."""
    if not args_cli.record:
        return None, None
    _ensure_il_on_path()
    try:
        from humanoid_il.record_utils import resolve_config_path
        from humanoid_il.schema import enabled_images, load_yaml
        from humanoid_il.sim_recorder import SimLeRobotRecorder
    except ImportError as exc:
        raise ImportError("Recording requires humanoid-il. Install with:\n  pip install -e autonomy/il[sim]") from exc

    schema_path = resolve_config_path(args_cli.schema, anchor=_IL_PKG)
    cfg = load_yaml(schema_path)
    dataset_root = (
        Path(args_cli.dataset_root) if args_cli.dataset_root
        else Path((cfg.get("record") or {}).get("root", "datasets/record_wato_arm_v2_push_box"))
    )
    cameras = {name: {"height": spec["height"], "width": spec["width"]} for name, spec in enabled_images(cfg).items()}
    # Always "cpu" here, regardless of the physics device -- SimLeRobotRecorder allocates a
    # buffer_capacity_s circular buffer of full-res RGB frames per camera on whatever device
    # it's given (multiple GB for 2 cameras), and _capture_record_images already converts frames
    # to CPU numpy before handing them to the recorder anyway. Passing the GPU physics device
    # (--device cuda) through here caused a hard CUDA OutOfMemoryError crash live -- the recorder's
    # buffer allocation competed with Isaac Sim's own rendering/physics GPU memory on a 7.5GB GPU
    # and lost, killing the whole process silently in the background (this is what showed up as
    # "teleop just did not work" -- the process had already died).
    #
    # buffer_capacity_s explicitly set to 30 (was defaulting to 120) -- root-caused as the actual
    # source of the "headset fps is ass" + host RAM/swap-thrashing problem (which had already twice
    # caused this exact process to hang entirely): with 2 cameras at 640x480 and the recorder's
    # _NUM_CPU_SLOTS=2 pinned-memory pool (see sim_recorder.py's _allocate_cpu_slots), the buffer
    # alone is (1 main + 2 pinned slots) x 120s x 30fps x 640x480x3 bytes x 2 cameras ~= 20GB RSS,
    # on a 30GB host -- confirmed live: idle RSS with no recording is a flat 7.3GB (sampled over
    # 90s, no growth), but climbs to ~24-27GB once recording starts, matching this calculation.
    # Actual recorded episodes here run ~15-20s (S/D are meant for FAST short demos) -- 30s is
    # generous headroom over that while cutting the buffer to ~1/4 size (~5GB). If an episode ever
    # runs past 30s, push_frame_to_buffer degrades safely (prints "[WARN]: Buffer full, skipping"
    # and drops further frames) rather than crashing -- raise this back up only if demos genuinely
    # need to run longer than ~25s.
    recorder = SimLeRobotRecorder(
        task_name=args_cli.task_description,
        repo_id=str(cfg.get("repo_id", "humanoid/wato_arm_v2_push_box")),
        dataset_root=dataset_root,
        fps=int(cfg.get("fps", 30)),
        device="cpu",
        joint_names=list(cfg["joint_names"]),
        cameras=cameras,
        num_episodes=args_cli.num_episodes,
        buffer_capacity_s=30.0,
    )
    recorder.init_dataset()
    print(f"[RECORD] Writing to {dataset_root}", flush=True)
    # NOT humanoid_il's default S=start/N=save/D=discard keys -- this script auto-starts
    # recording on VR connect and rebinds S/D itself (see run_simulator's _on_keyboard_event) to
    # save/discard-and-continue instead. The "Press S..." message printed later reflects the
    # actual bindings; this generic init message intentionally doesn't restate stale ones here.
    return recorder, cfg


def _capture_record_images(scene: InteractiveScene) -> dict:
    """{schema_key: HxWx3 uint8 rgb} for the two data-collection cameras --
    NOT the RSD455 headset pair, see module docstring."""
    images = {}
    for schema_key, scene_key in (("ego", "ego_cam"), ("wrist", "wrist_cam")):
        rgb = scene[scene_key].data.output["rgb"]
        frame = rgb[0].detach().cpu().numpy()
        if frame.shape[-1] > 3:
            frame = frame[..., :3]
        images[schema_key] = frame.astype("uint8")
    return images


def _write_wrist_cam_hud_frame(scene: InteractiveScene) -> None:
    """Write wrist_cam's current sensor frame to _WRIST_CAM_FRAME_PATH for the headset HUD
    overlay -- reads the same CameraCfg sensor tensor recording uses, no extra viewport render."""
    from PIL import Image

    rgb = scene["wrist_cam"].data.output["rgb"]
    frame = rgb[0].detach().cpu().numpy()
    if frame.shape[-1] > 3:
        frame = frame[..., :3]
    Image.fromarray(frame.astype("uint8")).save(str(_WRIST_CAM_FRAME_PATH))


# ── main simulation loop ──────────────────────────────────────────────────────

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene) -> None:
    robot = scene["robot"]
    device = sim.device
    sim_dt = sim.get_physics_dt()
    gain = args_cli.gain

    scene.update(sim_dt)
    apply_joint_limits(robot)

    _root_pose_w_diag = robot.data.root_state_w[0, :7].tolist()
    print(f"[Quest][diag] robot root world pose (pos xyz, quat wxyz)={_root_pose_w_diag}", flush=True)

    _base_link_prim_path = "/World/envs/env_0/Robot/base_link"  # num_envs is always 1 in this script
    _base_link_prim = omni.usd.get_context().get_stage().GetPrimAtPath(_base_link_prim_path)
    print(f"[Quest] base_link prim valid={_base_link_prim.IsValid()} "
          f"authored={_base_link_prim.HasAuthoredReferences() or _base_link_prim.GetTypeName() != ''} "
          f"body_names={list(robot.data.body_names)}", flush=True)
    left_eye_mount = _attach_rsd455_camera(
        _base_link_prim_path, "left_eye_camera_mount", _HEAD_VIEWPOINT_HOME_POS, _HEAD_VIEWPOINT_HOME_QUAT
    )
    right_eye_mount = _attach_rsd455_camera(
        _base_link_prim_path, "right_eye_camera_mount", _HEAD_VIEWPOINT_HOME_POS, _HEAD_VIEWPOINT_HOME_QUAT
    )
    print(f"[Quest] Stereo head-tracked RealSense D455 pair attached: {left_eye_mount}, {right_eye_mount}", flush=True)
    # Read the RSD455's real, native (unwidened) FOV BEFORE _widen_camera_fov
    # changes it for the teleop display below -- this is what actually gets
    # copied onto ego_cam, so recorded frames match a real D455's true FOV.
    _rsd455_native_fov = _read_camera_fov(f"{left_eye_mount}/{_RSD455_CAMERA_SUBPATH}")
    _widen_camera_fov(f"{left_eye_mount}/{_RSD455_CAMERA_SUBPATH}", _RSD455_WIDENED_FOCAL_LENGTH)
    _widen_camera_fov(f"{right_eye_mount}/{_RSD455_CAMERA_SUBPATH}", _RSD455_WIDENED_FOCAL_LENGTH)
    left_eye_viewport_api = _open_pov_viewport(f"{left_eye_mount}/{_RSD455_CAMERA_SUBPATH}", "Left Eye POV")
    right_eye_viewport_api = _open_pov_viewport(f"{right_eye_mount}/{_RSD455_CAMERA_SUBPATH}", "Right Eye POV")
    # Cached once here (not re-fetched every frame) for _project_world_point_to_uv -- valid
    # because _widen_camera_fov above already blocked until these prims finished loading.
    _stage_for_cams = omni.usd.get_context().get_stage()
    _left_eye_cam_prim = _stage_for_cams.GetPrimAtPath(f"{left_eye_mount}/{_RSD455_CAMERA_SUBPATH}")
    _right_eye_cam_prim = _stage_for_cams.GetPrimAtPath(f"{right_eye_mount}/{_RSD455_CAMERA_SUBPATH}")
    # Data-collection camera (ego_cam) -- separate from the teleop headset
    # display above. Preview window only, for visually checking framing;
    # not used by recording (that reads scene["ego_cam"] directly).
    _ego_cam_prim_path = f"{_base_link_prim_path}/ego_cam"
    if _rsd455_native_fov is not None:
        # Widened again (matches the headset eye views' _RSD455_WIDENED_FOCAL_LENGTH, ~120deg)
        # per final live feedback -- native FOV felt too zoomed in for actual use even with the
        # clippingRange bug fixed.
        _ego_cam_fov = (_RSD455_WIDENED_FOCAL_LENGTH, _rsd455_native_fov[1], _rsd455_native_fov[2])
        _set_camera_fov(_ego_cam_prim_path, *_ego_cam_fov)
        print(f"[Quest] ego_cam FOV set to WIDENED RSD455 (not native) "
              f"(focalLength/hAperture/vAperture)={_ego_cam_fov}", flush=True)
    # TEMP DIAGNOSTIC: commented out to test whether these 2 extra viewport windows (pure
    # operator convenience -- recording reads scene["ego_cam"]/["wrist_cam"] sensor tensors
    # directly, not these windows) are a meaningful chunk of the measured ~200ms/render cost,
    # independent of render_interval (which is an IsaacLab/physics-step concept -- these are
    # separate Kit UI windows that may render on every app tick regardless of that setting).
    # _open_pov_viewport(_ego_cam_prim_path, "Ego Cam Preview")
    # _open_pov_viewport("/World/envs/env_0/Robot/link6l/wrist_cam", "Wrist Cam Preview")
    if left_eye_viewport_api is not None and right_eye_viewport_api is not None:
        _POV_STATIC_DIR.mkdir(parents=True, exist_ok=True)
        print(f"[Quest] Stereo POV feed will be captured to {_POV_FRAME_PATH_LEFT} / {_POV_FRAME_PATH_RIGHT} "
              f"(served by webxr_server.py's static handler at /pov_left.png, /pov_right.png)", flush=True)

    left_arm_names = [resolve_joint_name(robot, n) for n in LEFT_ARM_JOINTS]
    right_arm_names = [resolve_joint_name(robot, n) for n in _RIGHT_ARM_JOINTS]

    left_arm = _ArmDlsController(
        scene, robot, device, left_arm_names, LEFT_EE_BODY,
        LEFT_FINGER_TIP_BODIES, LEFT_FINGER_DISTAL_TIP_LOCAL,
    )
    right_arm = _ArmDlsController(
        scene, robot, device, right_arm_names, RIGHT_EE_BODY,
        RIGHT_FINGER_TIP_BODIES, RIGHT_FINGER_DISTAL_TIP_LOCAL,
        lambda_val=_DLS_LAMBDA_RIGHT, lambda_max=_DLS_LAMBDA_MAX_RIGHT,
    )

    quest_to_world = _QUEST_TO_WORLD.to(device)
    quest_to_world_quat = quat_from_matrix(quest_to_world.unsqueeze(0))  # RETIRED for orientation, see comment above _QUEST_TO_WORLD
    quest_to_cam_local = _QUEST_TO_CAM_LOCAL.to(device)
    quest_to_cam_local_quat = quat_from_matrix(quest_to_cam_local.unsqueeze(0))
    axis_sign_left = _AXIS_SIGN_LEFT.to(device)
    axis_sign_right = _AXIS_SIGN_RIGHT.to(device)
    # Camera-relative basis vectors (world frame), built by rotating the
    # rsd455's local forward/up/right axes by the camera's fixed mount tilt
    # (_HEAD_VIEWPOINT_HOME_QUAT). Hand position deltas are composed directly
    # along these instead of fixed world axes -- see _AXIS_SIGN_LEFT/RIGHT's
    # comment above for why.
    camera_tilt_quat = torch.tensor([_HEAD_VIEWPOINT_HOME_QUAT], dtype=torch.float32, device=device)
    cam_fwd_world = quat_apply(camera_tilt_quat, _CAM_LOCAL_FORWARD.to(device).unsqueeze(0)).squeeze(0)
    cam_up_world = quat_apply(camera_tilt_quat, _CAM_LOCAL_UP.to(device).unsqueeze(0)).squeeze(0)
    cam_right_world = quat_apply(camera_tilt_quat, _CAM_LOCAL_RIGHT.to(device).unsqueeze(0)).squeeze(0)
    left_arm.wrist_orient_offset = _WRIST_ORIENT_OFFSET_LEFT.to(device).unsqueeze(0)
    right_arm.wrist_orient_offset = _WRIST_ORIENT_OFFSET_RIGHT.to(device).unsqueeze(0)

    left_gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    right_gripper_ids = _joint_ids(robot, _RIGHT_GRIPPER_JOINTS)

    left_g_open = torch.tensor([[GRIPPER_OPEN["joint7l"], GRIPPER_OPEN["joint8l"]]], device=device)
    left_g_closed = torch.tensor([[GRIPPER_CLOSED["joint7l"], GRIPPER_CLOSED["joint8l"]]], device=device)
    right_g_open = torch.tensor([[_RIGHT_GRIPPER_OPEN["joint7"], _RIGHT_GRIPPER_OPEN["joint8"]]], device=device)
    right_g_closed = torch.tensor([[_RIGHT_GRIPPER_CLOSED["joint7"], _RIGHT_GRIPPER_CLOSED["joint8"]]], device=device)
    # Smooths the gripper's open/closed target the same way _smooth_damp now smooths the arm
    # joints, per live feedback + industry-practice research -- the pinch-threshold open/closed
    # DECISION stays binary (a legitimate, common approach per that research), but without this
    # the resulting joint TARGET was applied via a raw set_joint_position_target snap, an
    # instant jump every time the pinch crossed the threshold -- a visible discontinuity in
    # recorded demos inconsistent with how smoothly the arm itself now moves.
    left_gripper_smoothed = left_g_open.clone()
    left_gripper_vel = torch.zeros_like(left_g_open)
    right_gripper_smoothed = right_g_open.clone()
    right_gripper_vel = torch.zeros_like(right_g_open)

    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    root_pose_w0 = robot.data.root_state_w[:, 0:7]
    init_tip_pos_b_l, init_tip_quat_b_l = left_arm.tip_pose_b(robot, root_pose_w0)
    init_tip_pos_b_r, init_tip_quat_b_r = right_arm.tip_pose_b(robot, root_pose_w0)
    print(f"[Quest][diag] rest EE pos (base frame, from FK of default joint pose) "
          f"L(tip)={init_tip_pos_b_l[0].tolist()} R(tip)={init_tip_pos_b_r[0].tolist()}", flush=True)
    init_tip_pos_b_l[:, 0] += _HOME_TIP_X_OFFSET
    init_tip_pos_b_r[:, 0] += _HOME_TIP_X_OFFSET
    init_tip_pos_b_l[:, 2] += _HOME_TIP_Z_OFFSET
    init_tip_pos_b_r[:, 2] += _HOME_TIP_Z_OFFSET
    print(f"[Quest][diag] rest EE pos AFTER _HOME_TIP_X_OFFSET={_HOME_TIP_X_OFFSET}/"
          f"_HOME_TIP_Z_OFFSET={_HOME_TIP_Z_OFFSET} (IK target, not yet solved) "
          f"L(tip)={init_tip_pos_b_l[0].tolist()} R(tip)={init_tip_pos_b_r[0].tolist()}", flush=True)

    rclpy.init()
    receiver = QuestRosReceiver()
    threading.Thread(target=rclpy.spin, args=(receiver,), daemon=True).start()

    # Real-hardware bridge (left arm only, see --publish-real-left-arm help
    # text and the module-level REAL_ARM_PUBLISH_* constants for the safety
    # rationale). Publisher is created on the SAME node/spin-thread as the
    # Quest receiver above rather than a second rclpy context.
    real_left_arm_pub = None
    real_left_arm_elapsed_s = 0.0
    real_left_arm_since_publish_s = 0.0
    real_left_arm_started = False
    if args_cli.publish_real_left_arm:
        from common_msgs.msg import ArmPose as WatoArmPose, JointState as WatoJointState

        real_left_arm_pub = receiver.create_publisher(WatoArmPose, "/behaviour/arm_pose", 10)
        print(f"[Quest][REAL HARDWARE] Left arm will start publishing to /behaviour/arm_pose in "
              f"{_REAL_ARM_PUBLISH_START_DELAY_S:.0f}s. Position the REAL left arm near the sim's "
              f"rest pose NOW. Right arm is NOT published (no hardware CAN mapping exists).", flush=True)

    # Targets applied every frame (unconditionally), updated only while tracked.
    target_pos_b_left = init_tip_pos_b_l.clone()
    target_quat_b_left = init_tip_quat_b_l.clone()
    target_pos_b_right = init_tip_pos_b_r.clone()
    target_quat_b_right = init_tip_quat_b_r.clone()
    # Only reassigned inside "if msg is not None:" below, but read unconditionally every frame
    # (marker UV projection) -- initialized here so that holds even before the first message.
    root_quat_w = robot.data.root_state_w[:, 3:7]

    # Head-tracked stereo viewpoint state -- same home-on-first-tracked-sample
    # pattern as the arms above, homed independently (a person can start
    # moving their head before their wrists are tracked or vice versa).
    head_home_xyz: torch.Tensor | None = None
    head_home_quat: torch.Tensor | None = None
    head_home_viewpoint_pos_b = torch.tensor([_HEAD_VIEWPOINT_HOME_POS], dtype=torch.float32, device=device)
    head_home_viewpoint_quat_b = torch.tensor([_HEAD_VIEWPOINT_HOME_QUAT], dtype=torch.float32, device=device)
    head_viewpoint_pos_b = head_home_viewpoint_pos_b.clone()
    head_viewpoint_quat_b = head_home_viewpoint_quat_b.clone()
    eye_local_right = _EYE_LOCAL_RIGHT.to(device)
    eye_local_forward = _EYE_LOCAL_FORWARD.to(device)
    ego_cam_convention_fix_quat = torch.tensor([_EGO_CAM_CONVENTION_FIX_QUAT], dtype=torch.float32, device=device)
    ego_cam_extra_tilt_quat = torch.tensor([_EGO_CAM_EXTRA_TILT_QUAT], dtype=torch.float32, device=device)

    left_closed = False
    right_closed = False

    recorder, record_cfg = _init_recorder(device)
    if recorder is not None:
        # NOT recorder.start_keyboard() -- that attaches humanoid_il's shared pynput-based
        # S=start/N=save/D=discard listener (used by other teleop scripts too, so its bindings
        # aren't ours to repurpose). S/D are handled below instead, through the SAME carb.input
        # listener already driving R/T, with different semantics: since recording auto-starts on
        # VR connect and runs continuously, S/D call save_episode()/cancel_recording() directly
        # (both safe to call standalone, no flags dependency) to commit/discard the current
        # buffer and immediately keep recording the next one -- no separate "resume" step needed,
        # for fast back-to-back demos. EpisodeFlags is still needed (tick() no-ops if
        # self._flags is None) but constructed directly instead of via start_keyboard().
        from humanoid_il.episode_keys import EpisodeFlags
        recorder._flags = EpisodeFlags(start=False)
        # SimLeRobotRecorder saves episodes ASYNCHRONOUSLY on a background thread --
        # pressing 'N' only enqueues the save, it doesn't complete it immediately. A hard
        # kill (SIGKILL, e.g. `pkill -9`) of this process terminates that thread mid-flight
        # and silently loses the episode, even though it looked saved. This handler makes
        # SIGTERM (a normal `kill`/`pkill` without -9) finalize the recorder first --
        # `finalize()` blocks until the save queue is actually drained -- before exiting.
        # SIGKILL still can't be caught (that's an OS-level guarantee), so this only helps
        # if whatever stops the process sends SIGTERM, not SIGKILL.
        def _graceful_shutdown_on_signal(signum, _frame):
            print(f"[RECORD] Caught signal {signum} -- finalizing recorder before exit "
                  f"(do not force-kill, this can take a few seconds to drain the save queue)...",
                  flush=True)
            recorder.finalize()
            print(f"[RECORD] Finalized. Saved under {recorder.dataset_root}", flush=True)
            sys.exit(0)

        signal.signal(signal.SIGTERM, _graceful_shutdown_on_signal)
        signal.signal(signal.SIGINT, _graceful_shutdown_on_signal)

    def _sphere_cfg(color, radius, opacity=1.0):
        return sim_utils.SphereCfg(
            radius=radius,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, opacity=opacity),
        )

    left_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/left_ik_target",
        markers={"sphere": _sphere_cfg((0.2, 0.5, 1.0), 0.025, opacity=0.3)},  # was 0.05, halved per live feedback
    ))
    right_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/right_ik_target",
        markers={"sphere": _sphere_cfg((1.0, 0.3, 0.1), 0.025, opacity=0.3)},  # was 0.05, halved per live feedback
    ))
    if args_cli.record:
        # VisualizationMarkers creates its sphere instance(s) at init time regardless of whether
        # .visualize() is ever called (defaults to position (0,0,0), NOT hidden) -- skipping the
        # per-frame .visualize() call below is NOT enough on its own to guarantee it's not
        # rendered. USD "visibility" (unlike the cameraVisibility collection that turned out to
        # not be honored by this RTX renderer at all) IS a standard, always-respected attribute --
        # force both markers fully invisible here as the actual guarantee.
        _stage_for_markers = omni.usd.get_context().get_stage()
        for _mp in ("/Visuals/left_ik_target", "/Visuals/right_ik_target"):
            _mprim = _stage_for_markers.GetPrimAtPath(_mp)
            if _mprim.IsValid():
                UsdGeom.Imageable(_mprim).MakeInvisible()
        print("[Quest] --record is active: IK target markers made invisible (guaranteed, not "
              "just unpositioned) to keep them out of recorded frames.", flush=True)

    # Hide BOTH IK target markers (blue=left, red=right -- "the balls") from wrist_cam's
    # RECORDED render specifically -- root cause finally found: CameraCfg sensors (used for
    # recording) are built on omni.replicator.core render products (rep.create.render_product),
    # a completely separate pipeline from Kit's interactive Hydra viewports. Per the USD Render
    # spec, the "cameraVisibility" collection belongs on the RenderProduct prim itself (under
    # /Render/..., created dynamically by Replicator), NOT on the camera prim -- applying it to
    # the camera prim (the previous two attempts) silently did nothing because renderers only
    # look for this collection on actual RenderProduct prims. Fixed by applying it to
    # scene["wrist_cam"].render_product_paths instead. purpose=guide on the markers is kept too
    # (harmless, doesn't hurt) but is NOT what's doing the real work here.
    _stage = omni.usd.get_context().get_stage()
    for _marker_path in ("/Visuals/left_ik_target", "/Visuals/right_ik_target"):
        _marker_prim = _stage.GetPrimAtPath(_marker_path)
        if _marker_prim.IsValid():
            UsdGeom.Imageable(_marker_prim).GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    for _rp_path in scene["wrist_cam"].render_product_paths:
        _rp_prim = _stage.GetPrimAtPath(_rp_path)
        if _rp_prim.IsValid():
            _cam_vis_collection = Usd.CollectionAPI.Apply(_rp_prim, "cameraVisibility")
            _cam_vis_collection.CreateIncludeRootAttr().Set(True)  # "/" isn't a valid rel target
            _excludes_rel = _cam_vis_collection.CreateExcludesRel()
            _excludes_rel.AddTarget("/Visuals/left_ik_target")
            _excludes_rel.AddTarget("/Visuals/right_ik_target")
            print(f"[Quest] Excluded both IK target markers from wrist_cam's RenderProduct "
                  f"({_rp_path}) cameraVisibility", flush=True)
        else:
            print(f"[Quest] WARNING: wrist_cam RenderProduct prim {_rp_path} not valid -- "
                  f"marker exclusion did not apply", flush=True)

    def _recalibrate() -> None:
        nonlocal head_home_xyz, head_home_quat
        left_arm.quest_home_xyz = None
        right_arm.quest_home_xyz = None
        head_home_xyz = None
        head_home_quat = None
        # Otherwise the filter's stale x_prev (from before recalibration) makes the next real
        # sample look like a huge, fast motion -- momentarily defeating its own jitter smoothing
        # right when a fresh, clean home anchor is what's needed most.
        left_arm.pos_filter.reset()
        right_arm.pos_filter.reset()
        print("[Quest] Recalibrating — hold wrists in a comfortable pose.", flush=True)

    def _reset_scene() -> None:
        """Full scene reset (distinct from R's recalibration, which only re-anchors tracking):
        box/container back to their original spawn pose+velocity, arm joints back to the default
        rest pose, IK targets back to the rest tip pose (+ offsets), grippers back to open, and
        tracking re-homed (same as R) so the arm doesn't immediately snap back toward wherever
        your hand currently is."""
        nonlocal target_pos_b_left, target_quat_b_left, target_pos_b_right, target_quat_b_right
        nonlocal left_closed, right_closed
        nonlocal left_gripper_smoothed, left_gripper_vel, right_gripper_smoothed, right_gripper_vel
        scene["box"].write_root_state_to_sim(scene["box"].data.default_root_state.clone())
        scene["container"].write_root_state_to_sim(scene["container"].data.default_root_state.clone())
        robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
        target_pos_b_left = init_tip_pos_b_l.clone()
        target_quat_b_left = init_tip_quat_b_l.clone()
        target_pos_b_right = init_tip_pos_b_r.clone()
        target_quat_b_right = init_tip_quat_b_r.clone()
        left_arm.smoothed_pos = None
        right_arm.smoothed_pos = None
        left_closed = False
        right_closed = False
        # Snap (not smoothly drift) to open on reset -- matches the arm/box/container all being
        # hard-reset too, not eased into their reset state.
        left_gripper_smoothed = left_g_open.clone()
        left_gripper_vel = torch.zeros_like(left_g_open)
        right_gripper_smoothed = right_g_open.clone()
        right_gripper_vel = torch.zeros_like(right_g_open)
        _recalibrate()
        print("[Quest] Scene reset -- box/container respawned, arm back to rest pose.", flush=True)

    def _on_keyboard_event(event, *args, **kwargs) -> None:
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return
        if event.input.name == "R":
            _recalibrate()
        elif event.input.name == "T":
            _reset_scene()
        elif event.input.name == "S" and recorder is not None:
            recorder.save_episode()
            print("[RECORD] [S] Episode saved -- recording continues for the next one.", flush=True)
        elif event.input.name == "D" and recorder is not None:
            recorder.cancel_recording()
            print("[RECORD] [D] Episode discarded -- recording continues for the retry.", flush=True)

    _keyboard_iface = carb.input.acquire_input_interface()
    _keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    _keyboard_sub = _keyboard_iface.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

    print("[Quest] Ready. Waiting for /quest_teleop messages.", flush=True)
    print("[Quest] Both arms: Differential IK (DLS), fingertip-tip target.", flush=True)
    print("[Quest] Connect the Quest browser to start streaming hand data.", flush=True)
    print("[Quest] Press R (Isaac Sim window focused) to recalibrate to your current pose.", flush=True)
    if recorder is not None:
        print("[Quest] Press S to save the current episode, D to discard it -- recording "
              "continues either way for the next demo.", flush=True)
    print("[Quest] Press T (Isaac Sim window focused) to fully reset the scene (box/container/arm).", flush=True)

    diag_frame = 0
    pov_capture_frame = 0
    _vr_connected = False
    _fps_window_start_t = time.monotonic()
    _fps_window_frames = 0
    _fps_window_capture_ms = 0.0
    while simulation_app.is_running():
        msg = receiver.poll()

        if msg is not None:
            if not _vr_connected:
                _vr_connected = True
                print("[Quest] VR connected (first /quest_teleop message received).", flush=True)
                # Auto-start recording (if --record) the moment the headset connects to
                # localhost, rather than waiting for hand tracking or a manual 'S' keypress
                # on the host keyboard -- recorder.tick() still checks flags.start every
                # frame, so this just flips it on early. Keyboard N (save)/D (discard)/Esc
                # (stop) still work normally afterward.
                if recorder is not None and recorder._flags is not None and not recorder._flags.start:
                    recorder._flags.start = True
                    print("[RECORD] VR connected -- recording started automatically.", flush=True)
            left_xyz_q = _wrist_xyz(msg.left_wrist).to(device)
            right_xyz_q = _wrist_xyz(msg.right_wrist).to(device)
            left_quat = _wrist_quat_wxyz(msg.left_wrist).to(device)
            right_quat = _wrist_quat_wxyz(msg.right_wrist).to(device)
            head_xyz_q = _wrist_xyz(msg.head_pose).to(device)
            head_quat = _wrist_quat_wxyz(msg.head_pose).to(device)
            left_joints = list(msg.left_hand_joints)
            right_joints = list(msg.right_hand_joints)

            left_tracked = _is_tracked(left_xyz_q, left_quat)
            right_tracked = _is_tracked(right_xyz_q, right_quat)
            head_tracked = _is_tracked(head_xyz_q, head_quat)

            # Filter raw tracking noise out of wrist position BEFORE homing/displacement use it
            # (see _OneEuroFilter) -- only while actually tracked, so the untracked-sentinel
            # (0,0,0) never gets fed into the filter's state.
            if left_tracked:
                left_xyz_q = left_arm.pos_filter.filter(left_xyz_q, sim_dt)
            if right_tracked:
                right_xyz_q = right_arm.pos_filter.filter(right_xyz_q, sim_dt)

            root_quat_w = robot.data.root_state_w[:, 3:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]
            tip_pos_b_l, _ = left_arm.tip_pose_b(robot, root_pose_w)
            tip_pos_b_r, _ = right_arm.tip_pose_b(robot, root_pose_w)

            # ── Homing (fingertip-tip anchored) ─────────────────────────────────
            if left_arm.quest_home_xyz is None and left_tracked:
                left_arm.quest_home_xyz = left_xyz_q.clone()
                left_arm.quest_home_quat = left_quat.clone()
                # Anchor to the fixed launch-time rest tip pose, not wherever
                # the tip currently sits -- avoids baking in IK lag / making
                # recalibration a no-op.
                left_arm.home_tip_pos_b = init_tip_pos_b_l.clone()
                left_arm.home_tip_quat_b = init_tip_quat_b_l.clone()
                target_pos_b_left = left_arm.home_tip_pos_b.clone()
                target_quat_b_left = left_arm.home_tip_quat_b.clone()
                print(f"[Quest] Left wrist tracked — homed at {left_xyz_q.tolist()}", flush=True)

            if right_arm.quest_home_xyz is None and right_tracked:
                right_arm.quest_home_xyz = right_xyz_q.clone()
                right_arm.quest_home_quat = right_quat.clone()
                right_arm.home_tip_pos_b = init_tip_pos_b_r.clone()
                right_arm.home_tip_quat_b = init_tip_quat_b_r.clone()
                target_pos_b_right = right_arm.home_tip_pos_b.clone()
                target_quat_b_right = right_arm.home_tip_quat_b.clone()
                print(f"[Quest] Right wrist tracked — homed at {right_xyz_q.tolist()}", flush=True)

            # ── Target (fingertip-tip, base frame) ──────────────────────────────
            if left_arm.quest_home_xyz is not None and left_tracked:
                left_disp_raw = left_xyz_q - left_arm.quest_home_xyz
                left_gain = _ramped_gain(left_disp_raw.norm(), gain)
                left_right_amt = left_disp_raw[0] * axis_sign_left[0]
                left_up_amt = left_disp_raw[1] * axis_sign_left[1]
                left_fwd_amt = -left_disp_raw[2] * axis_sign_left[2]
                left_delta_w = (
                    left_right_amt * cam_right_world
                    + left_up_amt * cam_up_world
                    + left_fwd_amt * cam_fwd_world
                ) * left_gain
                left_delta_w = left_delta_w * min(1.0, _MAX_REACH_M / max(left_delta_w.norm().item(), 1e-6))
                target_pos_b_left_dbg = (
                    left_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, left_delta_w.unsqueeze(0))
                )
                if left_disp_raw.norm().item() > 0.05:
                    print(f"[Quest][axisdbg] L quest_disp(x,y,z)={left_disp_raw.tolist()}  "
                          f"world_delta={left_delta_w.tolist()}  "
                          f"base_delta_from_home={(target_pos_b_left_dbg - left_arm.home_tip_pos_b)[0].tolist()}",
                          flush=True)
                target_pos_b_left = target_pos_b_left_dbg
                # Quest-local -> camera-local (fixed ergonomic mapping, _QUEST_TO_CAM_LOCAL) ->
                # world (via the camera's ACTUAL current tilt, camera_tilt_quat) -- same
                # camera-relative philosophy as the position mapping above, replacing the old
                # fixed-world-frame quest_to_world_quat conjugation (see _QUEST_TO_WORLD's
                # comment for why that was wrong). wrist_orient_offset remains available as a
                # small residual fine-tune knob (identity until/unless live testing shows one is
                # still needed).
                dq_left = quat_mul(left_quat.unsqueeze(0), quat_inv(left_arm.quest_home_quat.unsqueeze(0)))
                dq_left_camlocal = quat_mul(quat_mul(quest_to_cam_local_quat, dq_left), quat_inv(quest_to_cam_local_quat))
                dq_left_world = quat_mul(quat_mul(camera_tilt_quat, dq_left_camlocal), quat_inv(camera_tilt_quat))
                dq_left_world = quat_mul(quat_mul(left_arm.wrist_orient_offset, dq_left_world),
                                         quat_inv(left_arm.wrist_orient_offset))
                dq_left_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_left_world), root_quat_w)
                target_quat_b_left = quat_mul(dq_left_base, left_arm.home_tip_quat_b)

            if right_arm.quest_home_xyz is not None and right_tracked:
                right_disp_raw = right_xyz_q - right_arm.quest_home_xyz
                right_gain = _ramped_gain(right_disp_raw.norm(), gain)
                right_right_amt = right_disp_raw[0] * axis_sign_right[0]
                right_up_amt = right_disp_raw[1] * axis_sign_right[1]
                right_fwd_amt = -right_disp_raw[2] * axis_sign_right[2]
                right_delta_w = (
                    right_right_amt * cam_right_world
                    + right_up_amt * cam_up_world
                    + right_fwd_amt * cam_fwd_world
                ) * right_gain
                right_delta_w = right_delta_w * min(1.0, _MAX_REACH_M / max(right_delta_w.norm().item(), 1e-6))
                target_pos_b_right_dbg = (
                    right_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, right_delta_w.unsqueeze(0))
                )
                if right_disp_raw.norm().item() > 0.05:
                    print(f"[Quest][axisdbg] R quest_disp(x,y,z)={right_disp_raw.tolist()}  "
                          f"world_delta={right_delta_w.tolist()}  "
                          f"base_delta_from_home={(target_pos_b_right_dbg - right_arm.home_tip_pos_b)[0].tolist()}",
                          flush=True)
                target_pos_b_right = target_pos_b_right_dbg
                # See the left-arm block above for why this now goes through
                # quest_to_cam_local_quat + camera_tilt_quat instead of quest_to_world_quat.
                dq_right = quat_mul(right_quat.unsqueeze(0), quat_inv(right_arm.quest_home_quat.unsqueeze(0)))
                dq_right_camlocal = quat_mul(quat_mul(quest_to_cam_local_quat, dq_right), quat_inv(quest_to_cam_local_quat))
                dq_right_world = quat_mul(quat_mul(camera_tilt_quat, dq_right_camlocal), quat_inv(camera_tilt_quat))
                dq_right_world = quat_mul(quat_mul(right_arm.wrist_orient_offset, dq_right_world),
                                          quat_inv(right_arm.wrist_orient_offset))
                dq_right_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_right_world), root_quat_w)
                target_quat_b_right = quat_mul(dq_right_base, right_arm.home_tip_quat_b)

            # Stereo viewpoint is fully fixed -- no head tracking.
            # head_viewpoint_pos_b/quat_b stay at their home values for the
            # whole run; head_home_xyz/quat/head_tracked above are unused for
            # positioning (only feed _is_tracked). Head-tracked position and
            # rotate-only variants are both in git history if wanted again.

            diag_frame += 1
            if diag_frame % 100 == 0:
                if left_arm.quest_home_xyz is not None and left_tracked:
                    err_l = (target_pos_b_left - tip_pos_b_l).norm().item()
                    print(f"[Quest][diag] L(tip) gain={left_gain.item():.2f} target_err={err_l:.3f}m", flush=True)
                if right_arm.quest_home_xyz is not None and right_tracked:
                    err_r = (target_pos_b_right - tip_pos_b_r).norm().item()
                    print(f"[Quest][diag] R(tip) gain={right_gain.item():.2f} target_err={err_r:.3f}m", flush=True)
                # Sanity-check for _adaptive_dls_lambda (new, unverified live) -- confirms what
                # manipulability values actually occur during normal use, since _DLS_MANIPULABILITY_EPSILON
                # was picked without live data on this arm's own conditioning. If lambda_used never
                # moves off lambda_min during normal reach, epsilon is set too low (damping never
                # kicks in); if it's frequently near lambda_max even in comfortable poses, epsilon
                # is set too high (over-damping, sluggish tracking).
                if left_arm.last_manipulability is not None:
                    print(f"[Quest][ikdiag] L manipulability={left_arm.last_manipulability:.4f} "
                          f"lambda_used={left_arm.controller.cfg.ik_params['lambda_val']:.3f} "
                          f"(min={left_arm.lambda_min} max={left_arm.lambda_max})", flush=True)
                if right_arm.last_manipulability is not None:
                    print(f"[Quest][ikdiag] R manipulability={right_arm.last_manipulability:.4f} "
                          f"lambda_used={right_arm.controller.cfg.ik_params['lambda_val']:.3f} "
                          f"(min={right_arm.lambda_min} max={right_arm.lambda_max})", flush=True)

            # USD VisualizationMarkers are only ever .visualize()'d (made non-degenerate) when NOT
            # recording, per explicit live feedback -- TWO separate attempts at excluding these
            # markers from wrist_cam's render specifically (Hydra-viewport cameraVisibility on the
            # camera prim, then the USD-spec-correct RenderProduct-level cameraVisibility) both
            # silently failed to actually hide them (confirmed visually both times) -- this RTX
            # renderer version apparently doesn't honor that collection at all, on any prim.
            # Rather than keep guessing at render-pipeline tricks, this guarantees zero risk of
            # contaminating recorded training data: no markers exist in the scene at all while
            # recording, so there's nothing to leak into ego_cam/wrist_cam regardless of any
            # camera-exclusion mechanism working or not. (World-space target positions for the
            # browser-side marker's screen-space projection are computed unconditionally, every
            # frame, further below -- not tied to this recording gate.)
            if not args_cli.record:
                _rp = robot.data.root_state_w[:, :3]
                if left_arm.quest_home_xyz is not None:
                    left_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_left))
                if right_arm.quest_home_xyz is not None:
                    right_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_right))

            if left_tracked:
                ld = _pinch_dist(left_joints)
                if ld < _PINCH_CLOSE_M:
                    left_closed = True
                elif ld > _PINCH_OPEN_M:
                    left_closed = False

            if right_tracked:
                rd = _pinch_dist(right_joints)
                if rd < _PINCH_CLOSE_M:
                    right_closed = True
                elif rd > _PINCH_OPEN_M:
                    right_closed = False

        # ── Differential IK (DLS) solve, both arms, every frame ─────────────────
        tip_pos_b_l_now, tip_quat_b_l_now = left_arm.solve_and_apply(robot, device, target_pos_b_left, target_quat_b_left, sim_dt)
        tip_pos_b_r_now, tip_quat_b_r_now = right_arm.solve_and_apply(robot, device, target_pos_b_right, target_quat_b_right, sim_dt)

        # ── Real-hardware bridge (left arm only) ─────────────────────────────────
        # Same held-off-then-throttled pattern as task_space_real.py: elapsed
        # time only starts accumulating toward the publish period AFTER the
        # startup delay has fully passed, so it can't build up a backlog
        # during the hold and burst-publish once the delay ends.
        if real_left_arm_pub is not None:
            real_left_arm_elapsed_s += sim_dt
            if real_left_arm_elapsed_s >= _REAL_ARM_PUBLISH_START_DELAY_S:
                real_left_arm_since_publish_s += sim_dt
            if (real_left_arm_elapsed_s >= _REAL_ARM_PUBLISH_START_DELAY_S
                    and real_left_arm_since_publish_s >= _REAL_ARM_PUBLISH_PERIOD_S):
                real_left_arm_since_publish_s -= _REAL_ARM_PUBLISH_PERIOD_S
                if not real_left_arm_started:
                    real_left_arm_started = True
                    print("[Quest][REAL HARDWARE] Publishing left arm to /behaviour/arm_pose now.", flush=True)
                _publish_real_left_arm_pose(real_left_arm_pub, receiver, left_arm.last_joint_pos_des)

        if pov_capture_frame % 100 == 0:
            err_l = (target_pos_b_left - tip_pos_b_l_now).norm().item()
            err_r = (target_pos_b_right - tip_pos_b_r_now).norm().item()
            print(f"[Quest][diag] home-offset convergence: L target_err={err_l:.3f}m tip={tip_pos_b_l_now[0].tolist()} "
                  f"R target_err={err_r:.3f}m tip={tip_pos_b_r_now[0].tolist()}", flush=True)
            # Wrist-orientation calibration diagnostic: commanded delta (target_quat_b vs
            # home_tip_quat_b) vs the ACTUALLY achieved delta (tip_quat_b_now vs
            # home_tip_quat_b), both as axis-angle. If wrist_orient_offset is correctly tuned,
            # these two axes should match closely (angle can lag during fast motion, axis should
            # not). Used for offline calibration via synthetic /quest_teleop messages -- not
            # needed for normal teleop use.
            if left_arm.home_tip_quat_b is not None:
                cmd_dq_l = quat_mul(target_quat_b_left, quat_inv(left_arm.home_tip_quat_b))
                ach_dq_l = quat_mul(tip_quat_b_l_now, quat_inv(left_arm.home_tip_quat_b))
                print(f"[Quest][wristdiag] L commanded_delta(wxyz)={cmd_dq_l[0].tolist()} "
                      f"achieved_delta(wxyz)={ach_dq_l[0].tolist()}", flush=True)
            if right_arm.home_tip_quat_b is not None:
                cmd_dq_r = quat_mul(target_quat_b_right, quat_inv(right_arm.home_tip_quat_b))
                ach_dq_r = quat_mul(tip_quat_b_r_now, quat_inv(right_arm.home_tip_quat_b))
                print(f"[Quest][wristdiag] R commanded_delta(wxyz)={cmd_dq_r[0].tolist()} "
                      f"achieved_delta(wxyz)={ach_dq_r[0].tolist()}", flush=True)

        # ── Gripper targets ────────────────────────────────────────────────────
        # Smoothed via _smooth_damp (same filter as the arm joints) instead of a raw snap to
        # the binary open/closed target -- see left_gripper_smoothed's comment above for why.
        # smooth_time=0.05s (faster than the arms' 0.08s -- grasp timing still needs to feel
        # snappy) and max_speed generously large relative to the ~0.05-unit gripper range, so
        # smooth_time is what shapes the motion, not the speed cap.
        left_gripper_target = left_g_closed if left_closed else left_g_open
        right_gripper_target = right_g_closed if right_closed else right_g_open
        left_gripper_smoothed, left_gripper_vel = _smooth_damp(
            left_gripper_smoothed, left_gripper_vel, left_gripper_target, 0.05, 1.0, sim_dt,
        )
        right_gripper_smoothed, right_gripper_vel = _smooth_damp(
            right_gripper_smoothed, right_gripper_vel, right_gripper_target, 0.05, 1.0, sim_dt,
        )
        robot.set_joint_position_target(left_gripper_smoothed, joint_ids=left_gripper_ids)
        robot.set_joint_position_target(right_gripper_smoothed, joint_ids=right_gripper_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(left_gripper_ids), device=device), joint_ids=left_gripper_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(right_gripper_ids), device=device), joint_ids=right_gripper_ids)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # ── Recording (L-suffixed/link6l arm only -- the one ego_cam/
        # wrist_cam are mounted for) ─────────────────────────────────────────
        if recorder is not None:
            if recorder.is_complete:
                print("[RECORD] Session complete.", flush=True)
                break
            gripper_target = left_g_closed if left_closed else left_g_open
            action = torch.cat([left_arm.last_joint_pos_des, gripper_target[:, :1]], dim=1)[0]
            measured_gripper = robot.data.joint_pos[:, left_gripper_ids[:1]]
            state = torch.cat([robot.data.joint_pos[:, left_arm.arm_ids], measured_gripper], dim=1)[0]
            # _capture_record_images does two 640x480 GPU->CPU copies -- lazily evaluated (see
            # SimLeRobotRecorder.tick's images_fn param) so it only actually runs on frames the
            # recorder is really about to push, instead of unconditionally every 100Hz physics
            # step. Doing it every step tanked the sim's real-time factor (all the motion-clamp
            # math runs on fixed SIMULATED time, so the arm still moved correctly per sim-second,
            # but far less sim-time elapsed per second of the operator's real hand movement --
            # this is what live feedback described as the arm "barely moving" once --record was on).
            recorder.tick(action, state, lambda: _capture_record_images(scene))

        # Stereo eye mounts follow the current head viewpoint (a no-op right
        # now since it's fixed -- see above -- but cheap, and ready if head
        # tracking comes back). IPD offset applied along the viewpoint's
        # current "right" axis, rotated into base frame.
        right_offset_b = quat_apply(head_viewpoint_quat_b, eye_local_right.unsqueeze(0)) * (_EYE_IPD_M / 2)
        left_eye_pos_b = (head_viewpoint_pos_b - right_offset_b)[0].tolist()
        right_eye_pos_b = (head_viewpoint_pos_b + right_offset_b)[0].tolist()
        head_orient_b = head_viewpoint_quat_b[0].tolist()
        _set_mount_pose(left_eye_mount, tuple(left_eye_pos_b), tuple(head_orient_b))
        _set_mount_pose(right_eye_mount, tuple(right_eye_pos_b), tuple(head_orient_b))
        # ego_cam (data-collection camera) pinned to the left eye's pose so
        # recorded frames match what the operator saw, but nudged forward
        # along the gaze direction -- exact co-location put its lens inside
        # the RSD455 housing mesh (see _EGO_CAM_FORWARD_OFFSET_M above).
        forward_offset_b = quat_apply(head_viewpoint_quat_b, eye_local_forward.unsqueeze(0)) * _EGO_CAM_FORWARD_OFFSET_M
        ego_cam_pos_b = (head_viewpoint_pos_b - right_offset_b + forward_offset_b)[0].tolist()
        ego_cam_pos_b[2] += _EGO_CAM_UP_OFFSET_M  # straight base-frame +Z, per live feedback
        ego_cam_orient_b = quat_mul(
            ego_cam_extra_tilt_quat, quat_mul(head_viewpoint_quat_b, ego_cam_convention_fix_quat)
        )[0].tolist()
        if not _EGO_CAM_FREE_MOVE_DEBUG:
            _set_mount_pose(_ego_cam_prim_path, tuple(ego_cam_pos_b), tuple(ego_cam_orient_b))

        pov_capture_frame += 1
        if (left_eye_viewport_api is not None and right_eye_viewport_api is not None
                and pov_capture_frame % _POV_CAPTURE_EVERY_N_STEPS == 0):
            from omni.kit.viewport.utility import capture_viewport_to_file

            _capture_t0 = time.monotonic()
            capture_viewport_to_file(left_eye_viewport_api, str(_POV_FRAME_PATH_LEFT))
            capture_viewport_to_file(right_eye_viewport_api, str(_POV_FRAME_PATH_RIGHT))
            _write_wrist_cam_hud_frame(scene)

            # Marker screen position: project each arm's CURRENT IK target (world-space, updated
            # every frame above regardless of --record) through both real eye cameras, so the
            # browser can draw the marker at the pixel it actually occupies in pov_left.png/
            # pov_right.png -- not at the operator's own raw hand-tracking position, which is a
            # different coordinate space and was why the client-side marker appeared at the
            # physical wrist instead of "in front of, teleoped by" the operator.
            _rp_now = robot.data.root_state_w[:, :3]
            _target_world_left = (_rp_now + quat_apply(root_quat_w, target_pos_b_left))[0].tolist()
            _target_world_right = (_rp_now + quat_apply(root_quat_w, target_pos_b_right))[0].tolist()
            _lu_l, _lv_l, _lvis_l = _project_world_point_to_uv(_left_eye_cam_prim, _target_world_left)
            _ru_l, _rv_l, _rvis_l = _project_world_point_to_uv(_right_eye_cam_prim, _target_world_left)
            _lu_r, _lv_r, _lvis_r = _project_world_point_to_uv(_left_eye_cam_prim, _target_world_right)
            _ru_r, _rv_r, _rvis_r = _project_world_point_to_uv(_right_eye_cam_prim, _target_world_right)
            _marker_uv_tmp = _MARKER_UV_PATH.with_suffix(".json.tmp")
            _marker_uv_tmp.write_text(json.dumps({
                "left_eye": {
                    "left_arm": {"u": _lu_l, "v": _lv_l, "visible": _lvis_l and left_arm.quest_home_xyz is not None},
                    "right_arm": {"u": _lu_r, "v": _lv_r, "visible": _lvis_r and right_arm.quest_home_xyz is not None},
                },
                "right_eye": {
                    "left_arm": {"u": _ru_l, "v": _rv_l, "visible": _rvis_l and left_arm.quest_home_xyz is not None},
                    "right_arm": {"u": _ru_r, "v": _rv_r, "visible": _rvis_r and right_arm.quest_home_xyz is not None},
                },
            }))
            _marker_uv_tmp.replace(_MARKER_UV_PATH)
            _capture_dt_ms = (time.monotonic() - _capture_t0) * 1000.0
            # Real measured capture rate (wall-clock, not the theoretical render_interval
            # target) -- this is what the headset actually receives per second -- PLUS how much
            # of that cycle time is specifically the capture_viewport_to_file call (GPU readback
            # + PNG encode + disk write) vs. the rest of the loop (physics step + render +
            # everything else), so it's possible to tell whether capture itself is the
            # bottleneck or just general render/physics cost. Printed every ~2s.
            _fps_window_frames += 1
            _fps_window_capture_ms += _capture_dt_ms
            _fps_elapsed = time.monotonic() - _fps_window_start_t
            if _fps_elapsed >= 2.0:
                _avg_capture_ms = _fps_window_capture_ms / max(_fps_window_frames, 1)
                _avg_cycle_ms = (_fps_elapsed / max(_fps_window_frames, 1)) * 1000.0
                print(f"[Quest][fps] measured POV capture rate: {_fps_window_frames / _fps_elapsed:.1f} fps "
                      f"(target ~33fps at render_interval=3) | avg capture_viewport_to_file time: "
                      f"{_avg_capture_ms:.1f}ms | avg full cycle (render_interval steps) time: {_avg_cycle_ms:.1f}ms",
                      flush=True)
                _fps_window_start_t = time.monotonic()
                _fps_window_frames = 0
                _fps_window_capture_ms = 0.0

    if recorder is not None:
        recorder.finalize()
        print(f"[RECORD] Saved under {recorder.dataset_root}", flush=True)


def main() -> None:
    # render_interval=3: physics still steps at the full 100Hz (dt=0.01) for
    # accurate IK/control response, render (4 camera-ish sources at 640x480
    # viewport res, now 480x360 -- see _open_pov_viewport) happens every 3rd
    # step, ~33Hz. Raised back from render_interval=5 (~20Hz) once the real
    # headset-fps bottleneck (index.html's hardcoded 200ms/5fps poll
    # interval) was found and fixed -- laptop-side perf was already fine.
    # _POV_CAPTURE_EVERY_N_STEPS above is aligned to this so capture always
    # lands on a freshly-rendered frame.
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, render_interval=3, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    # Trim RTX real-time render cost -- reflections/AO aren't needed for a
    # functional data-collection view and are real per-frame cost on this
    # GPU. Best-effort: wrong/missing setting paths across Kit versions
    # shouldn't break teleop, just leave the feature at its default.
    _settings = carb.settings.get_settings()
    for _rtx_setting, _value in (
        ("/rtx/reflections/enabled", False),
        ("/rtx/ambientOcclusion/enabled", False),
        ("/rtx/indirectDiffuse/enabled", False),
    ):
        try:
            _settings.set(_rtx_setting, _value)
        except Exception as exc:  # noqa: BLE001 -- best-effort, see docstring above
            print(f"[Quest] Could not set {_rtx_setting}: {exc}", flush=True)

    scene = InteractiveScene(ArmV2SceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    print("[Quest] Simulation ready (lightbox enclosure walls added).")
    run_simulator(sim, scene)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
