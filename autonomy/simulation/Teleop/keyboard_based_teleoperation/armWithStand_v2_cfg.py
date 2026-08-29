"""
armWithStand (wato_arm_v2) articulation config for teleoperation.

Robot model: Humanoid_Wato/pioneer_bimanual_arm (pioneer_bimanual_arm.usd),
formerly wato_arm_v2/armWithStand.usd.

CORRECTION (see git history for the original claim this replaced): despite
matching link/joint NAMES with wato_bimanual_arm, this is a DIFFERENT CAD
export with DIFFERENT per-joint rotation axes and origins -- confirmed by
diffing both URDFs' <joint><axis>/<origin> tags directly. e.g. joint3's axis
is (0,-1,0) in bimanual vs (0,0,-1) here; joint4 is (-1,0,0) vs (0,-1,0);
joint5 is (0,-1,0) vs (0,0,1); joint6 is (1,0,0) vs (0,-1,0). Every revolute
joint from 3 onward rotates about a different local axis than its bimanual
namesake. Do NOT assume any constant below transfers from bimanual_arm_cfg.py
just because the joint/link names match.

What this means for the values below:
  - _DEFAULT_JOINT_POS: reset to the URDF's own zero pose (0.0 rad on every
    revolute joint) instead of copying bimanual's tuned degree values --
    applying angles tuned for bimanual's axes to armWithStand's differently-
    oriented joints produced a visibly twisted/unnatural rest pose (this was
    the root cause of "joints moved in different directions than my real
    hand" reported during Quest teleop testing: the DLS IK solver itself
    uses the actual PhysX Jacobian, so it's axis-agnostic and was solving
    correctly, but starting from a contorted configuration makes the
    resulting motion look wrong). Zero is not necessarily the most ergonomic
    rest pose, but it's a defined, correct-by-construction one -- retune
    live via Physics Inspector once you have GUI access, same as bimanual's
    values originally were.
  - LEFT_/RIGHT_FINGER_DISTAL_TIP_LOCAL: measured from this asset's own
    link7/8/7l/8l mesh geometry in object space, along its ACTUAL prismatic
    travel axis for joint7/8 (Y here, vs X in bimanual) -- "outward-Y
    extreme, X/Z bbox-center". Bimanual's X-based numbers are wrong for this
    asset's axis.
  - actuator gains / *_GRIPPER_OPEN-CLOSED: UNCHANGED from bimanual_arm_cfg.py,
    i.e. still not independently verified. The gripper's ±0.05m stroke and the
    actuator stiffness/damping/effort numbers are motor/mechanism-derived (same
    GL40 hardware presumed), not orientation-derived, so they're a reasonable
    carry-over but still worth confirming against the real mechanism.
  - JOINT_POS_LIMITS: from the URDF (authoritative); see the section below.
  - _WRIST_ORIENT_OFFSET_LEFT/RIGHT in run_quest_armv2_teleop.py
    (not in this file) were reset to identity for the same reason as
    _DEFAULT_JOINT_POS -- bimanual's offsets were empirically tuned against
    its own joint6 axis and don't transfer. Needs live retuning once wrist
    rotation is tested in-headset (see that script's module docstring).

Joint limits/poses: Isaac Sim Physics Inspector (/World/bimanual_arm/root_joint)
Motor specs: https://watonomous.github.io/humanoid-docs/mechanical/index.html

  Shoulder joints 1-2  AK10-9 V3.0  18 Nm rated / 53 Nm peak
  Elbow joints 3-5     AK80-9 V3.0   9 Nm rated / 22 Nm peak
  Wrist joint 6        GL40 KV70     0.25 Nm rated / 0.73 Nm peak
  Gripper              GL40 KV70     0.25 Nm rated / 0.73 Nm peak
                       (rotary motor + linkage → prismatic finger travel in URDF)

NOTE on arm naming: the "L"-suffixed URDF chain (joint1L..joint6l, link6l,
link7l/link8l) is the robot's LEFT arm, exactly as the CAD naming says, and it
is also the REAL, CAN-actuated one. The unsuffixed chain (joint1..joint6,
link6, link7/link8) is the RIGHT arm.

LEFT_* = L-suffixed chain, RIGHT_* = unsuffixed. An earlier revision had these
reversed, so importers aliased them back; don't reintroduce that swap.

Actuator groups below were tuned for holding a static pose, not for tracking a
moving IK target -- recheck stiffness/damping if an arm feels sluggish.

Gripper actuation note
----------------------
On hardware, ONE GL40 rotary motor closes/opens the gripper through a
mechanical linkage (screw / hinge stack). The URDF instead exposes TWO
independent prismatic joints (joint7 + joint8, joint7l + joint8l).

In sim we:
  1. Drive both prismatic joints with synchronized position targets (open/closed pair).
  2. Use HIGH stiffness/damping so fingers stay locked during arm motion.
  3. Do NOT copy 0.25 Nm directly — effort_limit_sim on prismatic DOFs is a
     linear force cap (Newtons), not motor torque. Tune by grasp/hold behaviour.
"""
import math
import os
import sys

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sensors import CameraCfg

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
# pioneer_bimanual_arm is the current export of this arm, superseding wato_arm_v2/armWithStand.
# Same joint/link names and topology, but the mesh and joint origins moved.
_ARM_V2_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "Humanoid_Wato", "pioneer_bimanual_arm"))
_ARM_USD_PATH = os.path.join(_ARM_V2_ROOT, "usd", "pioneer_bimanual_arm.usd")

if _ARM_V2_ROOT not in sys.path:
    sys.path.insert(0, _ARM_V2_ROOT)
from urdf_joint_limits import JOINT_POS_LIMITS  # noqa: E402  (needs the path insert above)


def _deg(degrees: float) -> float:
    return degrees * math.pi / 180.0


# --- Joint limits -----------------------------------------------------------
# JOINT_POS_LIMITS is imported above from pioneer_bimanual_arm/urdf_joint_limits.py, which
# parses urdf/pioneer_bimanual_arm.urdf. The URDF is the ground truth: edit it and every
# consumer of this module picks the change up, no re-conversion needed. It is re-exported
# here so existing `from armWithStand_v2_cfg import JOINT_POS_LIMITS` call sites keep working.
#
# This replaced a hardcoded +/-2pi placeholder (+/-360 deg per revolute joint, i.e. no limit
# at all) that was carried over from bimanual_arm_cfg.py and never verified. Because
# apply_joint_limits() and patch_joint_pos_limits_on_prim() below write this table over the
# spawned USD prims, that placeholder was actively DESTROYING the real limits the asset
# shipped with -- every revolute joint ran unbounded at runtime.

# --- Default (spawn) pose: URDF zero, EXCEPT the elbows.
#
# This is Isaac Lab's InitialStateCfg, a runtime spawn state -- it does NOT change the URDF's
# zero pose, and q=0 still means exactly what the asset says it means.
#
# At URDF zero both arms hang straight down with the elbow fully extended, which is the elbow
# EXTENSION SINGULARITY: measured there, cond(J) = 2560 and manipulability sqrt(det(J J^T)) =
# 2.0e-06, and the least-controllable translation direction is almost exactly +Z -- the very
# direction run_quest_armv2_teleop.py's _HOME_TIP_Z_OFFSET then commands the fingertip along.
# Moving the tip 1 m along +Z from there costs 282 rad of joint motion, so the first IK step is
# enormous and ill-conditioned: it dumps motion into the shoulder (the "shoulder hike") and
# picks an elbow bend direction essentially at random, because at joint4/joint4l = 0 both signs
# give identical first-order +Z motion, and a bad first step persists.
#
# Flexing the elbows fixes both problems at once: manipulability rises to 1.9e-02 (10,000x) and
# the arm starts unambiguously in the flexion branch. SIGNS ARE OPPOSITE because the two elbows
# have opposite axis vectors (joint4 is (0,-1,0), joint4l is (0,1,0)); both put the forearm
# FORWARD toward +X, the direction the robot faces. Keep them opposite and equal in magnitude --
# that, not equal signed values, is what makes the spawn pose mirror-symmetric.
#
# 75 deg, not 90: lowered from 90 to drop the fingertip ~10cm, solved by sweeping theta with
# joint4=+theta / joint4l=-theta and measuring the real tip with compute_gripper_tip_pose_b.
#
# MEASUREMENT NOTE, worth knowing before re-deriving any of this: InitialStateCfg.joint_pos only
# populates robot.data.default_joint_pos. PhysX still spawns the articulation at the USD ZERO
# pose, and the implicit actuators barely move it (measured: joint4 was -0.36 deg after 500
# steps). A script that just calls sim.reset() and steps therefore measures the arms hanging
# straight down, NOT this pose. run_simulator writes the state explicitly (see
# run_quest_armv2_teleop.py:1403); any offline check must pin the pose the same way each step.
#
# Measured with the pose pinned, theta = 75 vs 90:
#     tip (base frame)    (+0.3931, +/-0.2886, -0.3424)   vs (+0.4066, +/-0.2886, -0.2410)
#     drop                -0.1014 m at the tip
#     mirror error        0.000000 m  (L and R tips exactly opposite in Y)
#     tip above table     +0.1526 m   (table top at base-frame z -0.4950)
#     conditioning        manipulability 2.002e-02, cond(J) 14.7
#                         (90 deg: 2.134e-02 / 12.8;  theta=0 singularity: 1.21e-05 / 1220)
# So the drop costs ~6% of manipulability and stays three orders of magnitude clear of the
# singularity. Do not push much below ~60 deg -- the tip approaches the table and the sweep goes
# non-monotonic as the hand starts contacting it.
# Both inside the URDF limits with 57.5 deg margin each side: joint4 [-100, +132.5],
# joint4l its mirror [-132.5, +100]. (They used to be identical rather than mirrored ->
# opposite flexion/hyperextension budgets per arm; fixed in the URDF, see its header.)
_DEFAULT_JOINT_POS = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": _deg(75.0),
    "joint5": 0.0,
    "joint6": 0.0,
    "joint7": 0.0,
    "joint8": 0.0,
    # Left arm (the L-suffixed chain)
    "joint1L": 0.0,
    "joint2l": 0.0,
    "joint3l": 0.0,
    "joint4l": _deg(-75.0),
    "joint5l": 0.0,
    "joint6l": 0.0,
    "joint7l": 0.0,
    "joint8l": 0.0,
}

LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT_GRIPPER_JOINTS = ["joint7l", "joint8l"]
RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
# Jacobian anchor is the wrist link; IK pose target is the fingertip center (see below).
LEFT_EE_BODY = "link6l"
RIGHT_EE_BODY = "link6"
LEFT_FINGER_TIP_BODIES = ("link7l", "link8l")
RIGHT_FINGER_TIP_BODIES = ("link7", "link8")
# Distal mesh point of each finger, in that link's OWN object-space frame: the outward-Y extreme
# (larger-magnitude Y bbox bound) with X/Z at the bbox centre, measured via
# UsdGeom.BBoxCache.ComputeRelativeBound against this asset's mesh. Same "outward extreme along
# the travel axis" methodology bimanual_arm_cfg.py used, but along Y -- bimanual's joint7/8 axis
# is X, so its numbers are wrong here (see module docstring).
#
# LINK-LOCAL, so they do not survive a re-export that moves the link origins; re-measure the same
# way if the arm asset changes. Sanity check: composed with the joint7/8 origins these give a
# 0.1625m open fingertip gap on both grippers, matching the empirically measured 0.161m below.
LEFT_FINGER_DISTAL_TIP_LOCAL = {
    "link7l": (0.0, 0.151416, -0.045369),
    "link8l": (0.0, -0.114416, -0.039825),
}
RIGHT_FINGER_DISTAL_TIP_LOCAL = {
    "link7": (0.0, -0.151416, -0.045369),
    "link8": (0.0, 0.114416, -0.039825),
}

# Gripper finger targets (joint7/joint7l: [-0.05, 0], joint8/joint8l: [0, 0.05]).
# VERIFIED empirically (not copied from bimanual) via headless sim: measured
# the actual fingertip-to-fingertip gap at both endpoints of the configured
# range. (joint7l=-0.05, joint8l=+0.05) -> gap=0.074m (closed).
# (joint7l=0, joint8l=0) -> gap=0.159m (open); the right gripper measures
# 0.161m open on the same test. This is the OPPOSITE labeling
# bimanual_arm_cfg.py uses for its own joint7l/8l -- expected, since this
# asset's joint7/8 prismatic axis is Y, bimanual's is X (see module
# docstring). Synchronized pair mimics single GL40 motor driving both
# fingers via linkage.
#
# Both arms live here -- properties of the asset, not of whichever script drives it.
LEFT_GRIPPER_OPEN = {"joint7l": 0.0, "joint8l": 0.0}
LEFT_GRIPPER_CLOSED = {"joint7l": -0.05, "joint8l": 0.05}
RIGHT_GRIPPER_OPEN = {"joint7": 0.0, "joint8": 0.0}
RIGHT_GRIPPER_CLOSED = {"joint7": -0.05, "joint8": 0.05}

# Prismatic gripper PD — tuned for hold during arm motion (not from motor datasheet).
# If fingers bounce when the shoulder moves, raise stiffness; if jittery, raise damping.
_GRIPPER_STIFFNESS = 400.0
_GRIPPER_DAMPING = 40.0
_GRIPPER_EFFORT_LIMIT = 30.0  # N (sim linear-force cap; tune empirically)
_GRIPPER_VELOCITY_LIMIT = 0.2  # m/s


# ── data-collection cameras ───────────────────────────────────────────────────
# ego_cam (on base_link) and wrist_cam (on link6l) are the cameras recording reads. Defined here
# rather than in run_quest_armv2_teleop.py because their prim paths are properties of this arm.
#
# They are SPAWNED rather than read from the robot asset. armWithStand.usd used to bake camera
# prims into its sensor layer and the scripts pointed at them with spawn=None; the
# pioneer_bimanual_arm export has no cameras at all, so every such call site broke at once.
# Defining them in code means a future re-export cannot silently drop them.

# focal_length 18 is ~60deg horizontal; lower it to widen. run_quest_armv2_teleop.py overwrites
# ego_cam's at runtime to match the headset's widened RSD455 FOV.
DATA_CAM_LENS = sim_utils.PinholeCameraCfg(
    focal_length=7.336, horizontal_aperture=20.955, vertical_aperture=15.2908,
    clipping_range=(0.01, 100.0),
)

# ego_cam pose, relative to base_link. Only an initial value in the Quest teleop script, which
# re-aims ego_cam at the operator's head viewpoint at startup when --record is passed.
EGO_CAM_POS = (0.047450090928410314, -0.008096717438775313, 0.21180604954921534)
EGO_CAM_ROT = (0.8660254037844387, 0.49999999999999983, 0.0, 0.0)

# wrist_cam aiming. The two angles are independent: roll spins the image, pitch aims the camera.
WRIST_CAM_ROLL_DEG = 270.0   # rotates the image counter-clockwise; 90 / 180 / 270
WRIST_CAM_PITCH_DEG = 30.0   # angles the camera down toward the gripper

# wrist_cam mount point, relative to link6l's origin. Relative to the gripper:
#     +X moves the camera UP        +Y moves it LEFT        -Z moves it FORWARD
# (those follow from the roll/pitch above -- recompute them if you change either angle.)
# The camera sits inside the wrist housing, so large moves can bury it in the mesh. The Quest
# teleop script has a _SHOW_WRIST_CAM_MARKER flag that draws a sphere here to make it visible.
WRIST_CAM_POS = (0.06168, 0.053, -0.06995)


def _quat_mul_wxyz(a: tuple, b: tuple) -> tuple:
    """Hamilton product of two (w, x, y, z) quaternions as plain tuples.

    isaaclab.utils.math.quat_mul wants torch tensors; these are module constants."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


# Pitch must be composed OUTSIDE the roll. The other order rolls the pitch axis too, so changing
# the roll would silently change which way the camera tilts.
def _wrist_cam_rot(roll_deg: float) -> tuple:
    """Wrist-camera orientation for a given roll, at the shared WRIST_CAM_PITCH_DEG."""
    return _quat_mul_wxyz(
        (math.cos(math.radians(WRIST_CAM_PITCH_DEG) / 2.0), 0.0,
         math.sin(math.radians(WRIST_CAM_PITCH_DEG) / 2.0), 0.0),
        (math.cos(math.radians(roll_deg) / 2.0), 0.0, 0.0,
         math.sin(math.radians(roll_deg) / 2.0)),
    )


WRIST_CAM_ROT = _wrist_cam_rot(WRIST_CAM_ROLL_DEG)


def make_ego_cam_cfg() -> CameraCfg:
    """Fresh CameraCfg for ego_cam. Requires launching with --enable_cameras.

    A new instance per call, not a shared constant: InteractiveScene rewrites prim_path in place
    when it resolves {ENV_REGEX_NS}."""
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/ego_cam",
        spawn=DATA_CAM_LENS,
        offset=CameraCfg.OffsetCfg(pos=EGO_CAM_POS, rot=EGO_CAM_ROT, convention="opengl"),
        height=480, width=640,
        # Refresh whenever the app renders. An additional per-camera update_period on top of
        # that froze this feed on a stale scene state.
        update_period=0.0,
        data_types=["rgb"],
    )


# The two arms are mirror images through the robot's XZ plane, and in each wrist link's OWN
# object-space frame that mirror is "negate Y" -- see LEFT/RIGHT_FINGER_DISTAL_TIP_LOCAL above,
# whose measured mesh extremes differ only in the sign of Y. Confirmed in-sim: link6 and link6l's
# world orientations satisfy (w, x, y, z) -> (w, -x, y, -z) to 5 decimals.
#
#   position   (x, y, z) -> (x, -y, z)
#   rotation   roll -> 180 - roll.
#
# That 180 is NOT cosmetic and is easy to miss. Reflecting the frame maps the camera's view axis
# correctly (M sends local -Z to -Z), but it also maps the camera's local +Y -- the direction that
# becomes UP in the image -- to -Y. So the pure mirror M*R*M aims at the right gripper but renders
# it upside down; this was visibly confirmed before the term was added. Composing an extra 180
# roll about the camera's own view axis puts image-up back where it belongs, and since both rolls
# are about local Z they add: mirrored_roll = -ROLL + 180 = 180 - ROLL.
# Pitch is untouched because it is already about Y, the mirror plane's normal.
#
# ASSET ASYMMETRY, compensated: link6's origin sits 9.0mm further out in Y than the exact mirror
# of link6l's (measured base-frame y +0.23556 vs -0.24455, while x and z match to 6e-5). The
# fingertips themselves ARE perfectly mirrored, so this is only where the CAD put the link frames.
# A plain mirror of the mount offset therefore inherits that 9mm and pushes the right camera
# outboard, which showed up in the headset as the right feed sitting too far out. Adding the
# measured asymmetry back puts the right camera at the exact mirror of the left one's world
# position, so the two feeds frame their grippers identically.
_LINK6_ORIGIN_Y_ASYMMETRY_M = 0.008997  # link6l.y + link6.y, measured in-sim at the rest pose
def make_wrist_cam_cfg(body: str = "link6l", name: str = "wrist_cam", mirror: bool = False) -> CameraCfg:
    """Fresh CameraCfg for a wrist camera -- see make_ego_cam_cfg for why it is a factory.

    Defaults reproduce the original single link6l camera exactly, so existing call sites are
    unchanged. Pass ``body="link6", mirror=True`` for the opposite arm's wrist."""
    mirrored_pos = (WRIST_CAM_POS[0],
                    -WRIST_CAM_POS[1] + _LINK6_ORIGIN_Y_ASYMMETRY_M,
                    WRIST_CAM_POS[2])
    pos = mirrored_pos if mirror else WRIST_CAM_POS
    rot = _wrist_cam_rot(180.0 - WRIST_CAM_ROLL_DEG) if mirror else WRIST_CAM_ROT
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + f"{body}/{name}",
        spawn=DATA_CAM_LENS,
        offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="opengl"),
        height=480, width=640,
        update_period=0.0,
        data_types=["rgb"],
    )


def _joint_limit_key(name: str) -> str | None:
    """Map a USD/articulation joint name to a JOINT_POS_LIMITS key."""
    if name in JOINT_POS_LIMITS:
        return name
    if len(name) > 1 and name[-1] in ("l", "L"):
        alt = name[:-1] + ("L" if name[-1] == "l" else "l")
        if alt in JOINT_POS_LIMITS:
            return alt
    return None


def apply_joint_limits(robot) -> None:
    """Apply Physics Inspector joint limits to the articulation."""
    limits = robot.data.joint_pos_limits.clone()
    updated = []

    for joint_idx, joint_name in enumerate(robot.data.joint_names):
        limit_key = _joint_limit_key(joint_name)
        if limit_key is None:
            continue
        lo, hi = JOINT_POS_LIMITS[limit_key]
        limits[:, joint_idx, 0] = lo
        limits[:, joint_idx, 1] = hi
        updated.append(joint_name)

    if updated:
        robot.write_joint_position_limit_to_sim(limits, warn_limit_violation=False)
        print(f"[INFO] Applied joint limits for {len(updated)} joints.")


def resolve_body_ids(robot, names: tuple[str, ...] | list[str]) -> list[int]:
    """Map body names to articulation body indices."""
    body_names = list(robot.data.body_names)
    name_to_id = {name: idx for idx, name in enumerate(body_names)}
    missing = [name for name in names if name not in name_to_id]
    if missing:
        raise KeyError(f"Body names {missing} not found in {body_names}")
    return [name_to_id[name] for name in names]


def compute_gripper_tip_pos_w(
    robot, finger_body_ids: list[int],
    tip_bodies: tuple[str, ...] = LEFT_FINGER_TIP_BODIES,
    tip_local: dict = LEFT_FINGER_DISTAL_TIP_LOCAL,
):
    """World-frame midpoint between the distal tips of the two gripper fingers."""
    import torch
    from isaaclab.utils.math import quat_apply

    dtype = robot.data.body_pos_w.dtype
    device = robot.data.body_pos_w.device
    tips = []
    for body_name, body_id in zip(tip_bodies, finger_body_ids):
        local = torch.tensor([tip_local[body_name]], device=device, dtype=dtype)
        body_pos = robot.data.body_pos_w[:, body_id]
        body_quat = robot.data.body_quat_w[:, body_id]
        tips.append(body_pos + quat_apply(body_quat, local))
    return (tips[0] + tips[1]) * 0.5


def compute_gripper_tip_pose_w(
    robot, wrist_body_id: int, finger_body_ids: list[int],
    tip_bodies: tuple[str, ...] = LEFT_FINGER_TIP_BODIES,
    tip_local: dict = LEFT_FINGER_DISTAL_TIP_LOCAL,
):
    """Gripper-tip center pose in world frame (position from fingers, orientation from wrist)."""
    tip_pos_w = compute_gripper_tip_pos_w(robot, finger_body_ids, tip_bodies, tip_local)
    tip_quat_w = robot.data.body_quat_w[:, wrist_body_id]
    return tip_pos_w, tip_quat_w


def compute_gripper_tip_pose_b(
    robot, root_pose_w, wrist_body_id: int, finger_body_ids: list[int],
    tip_bodies: tuple[str, ...] = LEFT_FINGER_TIP_BODIES,
    tip_local: dict = LEFT_FINGER_DISTAL_TIP_LOCAL,
):
    """Gripper-tip center pose in the robot root frame."""
    from isaaclab.utils.math import subtract_frame_transforms

    tip_pos_w, tip_quat_w = compute_gripper_tip_pose_w(
        robot, wrist_body_id, finger_body_ids, tip_bodies, tip_local
    )
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], tip_pos_w, tip_quat_w
    )


def jacobian_world_to_root(robot, jacobian_w):
    """Rotate PhysX world-frame Jacobian into the articulation root frame."""
    import torch
    from isaaclab.utils.math import matrix_from_quat, quat_inv

    base_rot = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jacobian_b = jacobian_w.clone()
    jacobian_b[:, :3, :] = torch.bmm(base_rot, jacobian_b[:, :3, :])
    jacobian_b[:, 3:, :] = torch.bmm(base_rot, jacobian_b[:, 3:, :])
    return jacobian_b


def adjust_jacobian_for_gripper_tip(jacobian_b, wrist_pos_b, tip_pos_b):
    """Map link6l root-frame Jacobian to the fingertip center."""
    import torch
    from isaaclab.utils.math import skew_symmetric_matrix

    offset_b = tip_pos_b - wrist_pos_b
    tip_jacobian = jacobian_b.clone()
    tip_jacobian[:, 0:3, :] += torch.bmm(-skew_symmetric_matrix(offset_b), jacobian_b[:, 3:, :])
    return tip_jacobian


def compute_tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b):
    """World-frame link6l Jacobian -> root frame -> fingertip center."""
    return adjust_jacobian_for_gripper_tip(
        jacobian_world_to_root(robot, jacobian_w), wrist_pos_b, tip_pos_b
    )


def resolve_joint_name(robot, name: str) -> str:
    """Match a config joint name to the articulation's actual joint name."""
    names = list(robot.data.joint_names)
    if name in names:
        return name
    # Physics Inspector may show joint2L while URDF uses joint2l
    if len(name) > 1 and name[-1] in ("l", "L"):
        alt = name[:-1] + ("L" if name[-1] == "l" else "l")
        if alt in names:
            return alt
    raise KeyError(f"Joint '{name}' not found in {names}")


def patch_joint_pos_limits_on_prim(prim_path: str) -> None:
    """Write Physics Inspector limits onto spawned USD joints before articulation init."""
    import isaacsim.core.utils.stage as stage_utils
    from pxr import Usd, UsdPhysics

    stage = stage_utils.get_current_stage()
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        return

    updated = []
    for prim in Usd.PrimRange(root):
        limit_key = _joint_limit_key(prim.GetName())
        if limit_key is None:
            continue
        lo, hi = JOINT_POS_LIMITS[limit_key]
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            # USD RevoluteJoint limits are in degrees; JOINT_POS_LIMITS uses radians.
            lo, hi = math.degrees(lo), math.degrees(hi)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            joint = UsdPhysics.PrismaticJoint(prim)
        else:
            continue
        joint.GetLowerLimitAttr().Set(lo)
        joint.GetUpperLimitAttr().Set(hi)
        updated.append(prim.GetName())

    if updated:
        print(f"[INFO] Patched USD joint limits for {len(updated)} joints under {prim_path}.")


def _spawn_bimanual_arm_from_usd(prim_path, cfg, translation=None, orientation=None):
    from isaaclab.sim.spawners.from_files.from_files import _spawn_from_usd_file

    prim = _spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)
    patch_joint_pos_limits_on_prim(prim_path)
    return prim


from isaaclab.sim.utils import clone  # noqa: E402

spawn_bimanual_arm_from_usd = clone(_spawn_bimanual_arm_from_usd)
# clone() uses functools.wraps, which copies the INNER function's __name__
# ("_spawn_bimanual_arm_from_usd") onto this wrapped object. IsaacLab's config
# (de)serialization (callable_to_string / string_to_callable, used by the Hydra
# path in rsl_rl train.py) round-trips a callable as "module:__name__" — so the
# wrapper would serialize as ".._spawn_bimanual_arm_from_usd" and resolve BACK to
# the UNWRAPPED inner function, dropping the {ENV_REGEX_NS} path resolution the
# clone wrapper provides and crashing scene creation ("Path must be an absolute
# path: <>"). Restore the naming invariant the standard @clone decorator keeps
# (module-level name == __name__) so the round-trip resolves to THIS wrapper.
# Metadata-only: direct callers (teleop / pick_place / IL) are unaffected.
spawn_bimanual_arm_from_usd.__name__ = "spawn_bimanual_arm_from_usd"
spawn_bimanual_arm_from_usd.__qualname__ = "spawn_bimanual_arm_from_usd"


ARM_V2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        func=spawn_bimanual_arm_from_usd,
        usd_path=_ARM_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # OFF is load-bearing -- it is what makes grasping work. CAD export, so each
            # finger's collider is a convex hull bulging into the jaw gap; ON, the two hulls
            # hold the jaw wider than the box and only one finger touches it (friction was
            # then irrelevant from 0.5 to 220). Defaulted True before, with every scene
            # overriding it. Re-enable only once the finger colliders are real primitives.
            enabled_self_collisions=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos=_DEFAULT_JOINT_POS),
    actuators={
        # AK10-9 V3.0 — shoulder joints 1-2. effort_limit_sim uses the motor's
        # peak rating (53 Nm), not rated (18 Nm): at rated torque the shoulder
        # actuator saturates against gravity load when the arm is extended
        # (largest moment arm), causing the commanded IK target to visibly
        # undershoot on upward reach specifically — this is a simulation-only
        # cap (not applied to real hardware limits), so raising it is safe here.
        # NOTE: do NOT raise effort_limit_sim past the real peak rating (53 Nm)
        # to fix responsiveness — that would let the arm move in ways the real
        # motor can't, which matters for dataset collection (sim-to-real
        # mismatch). Stiffness/damping (below) is the safe knob instead: it
        # only changes how aggressively the PD controller commands torque
        # *within* that same real torque budget. Live testing showed the
        # shoulder visibly lagging the faster-responding elbow/wrist during
        # large reaches (diagnostic target_err in run_quest_bimanual_teleop.py
        # stayed elevated during active movement) — raised ~50% from the
        # original (which was tuned for holding a static pose, not tracking a
        # continuously moving IK target) to close that gap. Watch for shoulder
        # oscillation/overshoot if pushed further.
        "left_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["joint1L", "joint2l"],
            stiffness=2270.0,
            damping=180.0,
            effort_limit_sim=53.0,
            velocity_limit_sim=6.0,
        ),
        # AK80-9 V3.0 — elbow joints 3-5. Same rated (9 Nm) vs peak (22 Nm)
        # saturation concern as the shoulder, above. Raised alongside the
        # shoulder bump (same ~25% factor) — "forearm moves slower than it
        # should" reports track these joints directly, and this is the same
        # safe stiffness/damping knob (torque budget cap unchanged at 22 Nm).
        "left_elbow": ImplicitActuatorCfg(
            joint_names_expr=["joint3l", "joint4l", "joint5l"],
            stiffness=1550.0,
            damping=110.0,
            effort_limit_sim=22.0,
            velocity_limit_sim=6.0,
        ),
        # GL40 KV70 — wrist joint 6. PEAK (0.73), not rated (0.25), same as the shoulder and
        # elbow above; rated was inherited from bimanual_arm_cfg and saturated instantly. It was
        # invisible while the arm was teleported: with the actuators live, joint6l closed -0.2%
        # of its commanded gap per frame against 54-70% for every other joint (measured, 110
        # frames). 0.73 is the real motor peak -- do not go past it, raise stiffness instead.
        "left_wrist": ImplicitActuatorCfg(
            joint_names_expr=["joint6l"],
            stiffness=341.0,
            damping=18.0,
            effort_limit_sim=0.73,
            velocity_limit_sim=6.0,
        ),
        # GL40 KV70 rotary → linkage → two prismatic fingers (see module docstring)
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint7l", "joint8l"],
            stiffness=_GRIPPER_STIFFNESS,
            damping=_GRIPPER_DAMPING,
            effort_limit_sim=_GRIPPER_EFFORT_LIMIT,
            velocity_limit_sim=_GRIPPER_VELOCITY_LIMIT,
        ),
        # Right arm (unsuffixed) — mirrors the left groups above. Was one coarse
        # group (stiffness=1000/damping=100/effort=18) tuned for a static hold;
        # both arms are teleoperated now and need equal torque headroom.
        "right_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["joint1", "joint2"],
            stiffness=2270.0,
            damping=180.0,
            effort_limit_sim=53.0,
            velocity_limit_sim=6.0,
        ),
        "right_elbow": ImplicitActuatorCfg(
            joint_names_expr=["joint3", "joint4", "joint5"],
            stiffness=1550.0,
            damping=110.0,
            effort_limit_sim=22.0,
            velocity_limit_sim=6.0,
        ),
        "right_wrist": ImplicitActuatorCfg(
            joint_names_expr=["joint6"],
            stiffness=341.0,
            damping=18.0,
            effort_limit_sim=0.73,  # peak, see left_wrist
            velocity_limit_sim=6.0,
        ),
        # Right gripper — same coupled-prismatic hold as the left
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint7", "joint8"],
            stiffness=_GRIPPER_STIFFNESS,
            damping=_GRIPPER_DAMPING,
            effort_limit_sim=_GRIPPER_EFFORT_LIMIT,
            velocity_limit_sim=_GRIPPER_VELOCITY_LIMIT,
        ),
    },
)
