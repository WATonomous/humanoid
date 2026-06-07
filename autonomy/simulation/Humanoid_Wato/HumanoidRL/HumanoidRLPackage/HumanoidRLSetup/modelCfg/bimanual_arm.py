"""
Bimanual arm articulation config for keyboard teleoperation.

Robot model: Humanoid_Wato/wato_bimanual_arm (armDouble.SLDASM.usd)
Joint limits/poses: Isaac Sim Physics Inspector (/World/armDouble_SLDASM/root_joint)
Motor specs: https://watonomous.github.io/humanoid-docs/mechanical/index.html

  Shoulder joints 1-2  AK10-9 V3.0  18 Nm rated / 53 Nm peak
  Elbow joints 3-5     AK80-9 V3.0   9 Nm rated / 22 Nm peak
  Wrist joint 6        GL40 KV70     0.25 Nm rated / 0.73 Nm peak
  Gripper              GL40 KV70     0.25 Nm rated / 0.73 Nm peak
                       (rotary motor + linkage → prismatic finger travel in URDF)

Only the left arm is actuated for teleop. The right arm is held at the
Physics Inspector default pose below.

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

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_BIMANUAL_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", "..", "wato_bimanual_arm"))
_ARM_USD_PATH = os.path.join(_BIMANUAL_ROOT, "urdf", "armDouble.SLDASM", "armDouble.SLDASM.usd")


def _deg(degrees: float) -> float:
    return degrees * math.pi / 180.0


# --- Joint limits from Physics Inspector ------------------------------------
_REVOLUTE_LIMIT = (-math.pi, math.pi)
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

# --- Default poses from Physics Inspector (revolute: deg -> rad) ------------
_DEFAULT_JOINT_POS = {
    # Right arm — held fixed during teleop
    "joint1": _deg(-140.8),
    "joint2": _deg(55.7),
    "joint3": _deg(-66.0),
    "joint4": _deg(111.4),
    "joint5": _deg(34.8),
    "joint6": _deg(3.5),
    "joint7": -0.05,
    "joint8": 0.05,
    # Left arm — teleoperated
    "joint1L": _deg(139.2),
    "joint2l": _deg(66.1),
    "joint3l": _deg(147.9),
    "joint4l": _deg(-76.5),
    "joint5l": _deg(-76.5),
    "joint6l": _deg(-22.6),
    "joint7l": -0.05,
    "joint8l": 0.05,
}

RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
LEFT_GRIPPER_JOINTS = ["joint7l", "joint8l"]
# Jacobian anchor is the wrist link; IK pose target is the fingertip center (see below).
LEFT_EE_BODY = "link6l"
LEFT_FINGER_TIP_BODIES = ("link7l", "link8l")
# Distal mesh points in each finger link frame (link7l.STL +X, link8l.STL -X).
LEFT_FINGER_DISTAL_TIP_LOCAL = {
    "link7l": (0.13211595, -0.04057075, -0.00434997),
    "link8l": (-0.13211595, -0.04057075, -0.00435003),
}

# Gripper finger targets (joint7: [-0.05, 0], joint8: [0, 0.05])
# Synchronized pair mimics single GL40 motor driving both fingers via linkage.
GRIPPER_OPEN = {"joint7l": -0.05, "joint8l": 0.05}
GRIPPER_CLOSED = {"joint7l": 0.0, "joint8l": 0.0}

# Prismatic gripper PD — tuned for hold during arm motion (not from motor datasheet).
# If fingers bounce when the shoulder moves, raise stiffness; if jittery, raise damping.
_GRIPPER_STIFFNESS = 400.0
_GRIPPER_DAMPING = 40.0
_GRIPPER_EFFORT_LIMIT = 30.0  # N (sim linear-force cap; tune empirically)
_GRIPPER_VELOCITY_LIMIT = 0.2  # m/s


def apply_joint_limits(robot) -> None:
    """Apply Physics Inspector joint limits to the articulation."""
    limits = robot.data.joint_pos_limits.clone()
    updated = []

    for joint_idx, joint_name in enumerate(robot.data.joint_names):
        if joint_name not in JOINT_POS_LIMITS:
            continue
        lo, hi = JOINT_POS_LIMITS[joint_name]
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


def compute_gripper_tip_pos_w(robot, finger_body_ids: list[int]):
    """World-frame midpoint between the distal tips of link7l and link8l."""
    import torch
    from isaaclab.utils.math import quat_apply

    dtype = robot.data.body_pos_w.dtype
    device = robot.data.body_pos_w.device
    tips = []
    for body_name, body_id in zip(LEFT_FINGER_TIP_BODIES, finger_body_ids):
        local = torch.tensor([LEFT_FINGER_DISTAL_TIP_LOCAL[body_name]], device=device, dtype=dtype)
        body_pos = robot.data.body_pos_w[:, body_id]
        body_quat = robot.data.body_quat_w[:, body_id]
        tips.append(body_pos + quat_apply(body_quat, local))
    return (tips[0] + tips[1]) * 0.5


def compute_gripper_tip_pose_w(robot, wrist_body_id: int, finger_body_ids: list[int]):
    """Gripper-tip center pose in world frame (position from fingers, orientation from wrist)."""
    tip_pos_w = compute_gripper_tip_pos_w(robot, finger_body_ids)
    tip_quat_w = robot.data.body_quat_w[:, wrist_body_id]
    return tip_pos_w, tip_quat_w


def compute_gripper_tip_pose_b(robot, root_pose_w, wrist_body_id: int, finger_body_ids: list[int]):
    """Gripper-tip center pose in the robot root frame."""
    from isaaclab.utils.math import subtract_frame_transforms

    tip_pos_w, tip_quat_w = compute_gripper_tip_pose_w(robot, wrist_body_id, finger_body_ids)
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


_ARM_URDF_PATH = os.path.join(_BIMANUAL_ROOT, "urdf", "armDouble.SLDASM", "armDouble.SLDASM.urdf")

BIMANUAL_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=_ARM_URDF_PATH,
        fix_base=True,
        make_instanceable=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos=_DEFAULT_JOINT_POS),
    actuators={
        # AK10-9 V3.0 — shoulder joints 1-2
        "left_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["joint1L", "joint2l"],
            stiffness=757.6,
            damping=60.3,
            effort_limit_sim=18.0,
            velocity_limit_sim=3.0,
        ),
        # AK80-9 V3.0 — elbow joints 3-5
        "left_elbow": ImplicitActuatorCfg(
            joint_names_expr=["joint3l", "joint4l", "joint5l"],
            stiffness=615.5,
            damping=43.5,
            effort_limit_sim=9.0,
            velocity_limit_sim=3.0,
        ),
        # GL40 KV70 — wrist joint 6
        "left_wrist": ImplicitActuatorCfg(
            joint_names_expr=["joint6l"],
            stiffness=170.5,
            damping=9.0,
            effort_limit_sim=0.25,
            velocity_limit_sim=3.0,
        ),
        # GL40 KV70 rotary → linkage → two prismatic fingers (see module docstring)
        "left_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint7l", "joint8l"],
            stiffness=_GRIPPER_STIFFNESS,
            damping=_GRIPPER_DAMPING,
            effort_limit_sim=_GRIPPER_EFFORT_LIMIT,
            velocity_limit_sim=_GRIPPER_VELOCITY_LIMIT,
        ),
        # Right arm revolute joints — hold Physics Inspector default pose
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            stiffness=500.0,
            damping=50.0,
            effort_limit_sim=18.0,
            velocity_limit_sim=3.0,
        ),
        # Right gripper — same coupled-prismatic hold as left
        "right_gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint7", "joint8"],
            stiffness=_GRIPPER_STIFFNESS,
            damping=_GRIPPER_DAMPING,
            effort_limit_sim=_GRIPPER_EFFORT_LIMIT,
            velocity_limit_sim=_GRIPPER_VELOCITY_LIMIT,
        ),
    },
)
