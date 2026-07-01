"""Quest bimanual arm teleoperation — runs inside the simulation_il container.

Both arms are controlled via Quest hand tracking.  Hand pose data arrives via
ROS 2 /quest_teleop topic published by quest_teleop_node (also in this container).

The left Quest wrist drives the left arm (joints joint1L–joint6l).
The right Quest wrist drives the right arm (joints joint1–joint6).
Pinching thumb + index closes the corresponding gripper.

IK solver: Pink IK (Pinocchio-based QP with multiple weighted tasks).
  - Absolute pose targets tracked from Quest wrist home position.
  - Better singularity handling than Differential IK via per-task LM damping.
  - Uses WATO_BIMANUAL_IK_CONTROLLER_CFG from bimanual_pink_controller_cfg.py.

Coordinate mapping
------------------
WebXR uses a Y-up frame (X-right, Y-up, -Z-forward).  The robot base is in
a Z-up world frame and is rotated 180° about Z, so the mapping is applied in
two stages (see _QUEST_TO_WORLD and _base_delta below).

Usage
-----
Inside the simulation_il container:
    ISAAC_LAB=/workspace/isaaclab \\
    PYTHONPATH=/workspace/humanoid/autonomy/simulation/quest_isaac_teleop:$PYTHONPATH \\
    /workspace/isaaclab/isaaclab.sh -p \\
        /workspace/humanoid/autonomy/simulation/quest_isaac_teleop/run_quest_bimanual_teleop.py \\
        --device cpu

Or via the helper script:
    ./run_quest_bimanual_teleop.sh
"""

import argparse
import sys
import threading
from pathlib import Path

from isaaclab.app import AppLauncher

# ── path setup (must be before AppLauncher so PYTHONPATH is correct) ─────────
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parent
_KEYBOARD_TELEOP_DIR = _SIM_DIR / "Teleop" / "keyboard-based teleoperation"
sys.path.insert(0, str(_KEYBOARD_TELEOP_DIR))

# Bimanual arm URDF + mesh paths for Pink IK's Pinocchio model
_BIMANUAL_ROOT = _SIM_DIR / "Humanoid_Wato" / "wato_bimanual_arm"
_URDF_PATH = str(_BIMANUAL_ROOT / "urdf" / "armDouble.SLDASM.urdf")
_MESH_DIR = str(_BIMANUAL_ROOT / "meshes")

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Quest bimanual arm teleop (ROS 2, Pink IK)")
parser.add_argument("--gain", type=float, default=2.0,
                    help="Motion gain: metres of EE motion per metre of real wrist motion")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Pinocchio must be imported before AppLauncher starts Isaac Sim.
# Isaac Sim loads its own libboost_python311.so; if that loads first, its Boost
# type registry becomes authoritative and Pinocchio's C++ type registrations
# (StdVec_StdString, etc.) become invisible, causing "No Python class registered"
# errors. Importing here ensures cmeel's Boost wins the soname race.
import pinocchio as _pin_preload  # noqa: F401
del _pin_preload

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── post-launch imports (require Omniverse runtime) ───────────────────────────
# rclpy must be imported after AppLauncher — Omniverse must initialise first.
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from rclpy.node import Node  # noqa: E402
from common_msgs.msg import QuestHandPose  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers.pink_ik import PinkIKController  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
    quat_apply,
    quat_apply_inverse,
    quat_from_matrix,
    quat_inv,
    quat_mul,
    skew_symmetric_matrix,
    subtract_frame_transforms,
)

from bimanual_arm_cfg import (  # noqa: E402
    BIMANUAL_ARM_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_GRIPPER_JOINTS,
    apply_joint_limits,
    resolve_joint_name,
)
from quest_isaac_teleop.bimanual_pink_controller_cfg import (  # noqa: E402
    ARM_JOINTS,
    RIGHT_EE_LINK,
    WATO_BIMANUAL_IK_CONTROLLER_CFG,
)

# ── constants ─────────────────────────────────────────────────────────────────

_RIGHT_EE_BODY = RIGHT_EE_LINK          # "link6"
_RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
_RIGHT_GRIPPER_OPEN = {"joint7": -0.05, "joint8": 0.05}
_RIGHT_GRIPPER_CLOSED = {"joint7": 0.0, "joint8": 0.0}

# WebXR (Y-up: X-right, Y-up, -Z-forward) → simulation world frame (Z-up)
# det = +1 (proper rotation) — required so quat_from_matrix gives correct orientation.
# Depth: quest +Z (toward body) → world −X → ×depth_sign(−1) → base −X (arm retracts) ✓
# Lateral: quest +X (right) → world −Y → base +Y ✓
# Vertical: quest +Y (up) → world +Z ✓
_QUEST_TO_WORLD = torch.tensor(
    [[0.0, 0.0, -1.0],   # world +X = quest -Z
     [-1.0, 0.0, 0.0],   # world +Y = quest -X
     [0.0, 1.0, 0.0]],   # world +Z = quest +Y
    dtype=torch.float32,
)
_DEPTH_SIGN = torch.tensor([1.0, 1.0, 1.0])  # per-axis sign flip on world-frame position delta; tune if an axis is inverted

# Orientation-only correction applied after Quest→world frame remap, in world frame.
# Change this to rotate which physical wrist axis drives which EE rotation axis.
# Common values: [1,0,0,0]=identity, [0,0,0,1]=180°Z, [0.707,0,0,0.707]=90°Z, etc. (w,x,y,z)
_WRIST_ORIENT_OFFSET = torch.tensor([1.0, 0.0, 0.0, 0.0])

_THUMB_TIP_IDX = 4   # thumb-tip in Quest hand joint array
_INDEX_TIP_IDX = 9   # index-finger-tip
_PINCH_CLOSE_M = 0.030   # metres — gripper closes when thumb-index < this
_PINCH_OPEN_M = 0.050    # metres — gripper opens when > this (hysteresis)


# ── helpers ───────────────────────────────────────────────────────────────────

@configclass
class BimanualSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


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
    """Return wrist orientation as (w, x, y, z) — Isaac Lab convention."""
    o = wrist.orientation
    return torch.tensor([o.w, o.x, o.y, o.z], dtype=torch.float32)


def _pinch_dist(hand_joints: list) -> float:
    if len(hand_joints) != 75:
        return float("inf")
    ti, ii = _THUMB_TIP_IDX * 3, _INDEX_TIP_IDX * 3
    thumb = torch.tensor(hand_joints[ti:ti + 3])
    index = torch.tensor(hand_joints[ii:ii + 3])
    return (thumb - index).norm().item()


def _ee_pose_in_base(robot, body_id: int):
    root_pose_w = robot.data.root_state_w[:, 0:7]
    ee_pose_w = robot.data.body_state_w[:, body_id, 0:7]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )


def _to_pin_se3(pos_b: torch.Tensor, quat_wxyz: torch.Tensor) -> pin.SE3:
    """Convert Isaac Lab base-frame pose tensors to a Pinocchio SE3.

    pos_b   : (1, 3) tensor — position in robot base frame
    quat_wxyz: (1, 4) tensor — orientation (w, x, y, z) in robot base frame
    Returns a pin.SE3 suitable for LocalFrameTask.set_target().
    """
    t = pos_b[0].cpu().numpy().astype(np.float64)
    w, x, y, z = quat_wxyz[0].cpu().numpy().astype(np.float64)
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
    return pin.SE3(R, t)


# ── main simulation loop ──────────────────────────────────────────────────────

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene) -> None:
    robot = scene["robot"]
    device = sim.device
    sim_dt = sim.get_physics_dt()
    gain = args_cli.gain

    scene.update(sim_dt)
    apply_joint_limits(robot)

    # Resolve arm joint names (handle case variants like joint1L vs joint1l)
    left_arm_names = [resolve_joint_name(robot, n) for n in LEFT_ARM_JOINTS]
    right_arm_names = [resolve_joint_name(robot, n) for n in ["joint1", "joint2", "joint3",
                                                                "joint4", "joint5", "joint6"]]

    # Entity configs — only used to get EE body IDs for pose tracking
    left_cfg = _make_entity_cfg(scene, left_arm_names, LEFT_EE_BODY)
    right_cfg = _make_entity_cfg(scene, right_arm_names, _RIGHT_EE_BODY)
    left_body_id = left_cfg.body_ids[0]
    right_body_id = right_cfg.body_ids[0]

    # ── Pink IK controller setup ───────────────────────────────────────────────
    # ARM_JOINTS = right (joint1-6) + left (joint1L-joint6l), 12 joints total.
    # We resolve each Pink config name to its Isaac Lab USD name, then get its
    # index in robot.data.joint_names so PinkIKController can build the
    # reordering maps between Isaac Lab and Pinocchio conventions.
    all_joint_names = list(robot.data.joint_names)
    controlled_joint_indices = []
    arm_joints_il = []  # actual Isaac Lab names for the 12 arm joints
    for jname in ARM_JOINTS:
        if jname in all_joint_names:
            controlled_joint_indices.append(all_joint_names.index(jname))
            arm_joints_il.append(jname)
        else:
            # Try case variant (e.g. joint2L vs joint2l)
            idx = next((i for i, n in enumerate(all_joint_names) if n.lower() == jname.lower()), None)
            if idx is None:
                raise ValueError(f"Pink IK arm joint '{jname}' not found in robot joints {all_joint_names}")
            controlled_joint_indices.append(idx)
            arm_joints_il.append(all_joint_names[idx])

    pink_cfg = WATO_BIMANUAL_IK_CONTROLLER_CFG.copy()
    pink_cfg.urdf_path = _URDF_PATH
    pink_cfg.mesh_path = _MESH_DIR
    pink_cfg.joint_names = arm_joints_il        # USD names of the 12 controlled arm joints
    pink_cfg.all_joint_names = all_joint_names  # all USD joint names (16 total incl. grippers)

    pink_controller = PinkIKController(
        cfg=pink_cfg,
        robot_cfg=BIMANUAL_ARM_CFG,
        device=device,
        controlled_joint_indices=controlled_joint_indices,
    )

    # Frame-remap quaternion: maps Quest orientation deltas into world frame
    quest_to_world = _QUEST_TO_WORLD.to(device)
    quest_to_world_quat = quat_from_matrix(quest_to_world.unsqueeze(0))  # (1, 4) wxyz
    depth_sign = _DEPTH_SIGN.to(device)
    wrist_orient_offset = _WRIST_ORIENT_OFFSET.to(device).unsqueeze(0)  # (1, 4) wxyz

    # Gripper joint ids
    left_gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    right_gripper_ids = _joint_ids(robot, _RIGHT_GRIPPER_JOINTS)

    # Gripper position tensors
    left_g_open = torch.tensor(
        [[GRIPPER_OPEN["joint7l"], GRIPPER_OPEN["joint8l"]]], device=device
    )
    left_g_closed = torch.tensor(
        [[GRIPPER_CLOSED["joint7l"], GRIPPER_CLOSED["joint8l"]]], device=device
    )
    right_g_open = torch.tensor(
        [[_RIGHT_GRIPPER_OPEN["joint7"], _RIGHT_GRIPPER_OPEN["joint8"]]], device=device
    )
    right_g_closed = torch.tensor(
        [[_RIGHT_GRIPPER_CLOSED["joint7"], _RIGHT_GRIPPER_CLOSED["joint8"]]], device=device
    )

    # Start from default pose
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(),
        robot.data.default_joint_vel.clone(),
    )
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    # ROS 2 hand pose receiver (spun in a daemon thread so Isaac Sim loop is unblocked)
    rclpy.init()
    receiver = QuestRosReceiver()
    threading.Thread(target=rclpy.spin, args=(receiver,), daemon=True).start()

    # Home references for absolute-pose tracking (captured on first Quest message)
    quest_home_left: torch.Tensor | None = None
    quest_home_right: torch.Tensor | None = None
    quest_home_left_quat: torch.Tensor | None = None
    quest_home_right_quat: torch.Tensor | None = None
    home_ee_pos_b_left: torch.Tensor | None = None
    home_ee_quat_b_left: torch.Tensor | None = None
    home_ee_pos_b_right: torch.Tensor | None = None
    home_ee_quat_b_right: torch.Tensor | None = None

    # Gripper state (hysteresis)
    left_closed = False
    right_closed = False

    # ── Viewport visualization markers ────────────────────────────────────────
    _N_HAND = 25

    def _sphere_cfg(color, radius, opacity=1.0):
        return sim_utils.SphereCfg(
            radius=radius,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, opacity=opacity),
        )

    left_joint_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/left_hand_joints",
        markers={"sphere": _sphere_cfg((0.2, 0.5, 1.0), 0.012)},
    ))
    right_joint_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/right_hand_joints",
        markers={"sphere": _sphere_cfg((1.0, 0.3, 0.1), 0.012)},
    ))
    left_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/left_ik_target",
        markers={"sphere": _sphere_cfg((0.2, 0.5, 1.0), 0.05, opacity=0.3)},
    ))
    right_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/right_ik_target",
        markers={"sphere": _sphere_cfg((1.0, 0.3, 0.1), 0.05, opacity=0.3)},
    ))

    def _joints_world(hand_joints_list, wrist_xyz, wrist_target_w):
        """Map 75-float Quest joint array to (N_HAND, 3) world-frame positions.

        Joint 0 (wrist) coincides with the IK target (wrist_target_w). All other
        joints are at their real-world-scale offsets from the current wrist position,
        rotated into the sim world frame via quest_to_world. No gain is applied to
        inter-joint distances — the hand appears at its actual physical size.
        """
        xyz = torch.tensor(hand_joints_list, dtype=torch.float32, device=device).reshape(_N_HAND, 3)
        offsets_w = (quest_to_world @ (xyz - wrist_xyz).T).T  # (N_HAND, 3), real-world scale
        return wrist_target_w.expand(_N_HAND, -1) + offsets_w

    print("[Quest] Ready. Waiting for /quest_teleop messages.")
    print("[Quest] Connect the Quest browser to start streaming hand data.")

    while simulation_app.is_running():
        msg = receiver.poll()

        if msg is not None:
            left_xyz_q = _wrist_xyz(msg.right_wrist).to(device)
            right_xyz_q = _wrist_xyz(msg.left_wrist).to(device)
            left_quat = _wrist_quat_wxyz(msg.right_wrist).to(device)
            right_quat = _wrist_quat_wxyz(msg.left_wrist).to(device)
            left_joints = list(msg.right_hand_joints)
            right_joints = list(msg.left_hand_joints)

            root_quat_w = robot.data.root_state_w[:, 3:7]  # (1, 4)
            ee_pos_b_l, ee_quat_b_l = _ee_pose_in_base(robot, left_body_id)
            ee_pos_b_r, ee_quat_b_r = _ee_pose_in_base(robot, right_body_id)

            if quest_home_left is None:
                # First message: anchor home references so arm holds its current pose.
                quest_home_left = left_xyz_q.clone()
                quest_home_right = right_xyz_q.clone()
                quest_home_left_quat = left_quat.clone()
                quest_home_right_quat = right_quat.clone()
                home_ee_pos_b_left = ee_pos_b_l.clone()
                home_ee_quat_b_left = ee_quat_b_l.clone()
                home_ee_pos_b_right = ee_pos_b_r.clone()
                home_ee_quat_b_right = ee_quat_b_r.clone()
                # Sync Pink task targets to the actual Isaac Sim EE poses at home.
                right_task = pink_controller.cfg.variable_input_tasks[0]
                left_task = pink_controller.cfg.variable_input_tasks[1]
                right_task.set_target(_to_pin_se3(home_ee_pos_b_right, home_ee_quat_b_right))
                left_task.set_target(_to_pin_se3(home_ee_pos_b_left, home_ee_quat_b_left))
                print(f"[Quest] First wrist data — left: {left_xyz_q.tolist()}, "
                      f"right: {right_xyz_q.tolist()}", flush=True)

            # ── Absolute position target in robot base frame ───────────────────
            # Quest wrist delta → world frame (via frame-remap matrix) → base frame.
            left_delta_w = (quest_to_world @ (left_xyz_q - quest_home_left)) * gain * depth_sign
            right_delta_w = (quest_to_world @ (right_xyz_q - quest_home_right)) * gain * depth_sign
            target_pos_b_left = (
                home_ee_pos_b_left + quat_apply_inverse(root_quat_w, left_delta_w.unsqueeze(0))
            )
            target_pos_b_right = (
                home_ee_pos_b_right + quat_apply_inverse(root_quat_w, right_delta_w.unsqueeze(0))
            )

            # ── Absolute orientation target in robot base frame ────────────────
            # Relative wrist rotation since home, remapped through the axis-remap
            # quaternion (Quest→world) then into the robot base frame, applied to
            # the home palm orientation.  Matches the approach in quest_Isaac.py.
            dq_left = quat_mul(left_quat.unsqueeze(0), quat_inv(quest_home_left_quat.unsqueeze(0)))
            dq_left_world = quat_mul(quat_mul(quest_to_world_quat, dq_left),
                                     quat_inv(quest_to_world_quat))
            dq_left_world = quat_mul(quat_mul(wrist_orient_offset, dq_left_world),
                                     quat_inv(wrist_orient_offset))
            dq_left_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_left_world), root_quat_w)
            target_quat_b_left = quat_mul(dq_left_base, home_ee_quat_b_left)

            dq_right = quat_mul(right_quat.unsqueeze(0), quat_inv(quest_home_right_quat.unsqueeze(0)))
            dq_right_world = quat_mul(quat_mul(quest_to_world_quat, dq_right),
                                      quat_inv(quest_to_world_quat))
            dq_right_world = quat_mul(quat_mul(wrist_orient_offset, dq_right_world),
                                      quat_inv(wrist_orient_offset))
            dq_right_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_right_world), root_quat_w)
            target_quat_b_right = quat_mul(dq_right_base, home_ee_quat_b_right)

            # ── Update Pink IK task targets ────────────────────────────────────
            # variable_input_tasks[0] = right EE (link6), [1] = left EE (link6l).
            # Targets are pin.SE3 in the robot base frame.
            pink_controller.cfg.variable_input_tasks[0].set_target(
                _to_pin_se3(target_pos_b_right, target_quat_b_right)
            )
            pink_controller.cfg.variable_input_tasks[1].set_target(
                _to_pin_se3(target_pos_b_left, target_quat_b_left)
            )

            # ── Viewport marker updates ────────────────────────────────────────
            # Target spheres: home EE world pos + the same world-frame delta the IK uses
            # (includes gain). This is mathematically identical to
            # root_pos_w + quat_apply(root_quat_w, target_pos_b) but avoids a
            # potential quat_apply sign issue on X by adding delta_w directly.
            _rp = robot.data.root_state_w[:, :3]
            home_w_l = _rp + quat_apply(root_quat_w, home_ee_pos_b_left)
            home_w_r = _rp + quat_apply(root_quat_w, home_ee_pos_b_right)
            tgt_l_w = home_w_l + left_delta_w.unsqueeze(0)
            tgt_r_w = home_w_r + right_delta_w.unsqueeze(0)
            left_target_vis.visualize(translations=tgt_l_w)
            right_target_vis.visualize(translations=tgt_r_w)
            # Joint spheres: wrist (joint 0) anchored to IK target; other joints at
            # real-world-scale offset from the current wrist, same axis mapping.
            left_joint_vis.visualize(translations=_joints_world(left_joints, left_xyz_q, tgt_l_w))
            right_joint_vis.visualize(translations=_joints_world(right_joints, right_xyz_q, tgt_r_w))

            # ── Gripper hysteresis — pinch = close, release = open ─────────────
            ld = _pinch_dist(left_joints)
            if ld < _PINCH_CLOSE_M:
                left_closed = False
            elif ld > _PINCH_OPEN_M:
                left_closed = True

            rd = _pinch_dist(right_joints)
            if rd < _PINCH_CLOSE_M:
                right_closed = False
            elif rd > _PINCH_OPEN_M:
                right_closed = True

        # ── Pink IK solve ──────────────────────────────────────────────────────
        # compute() takes ALL joint positions (numpy, Isaac Lab ordering) and
        # returns target positions for the 12 CONTROLLED arm joints only.
        curr_joint_pos_np = robot.data.joint_pos[0].cpu().numpy()
        target_arm_pos = pink_controller.compute(curr_joint_pos_np, sim_dt)  # (12,) tensor

        robot.set_joint_position_target(
            target_arm_pos.unsqueeze(0), joint_ids=controlled_joint_indices
        )

        # ── Gripper targets ────────────────────────────────────────────────────
        robot.set_joint_position_target(
            left_g_closed if left_closed else left_g_open, joint_ids=left_gripper_ids
        )
        robot.set_joint_position_target(
            right_g_closed if right_closed else right_g_open, joint_ids=right_gripper_ids
        )
        robot.set_joint_velocity_target(
            torch.zeros(1, len(left_gripper_ids), device=device), joint_ids=left_gripper_ids
        )
        robot.set_joint_velocity_target(
            torch.zeros(1, len(right_gripper_ids), device=device), joint_ids=right_gripper_ids
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene = InteractiveScene(BimanualSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    print("[Quest] Simulation ready.")
    run_simulator(sim, scene)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
