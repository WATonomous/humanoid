import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

SCRIPT_DIR = Path(__file__).resolve().parent
SIMULATION_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(SIMULATION_DIR / "Humanoid_Wato"))

parser = argparse.ArgumentParser(description="Quest hand teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import rclpy
from rclpy.node import Node
from common_msgs.msg import QuestHandPose
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    matrix_from_quat,
    quat_apply_inverse,
    quat_from_matrix,
    quat_inv,
    quat_mul,
    subtract_frame_transforms,
)

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG, LEFT_ARM_CFG

from quest_isaac_teleop.quest_calc import compute_all_targets


LEFT_PALM_BODY = "left_PALM_GAVIN_1DoF_Hinge_v2_1"
RIGHT_PALM_BODY = "PALM_GAVIN_1DoF_Hinge_v2_1"

# WebXR (Y-up, X-right, -Z-forward) -> arm base frame (Z-up, X-forward).
# Applied to wrist *position deltas* only, so the WebXR/arm origin offset
# doesn't matter. Flip a row's sign if a motion axis feels inverted.
QUEST_TO_ARM = torch.tensor(
    [
        [0.0, 0.0, -1.0],  # world +x = quest -z (forward)
        [-1.0, 0.0, 0.0],  # world +y = quest -x (left/right)
        [0.0, 1.0, 0.0],   # world +z = quest +y (up)
    ]
)
WRIST_GAIN = 1.0  # scale from real-world wrist delta (m) to arm palm delta (m)

LEFT_ARM_JOINTS = [
    "left_shoulder_flexion_extension",
    "left_shoulder_abduction_adduction",
    "left_shoulder_rotation",
    "left_elbow_flexion_extension",
    "left_forearm_rotation",
    "left_wrist_extension",
]

RIGHT_ARM_JOINTS = [
    "shoulder_flexion_extension",
    "shoulder_abduction_adduction",
    "shoulder_rotation",
    "elbow_flexion_extension",
    "forearm_rotation",
    "wrist_extension",
]

LEFT_FINGER_JOINTS = [
    "left_mcp_index",
    "left_pip_index",
    "left_dip_index",
    "left_mcp_middle",
    "left_pip_middle",
    "left_dip_middle",
    "left_mcp_ring",
    "left_pip_ring",
    "left_dip_ring",
    "left_mcp_pinky",
    "left_pip_pinky",
    "left_dip_pinky",
    "left_cmc_thumb",
    "left_mcp_thumb",
    "left_ip_thumb",
]

RIGHT_FINGER_JOINTS = [
    "mcp_index",
    "pip_index",
    "dip_index",
    "mcp_middle",
    "pip_middle",
    "dip_middle",
    "mcp_ring",
    "pip_ring",
    "dip_ring",
    "mcp_pinky",
    "pip_pinky",
    "dip_pinky",
    "cmc_thumb",
    "mcp_thumb",
    "ip_thumb",
]


class QuestListener(Node):
    def __init__(self):
        super().__init__("quest_listener")
        self.latest = None
        self.message_count = 0
        self.create_subscription(QuestHandPose, "/quest_teleop", self.cb, 1)

    def cb(self, msg):
        self.latest = msg
        self.message_count += 1


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    right_arm = ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/RightArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=ARM_CFG.init_state.joint_pos,
        ),
    )
    left_arm = LEFT_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/LeftArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.5, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


def joint_indices(available_joint_names, required_joint_names, side):
    missing = [name for name in required_joint_names if name not in available_joint_names]
    if missing:
        raise RuntimeError(f"{side} arm is missing joints: {', '.join(missing)}")
    return [available_joint_names.index(name) for name in required_joint_names]


class ArmWristIK:
    """Delta-control wrist pose IK using Isaac Lab's differential IK on the in-scene arm.

    Tracks the Quest wrist *pose* (position + orientation): the palm is
    commanded to its startup pose plus the (frame-mapped, scaled) position
    delta and the relative rotation the real wrist has undergone since the
    first message. Matching orientation as well as position constrains the
    6-DOF arm so the elbow settles into a human-like configuration.

    No second physics/kinematics model is loaded -- it reuses the
    articulation's PhysX Jacobian, so there's no MuJoCo/Omniverse conflict.

    Everything is anchored relative to a "home" captured on the first message,
    so the absolute palm-frame convention in the URDF cancels out: at home the
    relative rotation is identity and the arm holds its startup pose.
    """

    def __init__(self, scene, robot_key, joint_names, palm_body, device,
                 label="", debug_every=0, lambda_val=0.05):
        self._robot = scene[robot_key]
        self._device = device
        self._label = label or robot_key
        self._debug_every = debug_every  # log every Nth update; 0 disables
        self._update_count = 0
        self._entity = SceneEntityCfg(robot_key, joint_names=joint_names, body_names=[palm_body])
        self._entity.resolve(scene)
        self._body_id = self._entity.body_ids[0]
        self._joint_ids = self._entity.joint_ids
        # For a fixed-base robot the root body is absent from the returned
        # Jacobians, so the Jacobian row index is one less than the body index.
        self._ee_jacobi_idx = self._body_id - 1 if self._robot.is_fixed_base else self._body_id
        self._controller = DifferentialIKController(
            DifferentialIKControllerCfg(
                command_type="pose", use_relative_mode=False, ik_method="dls",
                ik_params={"lambda_val": lambda_val},
            ),
            num_envs=scene.num_envs,
            device=device,
        )
        self._rot = QUEST_TO_ARM.to(device)
        # Same axis remap as a quaternion, to carry wrist rotation from the
        # WebXR frame into the sim world frame by conjugation.
        self._rot_quat = quat_from_matrix(self._rot.unsqueeze(0))  # (1, 4)
        # Joint position limits for the arm joints, used only for diagnostics.
        self._joint_limits = self._robot.data.joint_pos_limits[:, self._joint_ids, :]
        self._quest_home = None        # quest wrist xyz at first message
        self._wrist_home_quat = None   # quest wrist quaternion (w,x,y,z) at first message
        self._home_ee_pos_b = None     # palm position in arm base frame at first message
        self._home_ee_quat_b = None    # palm orientation in arm base frame at first message

    def _ee_in_base(self):
        ee_pose_w = self._robot.data.body_state_w[:, self._body_id, 0:7]
        root_pose_w = self._robot.data.root_state_w[:, 0:7]
        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        return ee_pos_b, ee_quat_b, root_pose_w[:, 3:7]

    def update(self, wrist_pose):
        ee_pos_b, ee_quat_b, root_quat_w = self._ee_in_base()
        p, o = wrist_pose.position, wrist_pose.orientation
        quest_xyz = torch.tensor([p.x, p.y, p.z], device=self._device, dtype=ee_pos_b.dtype)
        # geometry_msgs orientation is (x,y,z,w); Isaac quaternions are (w,x,y,z).
        wrist_quat = torch.tensor(
            [[o.w, o.x, o.y, o.z]], device=self._device, dtype=ee_pos_b.dtype
        )
        # First message: anchor home references; the arm holds its startup pose.
        if self._quest_home is None:
            self._quest_home = quest_xyz
            self._wrist_home_quat = wrist_quat.clone()
            self._home_ee_pos_b = ee_pos_b.clone()
            self._home_ee_quat_b = ee_quat_b.clone()

        # --- Position target ---
        # Map the wrist delta to a world-frame palm delta, then express it in this
        # arm's base frame so both arms (different root orientations) track the
        # same world direction.
        delta_world = (self._rot @ (quest_xyz - self._quest_home)) * WRIST_GAIN
        delta_base = quat_apply_inverse(root_quat_w, delta_world.unsqueeze(0))
        target_pos_b = self._home_ee_pos_b + delta_base

        # --- Orientation target ---
        # Relative wrist rotation since home, in the WebXR frame, carried into the
        # sim world frame (conjugate by the axis remap), then into the arm base
        # frame (conjugate by the root orientation), then applied to the home palm
        # orientation. Using a relative rotation makes the URDF palm-frame
        # convention irrelevant.
        dq_quest = quat_mul(wrist_quat, quat_inv(self._wrist_home_quat))
        dq_world = quat_mul(quat_mul(self._rot_quat, dq_quest), quat_inv(self._rot_quat))
        dq_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_world), root_quat_w)
        target_quat_b = quat_mul(dq_base, self._home_ee_quat_b)

        command = torch.cat([target_pos_b, target_quat_b], dim=-1)  # (N, 7)
        self._controller.set_command(command)
        # get_jacobians() returns the Jacobian in the WORLD frame; our ee pose and
        # error are in the arm base frame, so rotate the Jacobian into the base
        # frame too. Without this, a non-identity root orientation (the right arm
        # is rotated 180deg about Z) flips the error mapping and the IK diverges.
        jacobian_w = self._robot.root_physx_view.get_jacobians()[:, self._ee_jacobi_idx, :, self._joint_ids]
        base_rot = matrix_from_quat(quat_inv(self._robot.data.root_quat_w))
        jacobian = torch.empty_like(jacobian_w)
        jacobian[:, :3, :] = torch.bmm(base_rot, jacobian_w[:, :3, :])
        jacobian[:, 3:, :] = torch.bmm(base_rot, jacobian_w[:, 3:, :])
        joint_pos = self._robot.data.joint_pos[:, self._joint_ids]
        joint_pos_des = self._controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        self._robot.set_joint_position_target(joint_pos_des, joint_ids=self._joint_ids)

        self._update_count += 1
        if self._debug_every and self._update_count % self._debug_every == 0:
            self._log_debug(quest_xyz, target_pos_b, target_quat_b, ee_pos_b, ee_quat_b,
                            joint_pos, joint_pos_des)

    def _log_debug(self, quest_xyz, target_pos_b, target_quat_b, ee_pos_b, ee_quat_b,
                   joint_pos, joint_pos_des):
        def fmt(t):
            return "[" + ", ".join(f"{v:+.3f}" for v in t.flatten().tolist()) + "]"

        pos_err = (target_pos_b - ee_pos_b).norm().item()
        # Angle between current and target palm orientation (radians).
        dq = quat_mul(target_quat_b, quat_inv(ee_quat_b))
        rot_err = 2.0 * torch.acos(dq[:, 0].abs().clamp(max=1.0)).item()
        delta_q = (joint_pos_des - joint_pos).abs().max().item()
        lo = self._joint_limits[0, :, 0]
        hi = self._joint_limits[0, :, 1]
        # Per-joint fraction of range used by the *desired* position (0=lo, 1=hi).
        frac = ((joint_pos_des[0] - lo) / (hi - lo).clamp_min(1e-6)).tolist()
        at_limit = [i for i, f in enumerate(frac) if f <= 0.02 or f >= 0.98]
        print(
            f"[ik:{self._label}] quest_d={fmt(quest_xyz - self._quest_home)} "
            f"ee={fmt(ee_pos_b)} tgt={fmt(target_pos_b)} pos_err={pos_err:.4f} "
            f"rot_err={rot_err:.3f} max|dq|={delta_q:.4f} q_des={fmt(joint_pos_des)} "
            f"limit_frac={[round(f, 2) for f in frac]} at_limit_joints={at_limit}",
            flush=True,
        )


def apply_hand_targets(arm, joint_indices, joint_names, hand_joints, side, logger):
    if len(hand_joints) != 75:
        side_label = side or "right"
        logger.warning(f"Expected 75 {side_label} hand values, got {len(hand_joints)}")
        return

    targets = compute_all_targets(hand_joints, side)
    target_positions = arm.data.joint_pos[:, joint_indices].clone()
    for target_idx, joint_name in enumerate(joint_names):
        target_positions[0, target_idx] = targets[joint_name]
    arm.set_joint_position_target(target_positions, joint_ids=joint_indices)


def run_simulator(sim, scene, listener):
    sim_dt = sim.get_physics_dt()
    left_arm = scene["left_arm"]
    right_arm = scene["right_arm"]

    left_joint_names = left_arm.data.joint_names
    left_finger_indices = joint_indices(left_joint_names, LEFT_FINGER_JOINTS, "left")

    right_joint_names = right_arm.data.joint_names
    right_finger_indices = joint_indices(right_joint_names, RIGHT_FINGER_JOINTS, "right")

    # Force the configured startup pose and hold it as the initial target, so the
    # arms begin in the human-like pose (and don't sag toward zero) before the
    # first Quest message arrives. The printed values also confirm whether the
    # edited init_state.joint_pos actually loaded.
    for name, arm, arm_joints in (("left", left_arm, LEFT_ARM_JOINTS),
                                  ("right", right_arm, RIGHT_ARM_JOINTS)):
        default_pos = arm.data.default_joint_pos.clone()
        arm.write_joint_state_to_sim(default_pos, arm.data.default_joint_vel.clone())
        arm.set_joint_position_target(default_pos)
        arm_idx = joint_indices(list(arm.data.joint_names), arm_joints, name)
        shown = default_pos[0, arm_idx].cpu().numpy().round(3)
        print(f"[INFO]: {name} startup arm joint_pos = {dict(zip(arm_joints, shown))}", flush=True)
    scene.write_data_to_sim()

    left_ik = ArmWristIK(scene, "left_arm", LEFT_ARM_JOINTS, LEFT_PALM_BODY, sim.device,
                         label="left", debug_every=30)
    right_ik = ArmWristIK(scene, "right_arm", RIGHT_ARM_JOINTS, RIGHT_PALM_BODY, sim.device,
                          label="right", debug_every=30)
    print("[INFO]: Quest teleop ready (fingers + Isaac differential IK)", flush=True)

    while simulation_app.is_running():
        rclpy.spin_once(listener, timeout_sec=0)

        if listener.latest is not None:
            msg = listener.latest

            # Fingers
            apply_hand_targets(
                left_arm,
                left_finger_indices,
                LEFT_FINGER_JOINTS,
                list(msg.left_hand_joints),
                "left",
                listener.get_logger(),
            )
            apply_hand_targets(
                right_arm,
                right_finger_indices,
                RIGHT_FINGER_JOINTS,
                list(msg.right_hand_joints),
                "",
                listener.get_logger(),
            )

            # Arms (delta pose IK from Quest wrist pose)
            left_ik.update(msg.left_wrist)
            right_ik.update(msg.right_wrist)

            listener.latest = None

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    rclpy.init()
    listener = QuestListener()
    try:
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        sim.set_camera_view([0, -2.5, 2.0], [0.0, 0.0, 0.0])

        scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=2.0))
        sim.reset()

        print("[INFO]: Quest teleop ready")
        run_simulator(sim, scene, listener)
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
