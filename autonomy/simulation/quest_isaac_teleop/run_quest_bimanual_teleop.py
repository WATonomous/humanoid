"""Quest bimanual arm teleoperation — runs inside the simulation_il container.

Both arms are controlled via Quest hand tracking.  Hand pose data arrives as
JSON UDP packets sent by quest_teleop_bridge.py (running in the teleop
container on the same host network).

The left Quest wrist drives the left arm (joints joint1L–joint6l).
The right Quest wrist drives the right arm (joints joint1–joint6).
Pinching thumb + index closes the corresponding gripper.

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
import json
import socket
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

# ── path setup (must be before AppLauncher so PYTHONPATH is correct) ─────────
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parent
_KEYBOARD_TELEOP_DIR = _SIM_DIR / "Teleop" / "keyboard-based teleoperation"
sys.path.insert(0, str(_KEYBOARD_TELEOP_DIR))

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Quest bimanual arm teleop (UDP bridge)")
parser.add_argument("--port", type=int, default=19090,
                    help="UDP port for hand pose packets from quest_teleop_bridge.py")
parser.add_argument("--gain", type=float, default=4.0,
                    help="Motion gain: metres of EE motion per metre of real wrist motion")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── post-launch imports (require Omniverse runtime) ───────────────────────────
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
    matrix_from_quat,
    quat_apply,
    quat_apply_inverse,
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

# ── constants ─────────────────────────────────────────────────────────────────

# Right arm: 6 revolute joints (no gripper — controlled separately)
_RIGHT_IK_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_RIGHT_EE_BODY = "link6"
_RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
_RIGHT_GRIPPER_OPEN = {"joint7": -0.05, "joint8": 0.05}
_RIGHT_GRIPPER_CLOSED = {"joint7": 0.0, "joint8": 0.0}

# WebXR (Y-up: X-right, Y-up, -Z-forward) → simulation world frame (Z-up)
_QUEST_TO_WORLD = torch.tensor(
    [[0.0, 0.0, -1.0],   # world +X (forward) = quest -Z
     [-1.0, 0.0, 0.0],   # world +Y (left)    = quest -X
     [0.0, 1.0, 0.0]],   # world +Z (up)      = quest +Y
    dtype=torch.float32,
)

# WebXR joint indices (each joint = 3 floats xyz; 25 joints = 75 floats)
_THUMB_TIP_IDX = 4   # thumb-tip
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


class UdpHandReceiver:
    """Non-blocking UDP socket that keeps the most recent hand-pose packet."""

    def __init__(self, port: int) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setblocking(False)
        self._latest: dict | None = None
        print(f"[Quest] Listening for hand poses on UDP :{port}")

    def poll(self) -> dict | None:
        """Drain the receive buffer; return the most recent packet or None."""
        while True:
            try:
                data, _ = self._sock.recvfrom(65536)
                self._latest = json.loads(data.decode())
            except BlockingIOError:
                break
        return self._latest


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {n: i for i, n in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, n)] for n in names]


def _make_entity_cfg(scene, joint_names: list[str], ee_body: str) -> SceneEntityCfg:
    cfg = SceneEntityCfg("robot", joint_names=joint_names, body_names=[ee_body])
    cfg.resolve(scene)
    return cfg


def _make_diff_ik(scene, device: str) -> DifferentialIKController:
    cfg = DifferentialIKControllerCfg(
        command_type="pose",
        use_relative_mode=True,
        ik_method="dls",
    )
    return DifferentialIKController(cfg, num_envs=scene.num_envs, device=device)


def _wrist_xyz(wrist_dict: dict) -> torch.Tensor:
    p = wrist_dict["position"]
    return torch.tensor([p["x"], p["y"], p["z"]], dtype=torch.float32)


def _wrist_quat_wxyz(wrist_dict: dict) -> torch.Tensor:
    """Return wrist orientation as (w, x, y, z) — Isaac Lab convention."""
    o = wrist_dict["orientation"]
    return torch.tensor([o["w"], o["x"], o["y"], o["z"]], dtype=torch.float32)


def _rot_delta_to_axis_angle(q_curr: torch.Tensor, q_prev: torch.Tensor) -> torch.Tensor:
    """Relative rotation q_prev→q_curr expressed as axis-angle (3D) in world frame."""
    dq = quat_mul(q_curr.unsqueeze(0), quat_inv(q_prev.unsqueeze(0))).squeeze(0)
    # dq = (w, x, y, z); axis-angle = 2*acos(|w|) * axis
    w = dq[0].clamp(-1.0, 1.0)
    angle = 2.0 * torch.acos(w.abs())
    sin_half = torch.sqrt(1.0 - w * w).clamp(min=1e-6)
    axis = dq[1:] / sin_half
    return axis * angle


def _pinch_dist(hand_joints: list) -> float:
    if len(hand_joints) != 75:
        return float("inf")
    ti, ii = _THUMB_TIP_IDX * 3, _INDEX_TIP_IDX * 3
    thumb = torch.tensor(hand_joints[ti:ti + 3])
    index = torch.tensor(hand_joints[ii:ii + 3])
    return (thumb - index).norm().item()


def _base_delta(delta_world: torch.Tensor, root_quat_w: torch.Tensor) -> torch.Tensor:
    """Convert a world-frame delta into the robot root frame."""
    return quat_apply_inverse(root_quat_w, delta_world.unsqueeze(0)).squeeze(0)


def _ee_pose_in_base(robot, body_id: int):
    root_pose_w = robot.data.root_state_w[:, 0:7]
    ee_pose_w = robot.data.body_state_w[:, body_id, 0:7]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )


def _arm_jacobian(robot, jacobi_idx: int, joint_ids) -> torch.Tensor:
    """Return the Jacobian rotated into the robot root frame."""
    base_rot = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jac_w = robot.root_physx_view.get_jacobians()[:, jacobi_idx, :, joint_ids]
    jac_b = jac_w.clone()
    jac_b[:, :3, :] = torch.bmm(base_rot, jac_b[:, :3, :])
    jac_b[:, 3:, :] = torch.bmm(base_rot, jac_b[:, 3:, :])
    return jac_b


# ── main simulation loop ──────────────────────────────────────────────────────

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene) -> None:
    robot = scene["robot"]
    device = sim.device
    sim_dt = sim.get_physics_dt()
    gain = args_cli.gain

    scene.update(sim_dt)
    apply_joint_limits(robot)

    # Resolve joint names (handle case variants like joint2L vs joint2l)
    left_arm_names = [resolve_joint_name(robot, n) for n in LEFT_ARM_JOINTS]
    right_arm_names = [resolve_joint_name(robot, n) for n in _RIGHT_IK_JOINTS]

    # Entity configs for DiffIK
    left_cfg = _make_entity_cfg(scene, left_arm_names, LEFT_EE_BODY)
    right_cfg = _make_entity_cfg(scene, right_arm_names, _RIGHT_EE_BODY)

    left_arm_ids = left_cfg.joint_ids
    right_arm_ids = right_cfg.joint_ids

    left_body_id = left_cfg.body_ids[0]
    right_body_id = right_cfg.body_ids[0]

    # Jacobian row indices (fixed-base arm: body_id - 1)
    left_jacobi_idx = left_body_id - 1 if robot.is_fixed_base else left_body_id
    right_jacobi_idx = right_body_id - 1 if robot.is_fixed_base else right_body_id

    # Debug: confirm joint and body IDs resolved correctly
    print(f"[Debug] Left arm joint names:  {left_arm_names}", flush=True)
    print(f"[Debug] Left arm joint IDs:    {list(left_arm_ids)}", flush=True)
    print(f"[Debug] Left EE body ID:       {left_body_id}", flush=True)
    print(f"[Debug] Right arm joint names: {right_arm_names}", flush=True)
    print(f"[Debug] Right arm joint IDs:   {list(right_arm_ids)}", flush=True)
    print(f"[Debug] Right EE body ID:      {right_body_id}", flush=True)

    # DiffIK controllers
    left_ik = _make_diff_ik(scene, device)
    right_ik = _make_diff_ik(scene, device)

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

    # UDP hand pose receiver
    receiver = UdpHandReceiver(args_cli.port)

    # Previous wrist poses for delta computation
    prev_left: torch.Tensor | None = None
    prev_right: torch.Tensor | None = None
    prev_left_quat: torch.Tensor | None = None
    prev_right_quat: torch.Tensor | None = None

    # Gripper state (hysteresis)
    left_closed = False
    right_closed = False

    quest_to_world = _QUEST_TO_WORLD.to(device)
    zero_cmd = torch.zeros(1, 6, device=device)

    print("[Quest] Ready. Start quest_teleop_bridge.py in the teleop container,")
    print("[Quest] then connect the Quest browser to start streaming hand data.")

    while simulation_app.is_running():
        msg = receiver.poll()

        if msg is not None:
            left_xyz_q = _wrist_xyz(msg["left_wrist"]).to(device)
            right_xyz_q = _wrist_xyz(msg["right_wrist"]).to(device)
            left_quat = _wrist_quat_wxyz(msg["left_wrist"]).to(device)
            right_quat = _wrist_quat_wxyz(msg["right_wrist"]).to(device)
            left_joints = msg.get("left_hand_joints", [])
            right_joints = msg.get("right_hand_joints", [])

            root_quat_w = robot.data.root_state_w[:, 3:7]  # (1, 4)

            if prev_left is None:
                prev_left = left_xyz_q.clone()
                prev_right = right_xyz_q.clone()
                prev_left_quat = left_quat.clone()
                prev_right_quat = right_quat.clone()
                print(f"[Quest] First wrist data — left: {left_xyz_q.tolist()}, right: {right_xyz_q.tolist()}", flush=True)

            # Position delta: WebXR frame → world frame → robot base frame
            left_delta_w = (quest_to_world @ (left_xyz_q - prev_left)) * gain
            right_delta_w = (quest_to_world @ (right_xyz_q - prev_right)) * gain
            prev_left = left_xyz_q.clone()
            prev_right = right_xyz_q.clone()

            left_delta_b = _base_delta(left_delta_w, root_quat_w)
            right_delta_b = _base_delta(right_delta_w, root_quat_w)

            # Rotation delta: relative wrist rotation → axis-angle in world → base frame
            left_rot_w = _rot_delta_to_axis_angle(left_quat, prev_left_quat).to(device)
            right_rot_w = _rot_delta_to_axis_angle(right_quat, prev_right_quat).to(device)
            prev_left_quat = left_quat.clone()
            prev_right_quat = right_quat.clone()

            left_rot_b = _base_delta(left_rot_w, root_quat_w)
            right_rot_b = _base_delta(right_rot_w, root_quat_w)

            left_cmd = torch.cat([left_delta_b, left_rot_b]).unsqueeze(0)
            right_cmd = torch.cat([right_delta_b, right_rot_b]).unsqueeze(0)

            if left_delta_b.abs().max().item() > 0.0001:
                print(f"[Quest] left_delta_b={[round(v,4) for v in left_delta_b.tolist()]} right_delta_b={[round(v,4) for v in right_delta_b.tolist()]}", flush=True)

            # Gripper hysteresis — pinch = close, release = open
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
        else:
            left_cmd = zero_cmd
            right_cmd = zero_cmd

        # ── Left arm IK ──────────────────────────────────────────────────────
        ee_pos_b_l, ee_quat_b_l = _ee_pose_in_base(robot, left_body_id)
        left_ik.set_command(left_cmd, ee_pos=ee_pos_b_l, ee_quat=ee_quat_b_l)
        jac_l = _arm_jacobian(robot, left_jacobi_idx, left_arm_ids)
        left_des = left_ik.compute(
            ee_pos_b_l, ee_quat_b_l, jac_l, robot.data.joint_pos[:, left_arm_ids]
        )
        robot.set_joint_position_target(left_des, joint_ids=left_arm_ids)

        # ── Right arm IK ─────────────────────────────────────────────────────
        ee_pos_b_r, ee_quat_b_r = _ee_pose_in_base(robot, right_body_id)
        right_ik.set_command(right_cmd, ee_pos=ee_pos_b_r, ee_quat=ee_quat_b_r)
        jac_r = _arm_jacobian(robot, right_jacobi_idx, right_arm_ids)
        right_des = right_ik.compute(
            ee_pos_b_r, ee_quat_b_r, jac_r, robot.data.joint_pos[:, right_arm_ids]
        )
        robot.set_joint_position_target(right_des, joint_ids=right_arm_ids)

        # ── Gripper targets ───────────────────────────────────────────────────
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


if __name__ == "__main__":
    main()
    simulation_app.close()
