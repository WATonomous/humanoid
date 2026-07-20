"""Minimal bimanual-arm keyboard teleoperation (left arm only).

Robot: Humanoid_Wato wato_bimanual_arm (bimanual_arm.usd)
Motor specs: https://watonomous.github.io/humanoid-docs/mechanical/index.html
Teleop bindings: https://isaac-sim.github.io/IsaacLab/v2.0.1/source/overview/teleop_imitation.html

  K       Toggle gripper (open/close)
  W/S     Move along x-axis
  A/D     Move along y-axis
  Q/E     Move along z-axis
  Z/X     Rotate along x-axis
  T/G     Rotate along y-axis
  C/V     Rotate along z-axis
  R       Reset left arm to default pose
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

_IL_PKG = Path(__file__).resolve().parents[3] / "il"
_DEFAULT_SIM_SCHEMA = _IL_PKG / "config" / "dataset_schema_sim.yaml"

parser = argparse.ArgumentParser(description="Keyboard teleoperation for the WATonomous bimanual arm (left only).")
parser.add_argument(
    "--record",
    action="store_true",
    help="Record demonstrations (requires: pip install -e autonomy/il[record])",
)
parser.add_argument(
    "--sink",
    type=str,
    default="lerobot,hdf5",
    help="Output sinks when --record: lerobot, hdf5, or lerobot,hdf5",
)
parser.add_argument(
    "--schema",
    type=str,
    default=str(_DEFAULT_SIM_SCHEMA),
    help="dataset_schema YAML (default: autonomy/il/config/dataset_schema_sim.yaml)",
)
parser.add_argument(
    "--dataset_root",
    type=str,
    default=None,
    help="Override record.root from schema (e.g. datasets/record_sim)",
)
parser.add_argument("--num_episodes", type=int, default=10)
parser.add_argument("--task_description", type=str, default="sim keyboard teleop demonstration")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    matrix_from_quat,
    quat_apply,
    quat_inv,
    skew_symmetric_matrix,
    subtract_frame_transforms,
)

from bimanual_arm_cfg import (
    BIMANUAL_ARM_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    RIGHT_ARM_JOINTS,
    RIGHT_EE_BODY,
    RIGHT_GRIPPER_JOINTS,
    LEFT_ARM_JOINTS,
    apply_joint_limits,
    resolve_joint_name,
)

# Fingertip IK (same as task_space_test.py) — kept local, not shared via bimanual_arm_cfg.
_FINGER_TIP_BODIES = ("link7l", "link8l")
_FINGER_DISTAL_TIP_LOCAL = {
    "link7l": (0.13211595, -0.04057075, -0.00434997),
    "link8l": (-0.13211595, -0.04057075, -0.00435003),
}


@configclass
class BimanualSceneCfg(InteractiveSceneCfg):
    """Minimal scene with the WATonomous bimanual arm."""

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


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


def _body_ids(robot, names: tuple[str, ...]) -> list[int]:
    name_to_id = {name: idx for idx, name in enumerate(robot.data.body_names)}
    return [name_to_id[name] for name in names]


def _gripper_tip_pose_b(robot, root_pose_w, wrist_body_id: int, finger_body_ids: list[int]):
    dtype = robot.data.body_pos_w.dtype
    device = robot.data.body_pos_w.device
    tips = []
    for body_name, body_id in zip(_FINGER_TIP_BODIES, finger_body_ids):
        local = torch.tensor([_FINGER_DISTAL_TIP_LOCAL[body_name]], device=device, dtype=dtype)
        body_pos = robot.data.body_pos_w[:, body_id]
        body_quat = robot.data.body_quat_w[:, body_id]
        tips.append(body_pos + quat_apply(body_quat, local))
    tip_pos_w = (tips[0] + tips[1]) * 0.5
    tip_quat_w = robot.data.body_quat_w[:, wrist_body_id]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], tip_pos_w, tip_quat_w
    )


def _tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b):
    base_rot = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jacobian_b = jacobian_w.clone()
    jacobian_b[:, :3, :] = torch.bmm(base_rot, jacobian_b[:, :3, :])
    jacobian_b[:, 3:, :] = torch.bmm(base_rot, jacobian_b[:, 3:, :])
    offset_b = tip_pos_b - wrist_pos_b
    jacobian_b[:, 0:3, :] += torch.bmm(-skew_symmetric_matrix(offset_b), jacobian_b[:, 3:, :])
    return jacobian_b


def _init_recorder(device: str):
    if not args_cli.record:
        return None, None
    if str(_IL_PKG) not in sys.path:
        sys.path.insert(0, str(_IL_PKG))
    try:
        from humanoid_il.record_utils import resolve_config_path
        from humanoid_il.schema import enabled_images, load_yaml
        from humanoid_il.sim_recorder import SimLeRobotRecorder
    except ImportError as exc:
        raise ImportError(
            "Recording requires humanoid-il. Install with:\n"
            "  pip install -e autonomy/il[sim]"
        ) from exc

    schema_path = resolve_config_path(args_cli.schema, anchor=_IL_PKG)
    cfg = load_yaml(schema_path)
    dataset_root = (
        Path(args_cli.dataset_root)
        if args_cli.dataset_root
        else Path((cfg.get("record") or {}).get("root", "datasets/record_sim"))
    )
    cameras = {
        name: {"height": spec["height"], "width": spec["width"]}
        for name, spec in enabled_images(cfg).items()
    }
    recorder = SimLeRobotRecorder(
        task_name=args_cli.task_description,
        repo_id=str(cfg.get("repo_id", "humanoid/sim")),
        dataset_root=dataset_root,
        fps=int(cfg.get("fps", 30)),
        device=device,
        joint_names=list(cfg["joint_names"]),
        cameras=cameras,
        num_episodes=args_cli.num_episodes,
    )
    recorder.init_dataset()
    print(f"[RECORD] Writing to {dataset_root}")
    print("[RECORD] Keys: S=start, N=save episode, D=discard, Esc=stop")
    return recorder, cfg


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    recorder, record_cfg = _init_recorder(sim.device)

    import numpy as np

    if recorder is not None:
        recorder.start_keyboard()

    # Ensure robot buffers are populated before reading limits / joint names
    scene.update(sim_dt)
    apply_joint_limits(robot)

    right_arm_names = [resolve_joint_name(robot, name) for name in RIGHT_ARM_JOINTS]
    right_gripper_names = [resolve_joint_name(robot, name) for name in RIGHT_GRIPPER_JOINTS]
    left_arm_names = [resolve_joint_name(robot, name) for name in LEFT_ARM_JOINTS]

    print(f"[INFO] Robot joints: {robot.data.joint_names}")
    print(f"[INFO] Left arm joints: {right_arm_names}")
    print(f"[INFO] Right arm hold joints: {left_arm_names}")

    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    robot_entity_cfg = SceneEntityCfg("robot", joint_names=right_arm_names, body_names=[RIGHT_EE_BODY])
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    wrist_body_id = robot_entity_cfg.body_ids[0]
    finger_body_ids = _body_ids(robot, _FINGER_TIP_BODIES)

    left_arm_ids = robot_entity_cfg.joint_ids
    right_gripper_ids = _joint_ids(robot, RIGHT_GRIPPER_JOINTS)
    left_joint_ids = _joint_ids(robot, LEFT_ARM_JOINTS)
    right_gripper_ids = _joint_ids(robot, ["joint7", "joint8"])
    left_default_pos = robot.data.default_joint_pos[:, left_joint_ids].clone()

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in RIGHT_GRIPPER_JOINTS]],
        device=sim.device,
    )
    gripper_closed_targets = torch.tensor(
        [[GRIPPER_CLOSED[name] for name in RIGHT_GRIPPER_JOINTS]],
        device=sim.device,
    )

    teleop = Se3Keyboard(pos_sensitivity=0.005, rot_sensitivity=0.05)
    should_reset = False

    def reset_left_arm():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_left_arm)
    teleop.reset()
    print(teleop)
    print("[INFO] Teleoperating left arm only. Right arm is held at default pose.")
    print("[INFO] Click the 3D viewport window, then press W/A/S/D/Q/E to move.")

    debug_steps = 0
    while simulation_app.is_running():
        if recorder is not None and recorder.is_complete:
            print("[RECORD] Session complete.")
            break

        if should_reset:
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            diff_ik_controller.reset()
            teleop.reset()
            should_reset = False

        delta_pose, close_gripper = teleop.advance()
        command = torch.tensor(delta_pose, dtype=torch.float32, device=sim.device).unsqueeze(0)

        if debug_steps < 5 and torch.any(command.abs() > 1e-4):
            print(f"[DEBUG] Keyboard command: {command[0].tolist()}")
            debug_steps += 1

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, left_arm_ids]

        ee_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        tip_pos_b, tip_quat_b = _gripper_tip_pose_b(
            robot, root_pose_w, wrist_body_id, finger_body_ids
        )

        # Relative mode: target = current_tip + delta (stays close, no runaway).
        # Se3Keyboard outputs [dx,dy,dz, drx,dry,drz] all in base frame.
        # pos_sensitivity=0.005 keeps the IK step small enough for the linearization to hold.
        diff_ik_controller.set_command(command, ee_pos=tip_pos_b, ee_quat=tip_quat_b)

        jacobian = _tip_ik_jacobian(
            robot,
            robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, left_arm_ids],
            ee_pos_b,
            tip_pos_b,
        )
        joint_pos_des = diff_ik_controller.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=left_arm_ids)

        if recorder is not None:
            state = joint_pos[0].detach().cpu().numpy().astype(np.float32)
            action = joint_pos_des[0].detach().cpu().numpy().astype(np.float32)
            recorder.tick(action, state, {})

        # Hold gripper fingers at synchronized open/closed pair (one GL40 motor on hardware).
        # High stiffness in cfg + zero velocity target prevents bounce when the arm moves.
        gripper_targets = gripper_closed_targets if close_gripper else gripper_open_targets
        zero_gripper_vel = torch.zeros(1, len(right_gripper_ids), device=sim.device)
        robot.set_joint_position_target(gripper_targets, joint_ids=right_gripper_ids)
        robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=right_gripper_ids)

        # Keep right arm fixed at default pose (including coupled gripper fingers)
        robot.set_joint_position_target(left_default_pos, joint_ids=left_joint_ids)
        robot.set_joint_velocity_target(
            torch.zeros(1, len(right_gripper_ids), device=sim.device),
            joint_ids=right_gripper_ids,
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

    if recorder is not None:
        recorder.finalize()
        print(f"[RECORD] Saved under {recorder.dataset_root}")


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene_cfg = BimanualSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete. Use keyboard to teleoperate the left arm.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
