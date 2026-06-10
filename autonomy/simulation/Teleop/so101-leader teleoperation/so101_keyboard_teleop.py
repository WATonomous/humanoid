"""Keyboard teleoperation for the SO101 follower in Isaac Sim.

Uses Isaac Lab Se3Keyboard + differential IK on the 5 arm joints; gripper via K toggle.
Optional recording through ``humanoid_il`` (same schema as leader teleop).

Bindings: https://isaac-sim.github.io/IsaacLab/v2.0.1/source/overview/teleop_imitation.html

  K       Toggle gripper (open/close)
  W/S     Move along x-axis
  A/D     Move along y-axis
  Q/E     Move along z-axis
  Z/X     Rotate along x-axis
  T/G     Rotate along y-axis
  C/V     Rotate along z-axis
  R       Reset arm to default pose
  (record: S / N / D / Esc via pynput when --record)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

_IL_PKG = Path(__file__).resolve().parents[3] / "il"
_DEFAULT_SCHEMA = _IL_PKG / "config" / "dataset_schema_so101_sim.yaml"

parser = argparse.ArgumentParser(
    description="Keyboard teleoperation for SO101 follower in Isaac Sim."
)
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
    default=str(_DEFAULT_SCHEMA),
    help="dataset_schema YAML (default: dataset_schema_so101_sim.yaml)",
)
parser.add_argument(
    "--dataset_root",
    type=str,
    default=None,
    help="Override record.root from schema",
)
parser.add_argument("--num_episodes", type=int, default=10)
parser.add_argument(
    "--task_description",
    type=str,
    default="so101 keyboard sim demonstration",
)
parser.add_argument(
    "--robot",
    type=str,
    choices=("follower", "arm_camera"),
    default="follower",
    help="Sim robot USD: follower (LeRobot) or arm_camera (workshop mesh + visible gripper cam)",
)
parser.add_argument(
    "--scene",
    type=str,
    choices=("empty", "vial"),
    default="vial",
    help="Scene: empty (robot only) or vial (lightbox, mat, vials, rack; default)",
)
parser.add_argument(
    "--cameras",
    action="store_true",
    help="Enable ego + external D455 sim cameras (required for vision schema)",
)
parser.add_argument(
    "--domain_rand",
    action="store_true",
    help="Full workshop DR (VialsToRackDR): ±17° mat, HDRI sky, RTX vials, vial/rack/camera/robot color",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

from so101_teleop_runtime import prepare_launcher_args

prepare_launcher_args(args_cli)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene
from isaaclab.utils.math import subtract_frame_transforms

if str(_IL_PKG) not in sys.path:
    sys.path.insert(0, str(_IL_PKG))

ensure_il_on_path()

from humanoid_il.so101_sim import sim_rad_to_leader_raw

from so101_cfg import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    SO101_EE_BODY,
    robot_arm_joint_names,
    robot_gripper_joint_name,
    robot_joint_names,
)
from so101_teleop_runtime import (
    build_scene,
    build_sim_context,
    capture_record_images,
    ensure_il_on_path,
    maybe_apply_domain_rand,
)


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: idx for idx, name in enumerate(robot.data.joint_names)}
    missing = [name for name in names if name not in name_to_id]
    if missing:
        raise KeyError(f"Joint names {missing} not found in {robot.data.joint_names}")
    return [name_to_id[name] for name in names]


def _resolve_ee_body(robot) -> str:
    if SO101_EE_BODY in robot.data.body_names:
        return SO101_EE_BODY
    for candidate in ("gripper_link", "wrist_link", "gripper_frame_link"):
        if candidate in robot.data.body_names:
            print(f"[INFO] Using EE body '{candidate}' (fallback).")
            return candidate
    raise KeyError(f"No EE body found in {robot.data.body_names}")


def _init_record_session():
    if not args_cli.record:
        return None
    try:
        from humanoid_il.frame import joints_to_snapshot
        from humanoid_il.record_utils import resolve_config_path
        from humanoid_il.schema import load_yaml
        from humanoid_il.sim_session import create_sim_record_session
    except ImportError as exc:
        raise ImportError(
            "Recording requires humanoid-il. Install with:\n"
            "  pip install -e autonomy/il[record]"
        ) from exc

    schema_path = resolve_config_path(args_cli.schema, anchor=_IL_PKG)
    cfg = load_yaml(schema_path)
    record_root = (
        Path(args_cli.dataset_root)
        if args_cli.dataset_root
        else Path((cfg.get("record") or {}).get("root", "datasets/record_so101_sim"))
    )

    session = create_sim_record_session(
        cfg,
        record_root=record_root,
        sink=args_cli.sink,
        num_episodes=args_cli.num_episodes,
        task_description=args_cli.task_description,
        auto_start=False,
    )
    print(f"[RECORD] Writing to {session.output_dir} (sinks: {args_cli.sink})")
    print("[RECORD] Keys: S=start, N=save episode, D=discard, Esc=stop")
    return session, joints_to_snapshot


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    all_joint_ids = _joint_ids(robot, robot_joint_names(args_cli.robot))
    arm_joint_ids = _joint_ids(robot, robot_arm_joint_names(args_cli.robot))
    gripper_joint_id = _joint_ids(robot, [robot_gripper_joint_name(args_cli.robot)])[0]

    scene.update(sim_dt)
    default_pos = robot.data.default_joint_pos.clone()
    default_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(default_pos, default_vel)

    ee_body = _resolve_ee_body(robot)
    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="pose", use_relative_mode=True, ik_method="dls"
    )
    diff_ik_controller = DifferentialIKController(
        diff_ik_cfg, num_envs=scene.num_envs, device=sim.device
    )

    robot_entity_cfg = SceneEntityCfg(
        "robot", joint_names=robot_arm_joint_names(args_cli.robot), body_names=[ee_body]
    )
    robot_entity_cfg.resolve(scene)
    ee_jacobi_idx = (
        robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    )

    record_ctx = _init_record_session()
    record_session = record_ctx[0] if record_ctx else None
    joints_to_snapshot = record_ctx[1] if record_ctx else None

    gripper_open = torch.tensor([[GRIPPER_OPEN]], device=sim.device)
    gripper_closed = torch.tensor([[GRIPPER_CLOSED]], device=sim.device)

    teleop = Se3Keyboard(pos_sensitivity=0.005, rot_sensitivity=0.05)
    should_reset = False

    def reset_robot():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_robot)
    teleop.reset()
    print(teleop)
    print("[INFO] Click the 3D viewport, then use WASD/QE/etc. to move the arm.")

    while simulation_app.is_running():
        if record_session is not None and record_session.is_complete:
            print("[RECORD] Session complete.")
            break

        if should_reset:
            robot.write_joint_state_to_sim(default_pos, default_vel)
            robot.reset()
            diff_ik_controller.reset()
            teleop.reset()
            maybe_apply_domain_rand(scene, args_cli)
            should_reset = False

        delta_pose, close_gripper = teleop.advance()
        command = torch.tensor(delta_pose, dtype=torch.float32, device=sim.device).unsqueeze(0)

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        arm_joint_pos = robot.data.joint_pos[:, arm_joint_ids]

        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )

        diff_ik_controller.set_command(command, ee_pos=ee_pos_b, ee_quat=ee_quat_b)
        jacobian = robot.root_physx_view.get_jacobians()[
            :, ee_jacobi_idx, :, arm_joint_ids
        ]
        arm_joint_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, arm_joint_pos)
        robot.set_joint_position_target(arm_joint_des, joint_ids=arm_joint_ids)

        gripper_target = gripper_closed if close_gripper else gripper_open
        robot.set_joint_position_target(gripper_target, joint_ids=[gripper_joint_id])

        if record_session is not None:
            record_session.check_timed_episode()
            if record_session.flags.success:
                if record_session.save_episode_if_ready():
                    if not record_session.is_complete:
                        maybe_apply_domain_rand(scene, args_cli)
                        record_session.begin_episode()
            if record_session.should_record_frame():
                state_rad = robot.data.joint_pos[0, all_joint_ids].detach().cpu().numpy()
                action_rad = torch.cat(
                    [arm_joint_des[0], gripper_target[0]], dim=0
                ).detach().cpu().numpy()
                state_raw = sim_rad_to_leader_raw(state_rad)
                action_raw = sim_rad_to_leader_raw(action_rad)
                images = capture_record_images(scene, record_session)
                try:
                    record_session.ingest_snapshot(
                        joints_to_snapshot(state_raw, action_raw, images=images)
                    )
                except ValueError as exc:
                    print(f"[RECORD] Skipped frame: {exc}")

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

    if record_session is not None:
        record_session.finalize()
        record_session.cleanup()
        print(f"[RECORD] Saved under {record_session.output_dir}")


def main():
    sim = build_sim_context(args_cli)
    scene = build_scene(args_cli)
    sim.reset()
    maybe_apply_domain_rand(scene, args_cli)
    print(
        f"[INFO] SO101 keyboard teleop ready "
        f"(scene={args_cli.scene}, robot={args_cli.robot}, cameras={getattr(args_cli, 'enable_cameras', False)}, "
        f"domain_rand={args_cli.domain_rand})."
    )
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
