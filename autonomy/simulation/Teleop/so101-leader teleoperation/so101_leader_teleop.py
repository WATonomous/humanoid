"""SO101 Leader (USB) teleoperation of SO101 follower in Isaac Sim.

Mirrors the NVIDIA Sim-to-Real SO101 workshop flow: physical leader arm drives
the sim follower, with optional recording through ``humanoid_il``.

  R       Reset sim robot to default pose
  (record keys via pynput when --record: S / N / D / Esc)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

_IL_PKG = Path(__file__).resolve().parents[3] / "il"
_DEFAULT_SCHEMA = _IL_PKG / "config" / "dataset_schema_so101_sim.yaml"

parser = argparse.ArgumentParser(
    description="SO101 Leader teleoperation for SO101 follower in Isaac Sim."
)
parser.add_argument(
    "--port",
    type=str,
    default="/dev/ttyACM0",
    help="USB serial port for the SO101 Leader arm",
)
parser.add_argument(
    "--robot_id",
    type=str,
    default="leader_arm_1",
    help="Leader arm ID (LeRobot calibration namespace)",
)
parser.add_argument(
    "--robot",
    type=str,
    choices=("follower", "arm_camera"),
    default="follower",
    help="Sim robot USD: follower (LeRobot) or arm_camera (workshop mesh + visible gripper cam)",
)
parser.add_argument(
    "--recalibrate",
    action="store_true",
    help="Force SO101 Leader calibration before teleop",
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
    default="so101 leader sim demonstration",
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
from isaaclab.scene import InteractiveScene

if str(_IL_PKG) not in sys.path:
    sys.path.insert(0, str(_IL_PKG))

ensure_il_on_path()

from humanoid_il.so101_sim import leader_action_to_array, leader_raw_to_sim_rad, sim_rad_to_leader_raw

from so101_cfg import robot_joint_names
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


def _connect_leader(port: str, robot_id: str, recalibrate: bool):
    try:
        from lerobot.robots import make_robot_from_config
        from lerobot.teleoperators.so101_leader import SO101LeaderConfig
    except ImportError as exc:
        raise ImportError(
            "SO101 Leader teleop requires LeRobot with SO101 support.\n"
            "  pip install -e autonomy/il[record]"
        ) from exc

    cfg = SO101LeaderConfig(port=port, id=robot_id)
    leader = make_robot_from_config(cfg)
    if recalibrate and hasattr(leader, "calibrate"):
        leader.calibrate()
    leader.connect()
    print(f"[INFO] SO101 Leader connected on {port} (id={robot_id})")
    return leader


def _init_recorder(device: str):
    if not args_cli.record:
        return None, None
    ensure_il_on_path()
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
        else Path((cfg.get("record") or {}).get("root", "datasets/record_so101_sim"))
    )
    cameras = {
        name: {"height": spec["height"], "width": spec["width"]}
        for name, spec in enabled_images(cfg).items()
    }
    recorder = SimLeRobotRecorder(
        task_name=args_cli.task_description,
        repo_id=str(cfg.get("repo_id", "humanoid/so101_sim")),
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
    joint_ids = _joint_ids(robot, robot_joint_names(args_cli.robot))

    scene.update(sim_dt)
    default_pos = robot.data.default_joint_pos.clone()
    default_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(default_pos, default_vel)

    leader = _connect_leader(args_cli.port, args_cli.robot_id, args_cli.recalibrate)
    recorder, record_cfg = _init_recorder(sim.device)

    if recorder is not None:
        recorder.start_keyboard()

    should_reset = False
    last_leader_raw = None

    print("[INFO] Move the SO101 Leader to drive the sim follower.")
    print("[INFO] Press R in the Isaac viewport to reset the sim arm.")

    from isaaclab.devices import Se3Keyboard

    teleop = Se3Keyboard(pos_sensitivity=0.0, rot_sensitivity=0.0)

    def reset_robot():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_robot)

    while simulation_app.is_running():
        if recorder is not None and recorder.is_complete:
            print("[RECORD] Session complete.")
            break

        if should_reset:
            robot.write_joint_state_to_sim(default_pos, default_vel)
            robot.reset()
            maybe_apply_domain_rand(scene, args_cli)
            should_reset = False
            print("[INFO] Sim arm reset to default pose.")

        try:
            leader_action = leader.get_action()
            leader_raw = leader_action_to_array(leader_action)
            last_leader_raw = leader_raw
        except Exception as exc:
            print(f"[WARN] Leader read failed: {exc}")
            if last_leader_raw is None:
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim_dt)
                continue
            leader_raw = last_leader_raw

        target_rad = leader_raw_to_sim_rad(leader_raw)
        target = torch.tensor(target_rad, dtype=torch.float32, device=sim.device).unsqueeze(0)
        robot.set_joint_position_target(target, joint_ids=joint_ids)

        if recorder is not None:
            measured = robot.data.joint_pos[0, joint_ids].detach().cpu().numpy()
            state_raw = sim_rad_to_leader_raw(measured)
            images = capture_record_images(scene, record_cfg) or {}
            if recorder.tick(leader_raw.copy(), state_raw, images) and not recorder.is_complete:
                maybe_apply_domain_rand(scene, args_cli)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

    if recorder is not None:
        recorder.finalize()
        print(f"[RECORD] Saved under {recorder.dataset_root}")

    try:
        leader.disconnect()
    except Exception:
        pass


def main():
    sim = build_sim_context(args_cli)
    scene = build_scene(args_cli)
    sim.reset()
    maybe_apply_domain_rand(scene, args_cli)
    print(
        f"[INFO] SO101 Leader teleop ready "
        f"(scene={args_cli.scene}, robot={args_cli.robot}, cameras={getattr(args_cli, 'enable_cameras', False)}, "
        f"domain_rand={args_cli.domain_rand})."
    )
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
