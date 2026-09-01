"""Minimal bimanual-arm keyboard teleoperation (left arm only).

Robot: pioneer_bimanual_arm (see pioneer_humanoid.bimanual_arm)
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

_IL_PKG = Path(__file__).resolve().parents[2] / "il"
_DEFAULT_SIM_SCHEMA = _IL_PKG / "config" / "dataset_schema_sim.yaml"

# pioneer_humanoid package (canonical arm config). Editable-installed in the image; this fallback
# keeps a bare bind-mounted checkout working.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pioneer_humanoid"))

parser = argparse.ArgumentParser(description="Keyboard teleoperation for the Pioneer bimanual arm (left only).")
parser.add_argument(
    "--record",
    action="store_true",
    help="Record demonstrations (requires: pip install -e src/il[record])",
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
    help="dataset_schema YAML (default: src/il/config/dataset_schema_sim.yaml)",
)
parser.add_argument(
    "--dataset_root",
    type=str,
    default=None,
    help="Override record.root from schema (e.g. datasets/record_sim)",
)
parser.add_argument("--num_episodes", type=int, default=10)
parser.add_argument("--task_description", type=str, default="sim keyboard teleop demonstration")
parser.add_argument(
    "--scene",
    type=str,
    default="bare",
    help="scene name: 'bare' (arm only), 'push', or any scene registered in "
    "humanoid_scenes (validated after launch — pass an unknown name to list them)",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb
import omni.appwindow
import torch

import isaaclab.sim as sim_utils
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene
from isaaclab.utils.math import compute_pose_error, quat_from_angle_axis, quat_mul, subtract_frame_transforms

# This script actuates the L-suffixed chain (physical LEFT arm) and holds the unsuffixed
# one. Canonical names it LEFT_*; the aliases below keep this file's RIGHT_*/GRIPPER_* local
# names (RIGHT_* = the actuated arm) so the body is unchanged.
from pioneer_humanoid.bimanual_arm import (
    LEFT_GRIPPER_CLOSED as GRIPPER_CLOSED,
    LEFT_GRIPPER_OPEN as GRIPPER_OPEN,
    LEFT_ARM_JOINTS as RIGHT_ARM_JOINTS,
    LEFT_EE_BODY as RIGHT_EE_BODY,
    LEFT_GRIPPER_JOINTS as RIGHT_GRIPPER_JOINTS,
    LEFT_FINGER_TIP_BODIES as RIGHT_FINGER_TIP_BODIES,
    RIGHT_ARM_JOINTS as LEFT_ARM_JOINTS,
    apply_joint_limits,
    resolve_joint_name,
    resolve_body_ids,
    compute_gripper_tip_pose_b,
    compute_tip_ik_jacobian,
)
from pioneer_humanoid.teleop_scenes import build_scene_cfg, camera_for, scene_names


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


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
            "  pip install -e src/il[sim]"
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

    # Absolute-pose IK against a persistent target (like task_space_ik.py), NOT
    # relative mode. Relative mode re-anchors to the measured tip every step, so a
    # held key makes the arm chase a moving carrot -> laggy/mushy near limits. Here
    # the target is integrated from the keyboard deltas and leashed to stay within
    # _MAX_LEAD_* of the actual tip, so a held key = arm moves at the speed it can
    # sustain, no runaway, and releasing stops it immediately.
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)
    _MAX_LEAD_M = 0.06
    _MAX_LEAD_RAD = 0.35
    target = {"pos": None, "quat": None}  # persistent EE target in base frame

    robot_entity_cfg = SceneEntityCfg("robot", joint_names=right_arm_names, body_names=[RIGHT_EE_BODY])
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    wrist_body_id = robot_entity_cfg.body_ids[0]
    finger_body_ids = resolve_body_ids(robot, RIGHT_FINGER_TIP_BODIES)

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

    # Fast by default; hold Shift for fine control. Se3Keyboard has no built-in
    # modifier, so the base sensitivity is set high and Shift scales the command
    # down via a carb keyboard subscription (add_callback is press-only, can't
    # detect release).
    teleop = Se3Keyboard(Se3KeyboardCfg(pos_sensitivity=0.011, rot_sensitivity=0.09, gripper_term=True))
    _FINE_SCALE = 0.2
    fine = {"active": False}
    _shift_keys = {carb.input.KeyboardInput.LEFT_SHIFT, carb.input.KeyboardInput.RIGHT_SHIFT}

    def _on_kb_event(event, *args):
        if event.input in _shift_keys:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                fine["active"] = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                fine["active"] = False
        return True

    _appwin = omni.appwindow.get_default_app_window()
    _kb_sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        _appwin.get_keyboard(), _on_kb_event
    )
    should_reset = False

    def reset_left_arm():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_left_arm)
    teleop.reset()
    print(teleop)
    print("[INFO] Teleoperating left arm only. Right arm is held at default pose.")
    print("[INFO] Click the 3D viewport window, then W/A/S/D/Q/E to move. Hold SHIFT for fine control.")

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
            target["pos"] = None  # re-seed the persistent target from the tip
            should_reset = False

        # Se3Keyboard.advance() returns a 7-vec tensor: [dx,dy,dz,drx,dry,drz, gripper(+1 open/-1 close)].
        cmd = teleop.advance()
        command = cmd[:6].to(dtype=torch.float32, device=sim.device).unsqueeze(0)
        if fine["active"]:
            command = command * _FINE_SCALE
        close_gripper = bool(cmd[6].item() < 0.0) if cmd.numel() > 6 else False

        if debug_steps < 5 and torch.any(command.abs() > 1e-4):
            print(f"[DEBUG] Keyboard command: {command[0].tolist()}")
            debug_steps += 1

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, left_arm_ids]

        ee_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, wrist_body_id, finger_body_ids
        )

        # Persistent absolute target: seed from the tip once, then integrate the
        # keyboard deltas. Position deltas are base-frame; rotation deltas are
        # post-multiplied so Z/X/T/G/C/V spin about the *tool* axes, not base axes.
        if target["pos"] is None:
            target["pos"] = tip_pos_b.clone()
            target["quat"] = tip_quat_b.clone()

        target["pos"] = target["pos"] + command[:, 0:3]
        rot_vec = command[:, 3:6]
        angle = torch.linalg.vector_norm(rot_vec, dim=-1)
        if float(angle) > 1e-9:
            axis = rot_vec / angle.unsqueeze(-1)
            target["quat"] = quat_mul(target["quat"], quat_from_angle_axis(angle, axis))

        # Leash the target to stay near the actual tip: a held key then moves the
        # arm at whatever speed it can sustain and stops the instant the key lifts.
        pos_err, rot_err = compute_pose_error(
            tip_pos_b, tip_quat_b, target["pos"], target["quat"], rot_error_type="axis_angle"
        )
        target["pos"] = tip_pos_b + pos_err.clamp(-_MAX_LEAD_M, _MAX_LEAD_M)
        rot_mag = torch.linalg.vector_norm(rot_err, dim=-1)
        if float(rot_mag) > _MAX_LEAD_RAD:
            clamped = rot_err * (_MAX_LEAD_RAD / rot_mag).unsqueeze(-1)
            c_ang = torch.linalg.vector_norm(clamped, dim=-1)
            c_axis = clamped / c_ang.unsqueeze(-1)
            target["quat"] = quat_mul(quat_from_angle_axis(c_ang, c_axis), tip_quat_b)

        diff_ik_controller.set_command(
            torch.cat([target["pos"], target["quat"]], dim=-1), ee_quat=tip_quat_b
        )

        jacobian = compute_tip_ik_jacobian(
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
    if args_cli.scene not in scene_names():
        raise SystemExit(f"unknown --scene {args_cli.scene!r}; available: {scene_names()}")

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(*camera_for(args_cli.scene))

    scene_cfg = build_scene_cfg(args_cli.scene, num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete. Use keyboard to teleoperate the left arm.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
