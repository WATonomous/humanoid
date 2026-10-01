"""Drive the Pioneer left arm in Isaac Sim from the 7-servo leader arm (no IK), in any scene.

    A (servo ID 2) -> joint1L (shoulder flexion)
    B (servo ID 3) -> joint2l (shoulder abduction)
    C (servo ID 1) -> joint3l (shoulder rotation)
    D (servo ID 5) -> joint4l (elbow flexion)
    E (servo ID 4) -> joint5l (forearm rotation)
    F (servo ID 7) -> joint6l (wrist)
    G (servo ID 6) -> gripper (starts open at 41.5 deg, closes toward 0)

The leader is zeroed at startup and on R: hold it in the sim home pose (elbow bent, gripper
open). Leader zero maps to the arm's default pose; targets are clamped to the URDF limits.
Leader torque is always off; the physical leader is an input device only.

  R   re-zero the leader, reset the arm and every object in the scene

Recording (--record, src/robot_learning/config/dataset_schema_pioneer_v1.yaml), 25 fps = every
4th physics step. Keys S start, N save (then auto-reset), D discard:
  observation.state  6 joints (rad) + gripper closure (0 open .. 1 closed, mean of both fingers)
  action             6 joint targets (rad) + leader gripper closure (0..1, continuous)
  observation.images.<name>  cameras enabled in the schema, or --cameras ego,wrist_left / none
"""

import argparse
import math
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

# pioneer_humanoid (canonical arm config) and humanoid_robot_learning (recording). Editable-installed
# in the image; this fallback keeps a bare bind-mounted checkout working.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pioneer_humanoid"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_learning"))

from humanoid_robot_learning.sim_teleop_record import (  # noqa: E402
    add_record_args,
    load_record_schema,
    make_sim_recorder,
)

# Per-servo direction, A..F then G. Flip one with --signs if a joint moves the wrong way.
DEFAULT_SIGNS = "1,-1,-1,1,1,-1,1"
# Leader-to-sim gain per arm servo A..F (sim change = leader change x scale), times --scale.
JOINT_SCALES = (0.7, 0.7, 0.7, 1.0, 1.0, 1.0)
# Stock wrist damping (18) caps joint6l near 2 deg/s at the GL40's 0.73 Nm, so the sim wrist
# lags the leader. Lowered here only; BIMANUAL_ARM_CFG is shared with RL and other teleop.
WRIST_DAMPING = 2.5

parser = argparse.ArgumentParser(description="7-servo leader teleoperation of the Pioneer left arm in Isaac Sim.")
parser.add_argument(
    "--scene",
    type=str,
    default="bare",
    help="scene registered in humanoid_scenes (validated after launch; pass an unknown name to list them)",
)
parser.add_argument("--port", default="/dev/ttyACM0", help="leader serial port")
parser.add_argument("--baud", type=int, default=1_000_000, help="leader serial baud rate")
parser.add_argument("--signs", default=DEFAULT_SIGNS, help=f"direction per servo A..G (default: {DEFAULT_SIGNS})")
parser.add_argument("--scale", type=float, default=1.0, help="overall gain, multiplied into JOINT_SCALES")
parser.add_argument(
    "--filter-alpha",
    type=float,
    default=0.35,
    help="target low-pass coefficient in (0,1]; 1 disables filtering",
)
add_record_args(parser, task_description="sim leader teleop demonstration")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if not 0.0 < args_cli.filter_alpha <= 1.0:
    parser.error("--filter-alpha must be in (0, 1]")
if not (math.isfinite(args_cli.scale) and args_cli.scale > 0.0):
    parser.error("--scale must be finite and > 0")
_record = load_record_schema(parser, args_cli)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb
import omni.appwindow
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene

from pioneer_humanoid.bimanual_arm import (
    BIMANUAL_ARM_CFG,
    LEFT_ARM_JOINTS,
    LEFT_GRIPPER_CLOSED,
    LEFT_GRIPPER_JOINTS,
    LEFT_GRIPPER_OPEN,
    RIGHT_ARM_JOINTS,
    RIGHT_GRIPPER_JOINTS,
    RIGHT_GRIPPER_OPEN,
    apply_joint_limits,
    resolve_joint_name,
)
from humanoid_scenes import list_scenes, make_scene_cfg, scene_camera
from pioneer_humanoid.cameras import CAMERA_NAMES, make_camera_cfg

from arm_limits import gripper_fraction, limited_target
from servo_leader import ARM_SERVOS, GRIPPER_SERVO, SERVO_IDS, ServoLeader, parse_signs


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    recorder, record_every = make_sim_recorder(args_cli, _record, device=sim.device, sim_dt=sim_dt)
    if recorder is not None:
        print("[RECORD] Keys: S=start, N=save episode (then reset), D=discard")
        recorder.start_keyboard()

    # Populate robot buffers, then write the URDF limits the targets are clamped to.
    scene.update(sim_dt)
    apply_joint_limits(robot)

    arm_ids = _joint_ids(robot, LEFT_ARM_JOINTS)
    gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    held_arm_ids = _joint_ids(robot, RIGHT_ARM_JOINTS)
    held_gripper_ids = _joint_ids(robot, RIGHT_GRIPPER_JOINTS)
    if len(arm_ids) != len(ARM_SERVOS):
        raise ValueError(f"leader drives {len(ARM_SERVOS)} joints but the arm has {len(arm_ids)}")

    default_pos = robot.data.default_joint_pos.clone()
    default_vel = robot.data.default_joint_vel.clone()
    # Leader zero maps to the arm's default (home) pose: elbow bent, mirroring the held arm.
    arm_home = default_pos[0, arm_ids].tolist()
    held_arm_default = default_pos[:, held_arm_ids].clone()
    arm_limits_deg = [
        (math.degrees(lo), math.degrees(hi)) for lo, hi in robot.data.joint_pos_limits[0, arm_ids].tolist()
    ]

    gripper_open = torch.tensor([[LEFT_GRIPPER_OPEN[j] for j in LEFT_GRIPPER_JOINTS]], device=sim.device)
    gripper_closed = torch.tensor([[LEFT_GRIPPER_CLOSED[j] for j in LEFT_GRIPPER_JOINTS]], device=sim.device)
    held_gripper_open = torch.tensor([[RIGHT_GRIPPER_OPEN[j] for j in RIGHT_GRIPPER_JOINTS]], device=sim.device)
    zero_gripper_vel = torch.zeros(1, len(gripper_ids), device=sim.device)

    signs = parse_signs(args_cli.signs)
    scales = [args_cli.scale * s for s in JOINT_SCALES]
    gripper_axis = tuple(SERVO_IDS).index(GRIPPER_SERVO)
    print(
        "[LEADER] Mapping: "
        + " | ".join(
            f"{label}(ID{SERVO_IDS[label]}) {sign:+.0f}x{scale:g} -> {joint}"
            for label, joint, sign, scale in zip(ARM_SERVOS, LEFT_ARM_JOINTS, signs, scales)
        )
        + f" | {GRIPPER_SERVO}(ID{SERVO_IDS[GRIPPER_SERVO]}) {signs[gripper_axis]:+.0f} -> gripper",
        flush=True,
    )

    # Called by tick() only on recorded frames.
    def read_images():
        return {name: scene[f"record_cam_{name}"].data.output["rgb"][0, ..., :3] for name in _record.images}

    reset_requested = {"v": False}

    def _on_kb(event, *_):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == carb.input.KeyboardInput.R:
            reset_requested["v"] = True
        return True

    leader = ServoLeader(args_cli.port, args_cli.baud)
    state = {"angles": (0.0,) * len(SERVO_IDS), "target": None, "grip": None}
    # Held in state: the subscription stops when it is garbage-collected.
    state["kb_sub"] = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), _on_kb
    )

    def reset_all():
        """Arm and every rigid object back to their defaults; the leader re-zeros to match."""
        robot.write_joint_state_to_sim(default_pos, default_vel)
        robot.reset()
        for obj in scene.rigid_objects.values():
            root = obj.data.default_root_state.clone()
            root[:, :3] += scene.env_origins
            obj.write_root_pose_to_sim(root[:, :7])
            obj.write_root_velocity_to_sim(root[:, 7:])
            obj.reset()
        # Re-zero with the arm: otherwise the next read commands a jump equal to how far the
        # leader has moved since its last zero.
        leader.rezero()
        state.update(angles=(0.0,) * len(SERVO_IDS), target=None, grip=None)

    print("[INFO] Hold the leader in the home pose (elbow bent, gripper open). R = re-zero + reset.", flush=True)

    physics_step = 0
    last_warning = 0.0
    last_report = time.monotonic()
    next_step_t = time.monotonic()
    try:
        while simulation_app.is_running():
            if recorder is not None and recorder.is_complete:
                print("[RECORD] Session complete.")
                break
            if reset_requested["v"]:
                reset_requested["v"] = False
                if recorder is not None and recorder.num_buffered_frames > 0:
                    # Frames either side of a reset are not one demo: drop the take, keep recording.
                    recorder.cancel_recording()
                    print("[RECORD] Reset mid-episode: take discarded, recording restarts from home.")
                reset_all()
                print("[LEADER] Re-zeroed; arm and scene reset.", flush=True)

            try:
                state["angles"] = leader.read_radians()
            except RuntimeError as exc:
                # A dropped serial frame holds the last target rather than stopping physics.
                now = time.monotonic()
                if now - last_warning >= 1.0:
                    print(f"[WARN] {exc}; holding last leader target", flush=True)
                    last_warning = now
            angles = state["angles"]

            desired = torch.tensor(
                [[
                    limited_target(angles[i], signs[i], scales[i], arm_limits_deg[i], offset_rad=arm_home[i])
                    for i in range(len(ARM_SERVOS))
                ]],
                device=sim.device,
            )
            if state["target"] is None:
                state["target"] = desired.clone()
            state["target"].lerp_(desired, args_cli.filter_alpha)

            grip = gripper_fraction(angles[gripper_axis], signs[gripper_axis])
            if state["grip"] is None:
                state["grip"] = grip
            state["grip"] += args_cli.filter_alpha * (grip - state["grip"])

            robot.set_joint_position_target(state["target"], joint_ids=arm_ids)
            # Coupled fingers: high stiffness + zero velocity target stops bounce on the move.
            robot.set_joint_position_target(
                torch.lerp(gripper_open, gripper_closed, state["grip"]), joint_ids=gripper_ids
            )
            robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=gripper_ids)
            # Hold the other arm at its default pose, its gripper open.
            robot.set_joint_position_target(held_arm_default, joint_ids=held_arm_ids)
            robot.set_joint_position_target(held_gripper_open, joint_ids=held_gripper_ids)
            robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=held_gripper_ids)

            if recorder is not None and physics_step % record_every == 0:
                finger_q = robot.data.joint_pos[:, gripper_ids]
                closure = (
                    ((finger_q - gripper_open) / (gripper_closed - gripper_open))
                    .mean(dim=-1)
                    .clamp(0.0, 1.0)
                )
                obs = torch.cat([robot.data.joint_pos[0, arm_ids], closure])
                act = torch.cat([state["target"][0], torch.tensor([state["grip"]], device=sim.device)])
                saved = recorder.tick(
                    act.detach().cpu().numpy().astype("float32"),
                    obs.detach().cpu().numpy().astype("float32"),
                    read_images,
                )
                if saved:
                    reset_all()
                    print("[RECORD] Episode saved; arm and scene reset.", flush=True)

            scene.write_data_to_sim()
            sim.step()
            physics_step += 1
            scene.update(sim_dt)

            now = time.monotonic()
            if now - last_report >= 0.5:
                print(
                    "\r[LEADER] "
                    + " ".join(f"{label}={math.degrees(a):+6.1f}" for label, a in zip(SERVO_IDS, angles))
                    + f" grip={state['grip']:.2f}",
                    end="",
                    flush=True,
                )
                last_report = now

            # Keep sim time at or behind wall time, so recorded motion has real-world timing.
            next_step_t += sim_dt
            delay = next_step_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_step_t = time.monotonic()
    finally:
        leader.close()
        print("\n[INFO] Stopped. Leader torque is OFF.", flush=True)
        # Also on errors: flushes episodes still being written in the background.
        if recorder is not None:
            recorder.finalize()
            print(f"[RECORD] Saved under {recorder.dataset_root}")


def main():
    if args_cli.scene not in list_scenes():
        raise SystemExit(f"unknown --scene {args_cli.scene!r}; available: {list_scenes()}")

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, device=args_cli.device))
    sim.set_camera_view(*(scene_camera(args_cli.scene) or ([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])))

    robot_cfg = BIMANUAL_ARM_CFG.replace(
        actuators={
            **BIMANUAL_ARM_CFG.actuators,
            "left_wrist": BIMANUAL_ARM_CFG.actuators["left_wrist"].replace(damping=WRIST_DAMPING),
        }
    )
    scene_cfg = make_scene_cfg(args_cli.scene, robot_cfg, num_envs=1, env_spacing=2.0)
    # Added after the robot: cameras are parented under it, and entities are created in order.
    unknown = sorted(set(_record.images) - set(CAMERA_NAMES))
    if unknown:
        raise SystemExit(f"{_record.path}: unknown images {unknown}; available: {list(CAMERA_NAMES)}")
    for name, spec in _record.images.items():
        setattr(scene_cfg, f"record_cam_{name}", make_camera_cfg(name, int(spec["height"]), int(spec["width"])))
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete. Move the leader arm to drive the left arm.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
