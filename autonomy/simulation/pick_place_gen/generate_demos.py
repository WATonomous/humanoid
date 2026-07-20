"""Generate pick-and-place demonstrations with the cuRobo expert.

Episode loop: reset env -> sample place target -> cuRobo-planned pick/place
via the Orchestrator -> ground-truth success gate -> (optionally) save the
episode to a LeRobot dataset through humanoid_il's SimLeRobotRecorder.

Run from the repo inside the simulation_il container:
  cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
  PYTHONPATH=$(pwd) /workspace/isaaclab/isaaclab.sh -p \
      ../../pick_place_gen/generate_demos.py --headless --enable_cameras \
      --num_episodes 10 --seed 0 [--record --schema <yaml>] [--task_params <yaml>]
"""
import argparse
import math
import os
import sys
import time
from collections import Counter
from pathlib import Path

# Pin site-packages warp (1.11+) before Kit loads its older omni.warp bundle —
# cuRobo's collision kernels need wp.func(module=...). Same trick as the
# pinocchio-before-AppLauncher import in run_quest_bimanual_teleop.py.
import warp  # noqa: F401

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_episodes", type=int, default=10)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--task_params", type=str, default=None, help="YAML overriding PickPlaceTaskParams")
parser.add_argument("--record", action="store_true")
parser.add_argument("--schema", type=str, default=None, help="dataset schema YAML (required with --record)")
parser.add_argument("--dataset_root", type=str, default=None)
parser.add_argument("--task_description", type=str, default="pick and place the cube")
parser.add_argument("--save_debug_images", type=str, default=None,
                    help="directory to dump camera PNGs at phase transitions (episode 0)")
parser.add_argument("--debug_memory", action="store_true",
                    help="per-episode census of large live buffers (leak triage)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

_GEN_DIR = Path(__file__).resolve().parent
_HRL_DIR = _GEN_DIR.parents[1] / "Humanoid_Wato" / "HumanoidRL"
for _p in (str(_GEN_DIR), str(_HRL_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: E402,F401
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual import robot_cfg_shim as shim  # noqa: E402
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual.pick_place_env_cfg import (  # noqa: E402
    make_env_cfg,
)

import wato_constants as wc  # noqa: E402
from curobo_expert import CuRoboExpert  # noqa: E402
from orchestrator import Orchestrator  # noqa: E402
from task_params import PickPlaceTaskParams  # noqa: E402


def quat_yaw(quat_wxyz) -> float:
    w, x, y, z = [float(v) for v in quat_wxyz]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class GroundTruth:
    """Env-local (== robot base frame) ground-truth accessors."""

    def __init__(self, env, params):
        self.env = env.unwrapped
        self.scene = self.env.scene
        self.params = params
        self.robot = self.scene["robot"]
        self.origin = self.scene.env_origins[0]
        arm6 = shim.RIGHT_ARM_JOINTS
        names = list(self.robot.data.joint_names)
        self.arm_ids = [names.index(j) for j in arm6]
        self.grip_ids = [names.index(j) for j in shim.RIGHT_GRIPPER_JOINTS]
        self.wrist_id, = shim.resolve_body_ids(self.robot, [shim.RIGHT_EE_BODY])
        self.finger_ids = shim.resolve_body_ids(self.robot, list(shim.RIGHT_FINGER_TIP_BODIES))

    def q_arm(self):
        return self.robot.data.joint_pos[0, self.arm_ids].cpu().numpy()

    def q_grip(self):
        return self.robot.data.joint_pos[0, self.grip_ids].cpu().numpy()

    def tip_pose(self):
        root = torch.cat([self.robot.data.root_pos_w, self.robot.data.root_quat_w], dim=-1)
        pos, quat = shim.compute_gripper_tip_pose_b(self.robot, root, self.wrist_id, self.finger_ids)
        return pos[0].cpu().numpy(), quat[0].cpu().numpy()

    def object_pose(self, name="object"):
        obj = self.scene[name]
        pos = (obj.data.root_pos_w[0] - self.origin).cpu().numpy()
        quat = obj.data.root_quat_w[0].cpu().numpy()
        return pos, quat

    def object_lin_vel(self, name="object"):
        return self.scene[name].data.root_lin_vel_w[0].cpu().numpy()

    def state_dict(self, place_pos, support_z):
        p = self.params
        obj_pos, obj_quat = self.object_pose()
        state = {
            "q_arm": self.q_arm(),
            "tip_pos": self.tip_pose()[0],
            "object_pos": obj_pos,
            "object_quat": obj_quat,
            "object_yaw": quat_yaw(obj_quat),
            "object_size": np.array(p.object.size),
            "place_pos": place_pos,
            "support_top_z": support_z,
            "place_object_pos": None,
        }
        if p.place.mode == "stack":
            pl_pos, pl_quat = self.object_pose("place_object")
            state.update({
                "place_object_pos": pl_pos,
                "place_object_quat": pl_quat,
                "place_object_size": np.array(p.place.stack_object_size),
            })
        elif p.place.mode == "tray":
            state["extra_obstacles"] = _tray_wall_obstacles(p.place)
        return state


def _tray_wall_obstacles(place) -> list:
    """4 thin wall cuboids forming the tray rim (interior left open), so the
    planner keeps the arm clear of the walls while transiting over the tray.
    Returns (name, pos(3), quat_wxyz(4), dims(3)) tuples in the env frame."""
    s = place.tray_scale
    cx, cy = place.tray_center
    hx = wc.TRAY_FOOTPRINT[0] / 2 * s
    hy = wc.TRAY_FOOTPRINT[1] / 2 * s
    t = max(wc.TRAY_WALL_THICK * s, 0.006)  # floor for planner reliability
    h = wc.TRAY_WALL_HEIGHT * s * place.tray_height_scale
    zc = wc.TABLE_TOP_Z + h / 2
    q = [1.0, 0.0, 0.0, 0.0]
    return [
        ("tray_wall_xp", [cx + hx, cy, zc], q, [t, 2 * hy, h]),
        ("tray_wall_xn", [cx - hx, cy, zc], q, [t, 2 * hy, h]),
        ("tray_wall_yp", [cx, cy + hy, zc], q, [2 * hx, t, h]),
        ("tray_wall_yn", [cx, cy - hy, zc], q, [2 * hx, t, h]),
    ]


def sample_place_target(gt: GroundTruth, params, rng) -> tuple:
    """(place_pos xy+z_center, support_top_z) honoring min separation."""
    obj_pos, _ = gt.object_pose()
    if params.place.mode == "stack":
        pl_pos, _ = gt.object_pose("place_object")
        support_z = pl_pos[2] + params.place.stack_object_size[2] / 2
        target = np.array([pl_pos[0], pl_pos[1], support_z + params.object.size[2] / 2])
        return target, support_z
    if params.place.mode == "tray":
        # Fixed target: the centre of the tray interior. support_z = tray floor
        # (scales with the wall-height scale, not the footprint scale).
        sz = params.place.tray_scale * params.place.tray_height_scale
        support_z = wc.TABLE_TOP_Z + sz * wc.TRAY_FLOOR_LOCAL_Z
        cx, cy = params.place.tray_center
        return np.array([cx, cy, support_z + params.object.size[2] / 2]), support_z
    for _ in range(200):
        xy = rng.uniform(
            [params.place.x_range[0], params.place.y_range[0]],
            [params.place.x_range[1], params.place.y_range[1]],
        )
        if np.linalg.norm(xy - obj_pos[:2]) >= params.place.min_separation:
            support_z = wc.TABLE_TOP_Z
            return np.array([xy[0], xy[1], support_z + params.object.size[2] / 2]), support_z
    raise RuntimeError("could not sample place target with min separation")


def check_success(gt: GroundTruth, params, place_pos, support_z) -> bool:
    obj_pos, _ = gt.object_pose()
    xy_err = float(np.linalg.norm(obj_pos[:2] - place_pos[:2]))
    z_err = abs(float(obj_pos[2]) - (support_z + params.object.size[2] / 2))
    vel = float(np.linalg.norm(gt.object_lin_vel()))
    return (xy_err < params.place.xy_tolerance and z_err < params.place.z_tolerance
            and vel < params.success.max_lin_vel)


def save_debug_image(env, out_dir: Path, tag: str):
    try:
        import imageio.v2 as imageio
    except ImportError:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in ("camera_external", "camera_wrist"):
        if name in env.unwrapped.scene.sensors:
            rgb = env.unwrapped.scene.sensors[name].data.output["rgb"][0].cpu().numpy()
            imageio.imwrite(out_dir / f"{tag}_{name}.png", rgb)


def memory_snapshot(tag: str, deep: bool = False) -> None:
    """One-line process/host/GPU memory report, printed per episode.

    Kept permanently: long camera-recording sessions have crashed a 30 GB
    host before, and a visible RSS trend is the earliest warning. `deep`
    additionally reports large live numpy/torch-CPU buffers with their
    referrer types (leak triage; enable with --debug_memory).
    """
    import psutil

    proc = psutil.Process()
    rss_gb = proc.memory_info().rss / 1e9
    avail_gb = psutil.virtual_memory().available / 1e9
    cuda_gb = torch.cuda.memory_allocated() / 1e9
    cuda_res_gb = torch.cuda.memory_reserved() / 1e9
    print(f"[mem {tag}] rss={rss_gb:.2f}G avail={avail_gb:.2f}G "
          f"cuda={cuda_gb:.2f}G reserved={cuda_res_gb:.2f}G")
    if not deep:
        return

    import gc

    gc.collect()
    big = []
    for obj in gc.get_objects():
        try:
            if isinstance(obj, np.ndarray) and obj.nbytes > 50e6:
                big.append(("np", obj))
            elif isinstance(obj, torch.Tensor) and not obj.is_cuda and obj.element_size() * obj.nelement() > 50e6:
                big.append(("pt", obj))
        except ReferenceError:
            continue
    total = sum(o.nbytes if k == "np" else o.element_size() * o.nelement() for k, o in big)
    print(f"[mem {tag}] big buffers: {len(big)} ({total / 1e9:.2f}G)")
    for kind, obj in big[:6]:
        refs = [type(r).__name__ for r in gc.get_referrers(obj)][:5]
        print(f"    {kind} {tuple(obj.shape)} referrers={refs}")


def build_recorder(params, gt, args):
    """SimLeRobotRecorder wired for direct-buffer use (no wall-clock tick)."""
    _IL_DIR = _GEN_DIR.parents[2] / "il"
    sys.path.insert(0, str(_IL_DIR))
    from humanoid_il.schema import enabled_images, load_yaml
    from humanoid_il.sim_recorder import SimLeRobotRecorder

    cfg = load_yaml(Path(args.schema))
    dataset_root = args.dataset_root or cfg.get("record", {}).get("root", "datasets/record_pick_place")
    cameras = {k: {"height": v["height"], "width": v["width"]} for k, v in enabled_images(cfg).items()}
    recorder = SimLeRobotRecorder(
        task_name=args.task_description,
        repo_id=cfg.get("repo_id", "humanoid/pick_place_bimanual"),
        dataset_root=dataset_root,
        fps=int(cfg.get("fps", params.episode.record_fps)),
        device=gt.env.device,
        joint_names=cfg["joint_names"],
        cameras=cameras,
        robot_type=cfg.get("robot_id", "wato_bimanual_left"),
        extra_features={"observation.environment_state": cfg["env_state_names"]},
        # 45 s covers the longest episodes; sizes both the GPU ring buffers
        # and the recorder's constant pinned CPU slot pool (~92 MB/s/camera)
        buffer_capacity_s=45.0,
    )
    recorder.init_dataset()
    return recorder, dataset_root


def _fix_output_ownership(path) -> None:
    """The datagen container runs as root, so recorded files land root-owned and
    the host user cannot delete/manage them without sudo. Chown the finished
    dataset to match the bind-mounted repo root's owner (= the host user), so it
    behaves like a normal user file. No-op unless running as root on a real path.
    """
    if not path or not os.path.exists(path) or getattr(os, "geteuid", lambda: 1000)() != 0:
        return
    try:
        ref = os.stat(wc.REPO_ROOT)  # bind-mounted -> owned by the host user
    except OSError:
        return
    try:
        for root, _dirs, files in os.walk(path):
            os.chown(root, ref.st_uid, ref.st_gid)
            for name in files:
                os.chown(os.path.join(root, name), ref.st_uid, ref.st_gid)
        print(f"[ownership] {path} -> uid {ref.st_uid}:{ref.st_gid} (host user; no sudo needed)")
    except OSError as e:
        print(f"[ownership] could not chown {path}: {e}")


def main():
    params = PickPlaceTaskParams.from_yaml(args_cli.task_params)
    if not args_cli.enable_cameras:
        params.cameras.enabled = False
    shim.check_constants_consistency()

    env = gym.make("Isaac-PickPlace-BimanualLeft-v0", cfg=make_env_cfg(params))
    env.reset(seed=args_cli.seed)
    rng = np.random.default_rng(args_cli.seed)
    gt = GroundTruth(env, params)

    action_term = env.unwrapped.action_manager.get_term("arm_action")
    assert action_term._joint_names == shim.RIGHT_JOINTS_ALL, action_term._joint_names

    expert = CuRoboExpert(params)
    assert expert.joint_names == shim.RIGHT_ARM_JOINTS, expert.joint_names
    orch = Orchestrator(expert, params, rng)

    recorder, dataset_root = (
        build_recorder(params, gt, args_cli) if args_cli.record else (None, None)
    )
    debug_dir = Path(args_cli.save_debug_images) if args_cli.save_debug_images else None

    control_dt = params.episode.sim_dt * params.episode.decimation
    settle_needed = params.success.settle_steps
    attempted = succeeded = saved = 0
    failures = Counter()
    t_start = time.time()

    for ep in range(args_cli.num_episodes):
        memory_snapshot(f"ep{ep}", deep=args_cli.debug_memory)
        env.reset()
        # let objects settle on the table before reading ground truth
        hold0 = np.concatenate([gt.q_arm(), gt.q_grip()]).astype(np.float32)
        for _ in range(10):
            env.step(torch.tensor(hold0, device=gt.env.device).unsqueeze(0))

        place_pos, support_z = sample_place_target(gt, params, rng)
        state = gt.state_dict(place_pos, support_z)
        attempted += 1
        ep_frames = 0
        record_this = recorder is not None

        if not orch.start_episode(state):
            failures[orch.failure_reason] += 1
            print(f"[ep {ep}] FAIL {orch.failure_reason}")
            continue

        last_phase = orch.phase
        ep_failed = False
        step_i = 0
        while not orch.done:
            state = gt.state_dict(place_pos, support_z)
            action = orch.step(state)
            _, _, terminated, truncated, _ = env.step(torch.tensor(action, device=gt.env.device).unsqueeze(0))
            if record_this and step_i % params.episode.record_divisor == 0:
                push_frame(recorder, gt, action, place_pos)
                ep_frames += 1
            step_i += 1
            if debug_dir is not None and ep == 0 and orch.phase != last_phase:
                save_debug_image(env, debug_dir, f"ep{ep}_{step_i:05d}_{orch.phase}")
                tip_p, tip_q = gt.tip_pose()
                obj_p, _ = gt.object_pose()
                q_err = gt.q_arm() - orch._hold if orch._hold is not None else np.zeros(6)
                print(f"[dbg {orch.phase}] tip={np.round(tip_p, 4)} obj={np.round(obj_p, 4)} "
                      f"grip={np.round(gt.q_grip(), 4)} q_err={np.round(q_err, 4)}")
                last_phase = orch.phase
            if bool(terminated[0]) or bool(truncated[0]):
                orch._fail("env_terminated")
                ep_failed = True

        if orch.phase == "FAILED":
            failures[orch.failure_reason] += 1
            if record_this:
                recorder.cancel_recording()
            print(f"[ep {ep}] FAIL {orch.failure_reason}")
            continue

        # success gate: object must hold the place pose for settle_needed steps
        ok_steps = 0
        hold = np.concatenate([gt.q_arm(), gt.q_grip()]).astype(np.float32)
        for _ in range(int(params.success.post_release_timeout_s / control_dt)):
            env.step(torch.tensor(hold, device=gt.env.device).unsqueeze(0))
            ok_steps = ok_steps + 1 if check_success(gt, params, place_pos, support_z) else 0
            if ok_steps >= settle_needed:
                break
        if ok_steps >= settle_needed:
            succeeded += 1
            if record_this:
                recorder.save_episode()
                saved += 1
            print(f"[ep {ep}] SUCCESS ({ep_frames} frames)")
        else:
            failures["place_not_stable"] += 1
            if record_this:
                recorder.cancel_recording()
            print(f"[ep {ep}] FAIL place_not_stable")

    if recorder is not None:
        recorder.finalize()
        _fix_output_ownership(dataset_root)

    wall = time.time() - t_start
    print("\n===== pick-and-place generation report =====")
    print(f"attempted: {attempted}  succeeded: {succeeded}  saved: {saved}")
    print(f"success rate: {100.0 * succeeded / max(attempted, 1):.1f}%")
    print(f"failure histogram: {dict(failures)}")
    print(f"wall time: {wall:.0f}s ({wall / max(attempted, 1):.1f}s/episode)")
    env.close()


def push_frame(recorder, gt: GroundTruth, action: np.ndarray, place_pos: np.ndarray):
    state = np.concatenate([gt.q_arm(), gt.q_grip()]).astype(np.float32)
    obj_pos, obj_quat = gt.object_pose()
    env_state = np.concatenate([obj_pos, obj_quat, place_pos,
                                [1.0, 0.0, 0.0, 0.0]]).astype(np.float32)
    images = {}
    for schema_key, sensor in (("external", "camera_external"), ("wrist", "camera_wrist")):
        if sensor in gt.scene.sensors:
            images[schema_key] = gt.scene.sensors[sensor].data.output["rgb"][0]
    recorder.push_frame_to_buffer(action, state, images,
                                  extras={"observation.environment_state": env_state})


if __name__ == "__main__":
    main()
    simulation_app.close()
