"""Record a small seed set of source demonstrations for Isaac Lab Mimic, in
Isaac Lab's native HDF5 format (isaaclab.utils.datasets), by driving the SAME
cuRobo orchestrator generate_demos.py uses -- just against the Mimic env
variant (pick_place_bimanual_mimic_env.PickPlaceBimanualMimicEnv), with Isaac
Lab's own recorder manager attached instead of the LeRobot recorder.

Output feeds Isaac Lab Mimic's stock tooling:
  cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
  ISAACLAB_MIMIC=<path to IsaacLab>/scripts/imitation_learning/isaaclab_mimic

  PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $ISAACLAB_MIMIC/annotate_demos.py \
      --task Isaac-PickPlace-BimanualLeft-Mimic-v0 --headless --auto \
      --input_file <output_file> --output_file annotated.hdf5

  PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $ISAACLAB_MIMIC/generate_dataset.py \
      --task Isaac-PickPlace-BimanualLeft-Mimic-v0 --headless \
      --input_file annotated.hdf5 --output_file scaled.hdf5 \
      --generation_num_trials 500

NOTE: GroundTruth / sample_place_target / check_success / _tray_wall_obstacles
below are duplicated from generate_demos.py rather than imported: that script
parses CLI args and launches the sim app at import time (module-level
side effects), so it can't be imported as a library. sample_place_target's
"table" mode branch is intentionally NOT identical to generate_demos.py's --
see the comment on that function. Keep both copies in sync if the placement/
success logic in task_params.py ever changes.

Run from inside the simulation_il container, same convention as
generate_demos.py:
  cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
  PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p \
      ../../pick_place_gen/record_mimic_source_demos.py --headless \
      --num_episodes 20 --seed 0 --task_params <yaml> \
      --output_file datasets/pick_place_mimic_source.hdf5
"""
import argparse
import math
import sys
from pathlib import Path

# Pin site-packages warp before Kit loads its older omni.warp bundle -- same
# trick as generate_demos.py (cuRobo's collision kernels need this).
import warp  # noqa: F401

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_episodes", type=int, default=20, help="Number of SUCCESSFUL source demos to save.")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--task_params", type=str, default=None, help="YAML overriding PickPlaceTaskParams")
parser.add_argument("--output_file", type=str, default="./datasets/pick_place_mimic_source.hdf5")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg  # noqa: E402
from isaaclab.managers import RecorderTerm, RecorderTermCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

_GEN_DIR = Path(__file__).resolve().parent
_HRL_DIR = _GEN_DIR.parents[1] / "Humanoid_Wato" / "HumanoidRL"
for _p in (str(_GEN_DIR), str(_HRL_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: E402,F401
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual import robot_cfg_shim as shim  # noqa: E402
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual.pick_place_bimanual_mimic_env import (  # noqa: E402
    make_mimic_env_cfg,
)

import wato_constants as wc  # noqa: E402
from curobo_expert import CuRoboExpert  # noqa: E402
from orchestrator import Orchestrator  # noqa: E402
from task_params import PickPlaceTaskParams  # noqa: E402


# ---------------------------------------------------------------------------
# Duplicated from generate_demos.py -- see module docstring.
# ---------------------------------------------------------------------------

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
    s = place.tray_scale
    cx, cy = place.tray_center
    hx = wc.TRAY_FOOTPRINT[0] / 2 * s
    hy = wc.TRAY_FOOTPRINT[1] / 2 * s
    t = max(wc.TRAY_WALL_THICK * s, 0.006)
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
    """(place_pos xy+z_center, support_top_z).

    "table" mode differs from generate_demos.py's version on purpose: it
    reads the LIVE place_target marker (randomized by the Mimic cfg's
    reset_objects_min_separation event) instead of independently drawing a
    fresh random point. The recorded demo's target must match what
    get_object_poses()/the success termination read off that same marker --
    an independently-sampled point would silently diverge from it.
    """
    obj_pos, _ = gt.object_pose()
    if params.place.mode == "stack":
        pl_pos, _ = gt.object_pose("place_object")
        support_z = pl_pos[2] + params.place.stack_object_size[2] / 2
        target = np.array([pl_pos[0], pl_pos[1], support_z + params.object.size[2] / 2])
        return target, support_z
    if params.place.mode == "tray":
        sz = params.place.tray_scale * params.place.tray_height_scale
        support_z = wc.TABLE_TOP_Z + sz * wc.TRAY_FLOOR_LOCAL_Z
        cx, cy = params.place.tray_center
        return np.array([cx, cy, support_z + params.object.size[2] / 2]), support_z
    marker_pos, _ = gt.object_pose("place_target")
    support_z = wc.TABLE_TOP_Z
    return np.array([marker_pos[0], marker_pos[1], support_z + params.object.size[2] / 2]), support_z


def check_success(gt: GroundTruth, params, place_pos, support_z) -> bool:
    obj_pos, _ = gt.object_pose()
    xy_err = float(np.linalg.norm(obj_pos[:2] - place_pos[:2]))
    z_err = abs(float(obj_pos[2]) - (support_z + params.object.size[2] / 2))
    vel = float(np.linalg.norm(gt.object_lin_vel()))
    return (xy_err < params.place.xy_tolerance and z_err < params.place.z_tolerance
            and vel < params.success.max_lin_vel)


# ---------------------------------------------------------------------------
# Isaac Lab native recorder wiring (duplicated from IsaacLab's own
# annotate_demos.py -- same recorder terms, minus the subtask START signals
# term, which we don't use).
# ---------------------------------------------------------------------------

class PreStepDatagenInfoRecorder(RecorderTerm):
    def record_pre_step(self):
        eef_pose_dict = {
            eef_name: self._env.get_robot_eef_pose(eef_name=eef_name)
            for eef_name in self._env.cfg.subtask_configs.keys()
        }
        datagen_info = {
            "object_pose": self._env.get_object_poses(),
            "eef_pose": eef_pose_dict,
            "target_eef_pose": self._env.action_to_target_eef_pose(self._env.action_manager.action),
        }
        return "obs/datagen_info", datagen_info


@configclass
class PreStepDatagenInfoRecorderCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = PreStepDatagenInfoRecorder


class PreStepSubtaskTermsObservationsRecorder(RecorderTerm):
    def record_pre_step(self):
        return "obs/datagen_info/subtask_term_signals", self._env.get_subtask_term_signals()


@configclass
class PreStepSubtaskTermsObservationsRecorderCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = PreStepSubtaskTermsObservationsRecorder


@configclass
class MimicRecorderManagerCfg(ActionStateRecorderManagerCfg):
    record_pre_step_datagen_info = PreStepDatagenInfoRecorderCfg()
    record_pre_step_subtask_term_signals = PreStepSubtaskTermsObservationsRecorderCfg()


def main():
    params = PickPlaceTaskParams.from_yaml(args_cli.task_params)
    params.cameras.enabled = False  # source demos are state-only; Mimic scales actions, not video
    shim.check_constants_consistency()

    output_path = Path(args_cli.output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cfg = make_mimic_env_cfg(params)
    cfg.recorders = MimicRecorderManagerCfg()
    cfg.recorders.dataset_export_dir_path = str(output_path.parent)
    cfg.recorders.dataset_filename = output_path.stem
    # Success is checked manually below (see mdp/terminations.py's module
    # docstring: it must never run live in the TerminationManager here,
    # same reasoning as generate_demos.py's env_terminated failure path).
    cfg.terminations.success = None

    env = gym.make("Isaac-PickPlace-BimanualLeft-Mimic-v0", cfg=cfg)
    env.reset(seed=args_cli.seed)
    # This first reset's own internal record_pre_reset() auto-exports the
    # (empty) episode buffer as a bogus demo_0 (0 samples, success=False),
    # same mechanism as the in-loop failure branches below -- clear it.
    env.unwrapped.recorder_manager.reset()
    rng = np.random.default_rng(args_cli.seed)
    gt = GroundTruth(env, params)

    action_term = env.unwrapped.action_manager.get_term("arm_action")
    assert action_term._joint_names == shim.RIGHT_JOINTS_ALL, action_term._joint_names

    expert = CuRoboExpert(params)
    assert expert.joint_names == shim.RIGHT_ARM_JOINTS, expert.joint_names
    orch = Orchestrator(expert, params, rng)

    control_dt = params.episode.sim_dt * params.episode.decimation
    settle_needed = params.success.settle_steps
    attempted = 0
    succeeded = 0

    while succeeded < args_cli.num_episodes:
        attempted += 1
        # Clear any leftover buffer from a prior FAILED attempt BEFORE the
        # reset -- not after. env.reset()'s own record_post_reset() records
        # this episode's initial_state into the (now-empty) buffer, and
        # annotate_demos.py's replay_episode() requires that initial_state to
        # reset_to(). Resetting the recorder *after* env.reset() (as an
        # earlier version did) wiped that initial_state, producing demos with
        # actions but no initial_state -> KeyError('initial_state') at replay.
        env.unwrapped.recorder_manager.reset()
        env.reset()
        hold0 = np.concatenate([gt.q_arm(), gt.q_grip()]).astype(np.float32)
        for _ in range(10):
            env.step(torch.tensor(hold0, device=gt.env.device).unsqueeze(0))

        place_pos, support_z = sample_place_target(gt, params, rng)
        state = gt.state_dict(place_pos, support_z)

        if not orch.start_episode(state):
            print(f"[attempt {attempted}] FAIL {orch.failure_reason}")
            env.unwrapped.recorder_manager.reset()
            continue

        while not orch.done:
            state = gt.state_dict(place_pos, support_z)
            action = orch.step(state)
            _, _, terminated, truncated, _ = env.step(torch.tensor(action, device=gt.env.device).unsqueeze(0))
            if bool(terminated[0]) or bool(truncated[0]):
                orch._fail("env_terminated")

        if orch.phase == "FAILED":
            print(f"[attempt {attempted}] FAIL {orch.failure_reason}")
            env.unwrapped.recorder_manager.reset()
            continue

        ok_steps = 0
        hold = np.concatenate([gt.q_arm(), gt.q_grip()]).astype(np.float32)
        for _ in range(int(params.success.post_release_timeout_s / control_dt)):
            env.step(torch.tensor(hold, device=gt.env.device).unsqueeze(0))
            ok_steps = ok_steps + 1 if check_success(gt, params, place_pos, support_z) else 0
            if ok_steps >= settle_needed:
                break

        if ok_steps >= settle_needed:
            succeeded += 1
            env.unwrapped.recorder_manager.set_success_to_episodes(
                None, torch.tensor([[True]], dtype=torch.bool, device=env.unwrapped.device)
            )
            env.unwrapped.recorder_manager.export_episodes()
            print(f"[attempt {attempted}] SUCCESS ({succeeded}/{args_cli.num_episodes} saved)")
        else:
            print(f"[attempt {attempted}] FAIL place_not_stable")
            env.unwrapped.recorder_manager.reset()

    print(f"\nSaved {succeeded} source demos to {output_path} (attempted {attempted}).")
    print(
        "Next: annotate_demos.py --task Isaac-PickPlace-BimanualLeft-Mimic-v0 "
        f"--headless --auto --input_file {output_path} --output_file <annotated>.hdf5"
    )
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
