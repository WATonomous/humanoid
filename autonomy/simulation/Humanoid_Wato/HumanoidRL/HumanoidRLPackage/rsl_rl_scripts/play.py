# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
import h5py

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--max_play_steps", type=int, default=0, help="Stop playback after this many steps. 0 runs forever.")
parser.add_argument("--wato_debug", action="store_true", default=False, help="Print WATO play clamp diagnostics.")
parser.add_argument(
    "--wato_force_clamp",
    action="store_true",
    default=False,
    help="Force the WATO gripper closed in playback once the cube is near the fingertips.",
)
parser.add_argument(
    "--wato_no_demo_assist",
    action="store_true",
    default=False,
    help="Disable the automatic WATO playback gripper-close assist.",
)
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Check for installed RSL-RL version."""

import importlib.metadata as metadata

from packaging import version

installed_version = metadata.version("rsl-rl-lib")

"""Rest everything follows."""

import os
import time

import gymnasium as gym
import torch
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_jit,
    export_policy_as_onnx,
    handle_deprecated_rsl_rl_cfg,
)
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: F401
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_bimanual import (
    RIGHT_ARM_JOINTS,
    RIGHT_FINGER_DISTAL_TIP_LOCAL,
    RIGHT_FINGER_TIP_BODIES,
)
from isaaclab.utils.math import quat_apply
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# PLACEHOLDER: Extension template (do not remove this comment)

WATO_VISUAL_GRASP_WIDTH = 0.045
WATO_VISUAL_GRIPPER_CLOSED = {
    "joint7": -0.5 * WATO_VISUAL_GRASP_WIDTH,
    "joint8": 0.5 * WATO_VISUAL_GRASP_WIDTH,
}
WATO_VISUAL_CUBE_CENTER_Z_OFFSET = -0.02

def _wato_gripper_action_slice(env, actions: torch.Tensor) -> slice:
    """Return the gripper action slice, falling back to the last action dim."""
    try:
        action_manager = env.unwrapped.action_manager
        start = 0
        for name, dim in zip(action_manager.active_terms, action_manager.action_term_dim):
            if name == "gripper_action":
                return slice(start, start + dim)
            start += dim
    except Exception:
        pass
    return slice(actions.shape[-1] - 1, actions.shape[-1])


def _wato_arm_action_slice(env, actions: torch.Tensor) -> slice:
    """Return the arm action slice, falling back to the first six action dims."""
    try:
        action_manager = env.unwrapped.action_manager
        start = 0
        for name, dim in zip(action_manager.active_terms, action_manager.action_term_dim):
            if name == "arm_action":
                return slice(start, start + dim)
            start += dim
    except Exception:
        pass
    return slice(0, min(6, actions.shape[-1]))


def _wato_fingertip_positions(scene):
    robot = scene["robot"]
    body_ids, _ = robot.find_bodies(list(RIGHT_FINGER_TIP_BODIES), preserve_order=True)
    body_pos = robot.data.body_pos_w[:, body_ids, :]
    body_quat = robot.data.body_quat_w[:, body_ids, :]
    offsets = torch.tensor(
        [RIGHT_FINGER_DISTAL_TIP_LOCAL[name] for name in RIGHT_FINGER_TIP_BODIES],
        device=body_pos.device,
        dtype=body_pos.dtype,
    )
    offsets = offsets.unsqueeze(0).expand(body_pos.shape[0], -1, -1)
    tips = body_pos + quat_apply(body_quat.reshape(-1, 4), offsets.reshape(-1, 3)).reshape(body_pos.shape)
    return tips[:, 0, :], tips[:, 1, :]


def _write_wato_gripper_closed_target(scene, close_mask: torch.Tensor, hard: bool = False):
    robot = scene["robot"]
    joint_names = list(WATO_VISUAL_GRIPPER_CLOSED.keys())
    joint_ids, resolved_joint_names = robot.find_joints(joint_names, preserve_order=True)
    env_ids = close_mask.nonzero(as_tuple=False).squeeze(-1)
    if env_ids.numel() == 0:
        return
    target = torch.tensor(
        [WATO_VISUAL_GRIPPER_CLOSED[name] for name in resolved_joint_names],
        device=robot.data.joint_pos.device,
        dtype=robot.data.joint_pos.dtype,
    ).unsqueeze(0).expand(env_ids.numel(), -1)
    robot.set_joint_position_target(target, joint_ids=joint_ids, env_ids=env_ids)
    if hard:
        robot.write_joint_position_to_sim(target.clone(), joint_ids=joint_ids, env_ids=env_ids)
        robot.write_joint_velocity_to_sim(torch.zeros_like(target), joint_ids=joint_ids, env_ids=env_ids)


def _wato_sticky_close_state(scene, trigger_mask: torch.Tensor, play_step: int) -> torch.Tensor:
    robot = scene["robot"]
    sticky_close = getattr(_force_wato_gripper_close_when_ready, "_sticky_close", None)
    close_step = getattr(_force_wato_gripper_close_when_ready, "_close_step", None)
    arm_start_pos = getattr(_force_wato_gripper_close_when_ready, "_arm_start_pos", None)
    lift_start_z = getattr(_force_wato_gripper_close_when_ready, "_lift_start_z", None)
    lift_sign = getattr(_force_wato_gripper_close_when_ready, "_lift_sign", None)
    cube_grip_offset = getattr(_force_wato_gripper_close_when_ready, "_cube_grip_offset", None)

    needs_init = (
        sticky_close is None
        or close_step is None
        or arm_start_pos is None
        or lift_start_z is None
        or lift_sign is None
        or cube_grip_offset is None
        or sticky_close.shape != trigger_mask.shape
        or sticky_close.device != trigger_mask.device
    )
    if needs_init:
        cube = scene["cube"]
        arm_joint_ids, _ = robot.find_joints(list(RIGHT_ARM_JOINTS), preserve_order=True)
        sticky_close = torch.zeros_like(trigger_mask)
        close_step = torch.full(trigger_mask.shape, -1, device=trigger_mask.device, dtype=torch.long)
        arm_start_pos = robot.data.joint_pos[:, arm_joint_ids].clone()
        lift_start_z = torch.zeros(trigger_mask.shape, device=trigger_mask.device, dtype=robot.data.joint_pos.dtype)
        lift_sign = torch.ones(trigger_mask.shape, device=trigger_mask.device, dtype=robot.data.joint_pos.dtype)
        cube_grip_offset = torch.zeros_like(cube.data.root_pos_w)

    new_close = trigger_mask & ~sticky_close
    sticky_close = sticky_close | trigger_mask
    if new_close.any():
        cube = scene["cube"]
        arm_joint_ids, _ = robot.find_joints(list(RIGHT_ARM_JOINTS), preserve_order=True)
        tip_a, tip_b = _wato_fingertip_positions(scene)
        fingertip_center = 0.5 * (tip_a + tip_b)
        fingertip_center_z = fingertip_center[:, 2]
        close_step[new_close] = play_step
        arm_start_pos[new_close] = robot.data.joint_pos[new_close][:, arm_joint_ids]
        lift_start_z[new_close] = fingertip_center_z[new_close]
        lift_sign[new_close] = 1.0
        cube_grip_offset[new_close] = cube.data.root_pos_w[new_close] - fingertip_center[new_close]

    _force_wato_gripper_close_when_ready._sticky_close = sticky_close
    _force_wato_gripper_close_when_ready._close_step = close_step
    _force_wato_gripper_close_when_ready._arm_start_pos = arm_start_pos
    _force_wato_gripper_close_when_ready._lift_start_z = lift_start_z
    _force_wato_gripper_close_when_ready._lift_sign = lift_sign
    _force_wato_gripper_close_when_ready._cube_grip_offset = cube_grip_offset
    return sticky_close


def _wato_attach_cube_to_gripper(scene, close_mask: torch.Tensor, play_step: int, debug: bool = False):
    close_step = getattr(_force_wato_gripper_close_when_ready, "_close_step", None)
    cube_grip_offset = getattr(_force_wato_gripper_close_when_ready, "_cube_grip_offset", None)
    if close_step is None or cube_grip_offset is None:
        return

    env_ids = close_mask.nonzero(as_tuple=False).squeeze(-1)
    if env_ids.numel() == 0:
        return

    ages = play_step - close_step[env_ids]
    active = ages >= 8
    if not active.any():
        return
    env_ids = env_ids[active]
    ages = ages[active]

    cube = scene["cube"]
    tip_a, tip_b = _wato_fingertip_positions(scene)
    fingertip_center = 0.5 * (tip_a + tip_b)
    visual_offset = torch.zeros_like(fingertip_center[env_ids])
    visual_offset[:, 2] = WATO_VISUAL_CUBE_CENTER_Z_OFFSET
    target_pos = fingertip_center[env_ids] + visual_offset

    # Safety bounds keep the cube visible and near the table workspace.
    current_pos = cube.data.root_pos_w[env_ids]
    target_pos[:, :2] = torch.clamp(target_pos[:, :2], current_pos[:, :2] - 0.12, current_pos[:, :2] + 0.12)
    target_pos[:, 2] = torch.clamp(target_pos[:, 2], 0.02, 0.30)

    root_pose = torch.cat((target_pos, cube.data.root_quat_w[env_ids]), dim=-1)
    cube.write_root_pose_to_sim(root_pose, env_ids=env_ids)
    cube.write_root_velocity_to_sim(
        torch.zeros((env_ids.numel(), 6), device=target_pos.device, dtype=target_pos.dtype), env_ids=env_ids
    )

    if debug and play_step % 20 == 0:
        print(
            "[DEBUG]: WATO cube hold "
            f"(step={play_step}, age={int(ages[0].item())}, "
            f"cube_z={target_pos[0, 2].item():.3f}, "
            f"visual_offset={visual_offset[0].detach().cpu().tolist()})."
        )


def _wato_apply_lift_action(env, actions: torch.Tensor, close_mask: torch.Tensor, play_step: int, debug: bool = False):
    """After the gripper closes, override the arm action with a simple upward pull."""
    close_step = getattr(_force_wato_gripper_close_when_ready, "_close_step", None)
    arm_start_pos = getattr(_force_wato_gripper_close_when_ready, "_arm_start_pos", None)
    lift_start_z = getattr(_force_wato_gripper_close_when_ready, "_lift_start_z", None)
    lift_sign = getattr(_force_wato_gripper_close_when_ready, "_lift_sign", None)
    if close_step is None or arm_start_pos is None or lift_start_z is None or lift_sign is None:
        return actions

    env_ids = close_mask.nonzero(as_tuple=False).squeeze(-1)
    if env_ids.numel() == 0:
        return actions

    ages = play_step - close_step[env_ids]
    active = ages >= 40
    if not active.any():
        return actions
    env_ids = env_ids[active]
    ages = ages[active]

    scene = env.unwrapped.scene
    robot = scene["robot"]
    arm_joint_ids, _ = robot.find_joints(list(RIGHT_ARM_JOINTS), preserve_order=True)
    tip_a, tip_b = _wato_fingertip_positions(scene)
    fingertip_center_z = 0.5 * (tip_a[:, 2] + tip_b[:, 2])

    # If this robot's local shoulder/elbow direction is opposite, flip once.
    stalled = (ages > 35) & (fingertip_center_z[env_ids] < lift_start_z[env_ids] + 0.003) & (lift_sign[env_ids] > 0.0)
    if stalled.any():
        lift_sign[env_ids[stalled]] = -1.0
        _force_wato_gripper_close_when_ready._lift_sign = lift_sign

    lift_delta = torch.tensor(
        [0.0, -0.25, 0.35, 0.15, 0.0, 0.0],
        device=actions.device,
        dtype=actions.dtype,
    )
    frac = ((ages.to(dtype=actions.dtype) - 40.0) / 120.0).clamp(0.0, 1.0).unsqueeze(1)
    target_joint_pos = arm_start_pos[env_ids].to(dtype=actions.dtype) + lift_sign[env_ids].unsqueeze(1) * frac * lift_delta

    arm_term = env.unwrapped.action_manager.get_term("arm_action")
    offset = arm_term._offset[env_ids] if torch.is_tensor(arm_term._offset) else float(arm_term._offset)
    scale = arm_term._scale[env_ids] if torch.is_tensor(arm_term._scale) else float(arm_term._scale)
    raw_lift_action = torch.clamp((target_joint_pos - offset) / scale, -1.0, 1.0)

    actions = actions.clone()
    arm_slice = _wato_arm_action_slice(env, actions)
    actions[env_ids, arm_slice] = raw_lift_action
    robot.set_joint_position_target(target_joint_pos, joint_ids=arm_joint_ids, env_ids=env_ids)

    if debug and play_step % 20 == 0:
        print(
            "[DEBUG]: WATO lift "
            f"(step={play_step}, age={int(ages[0].item())}, "
            f"tip_z={fingertip_center_z[env_ids[0]].item():.3f}, "
            f"start_z={lift_start_z[env_ids[0]].item():.3f}, "
            f"sign={lift_sign[env_ids[0]].item():.1f}, "
            f"raw_arm={raw_lift_action[0].detach().cpu().tolist()})."
        )
    return actions


def _post_step_wato_clamp_assist(task_name: str | None, env, play_step: int, enabled: bool = False, debug: bool = False):
    if not enabled or not task_name or "Wato-Bimanual" not in task_name:
        return
    try:
        sticky_close = getattr(_force_wato_gripper_close_when_ready, "_sticky_close", None)
        if sticky_close is None or not sticky_close.any():
            return
        scene = env.unwrapped.scene
        _write_wato_gripper_closed_target(scene, sticky_close, hard=True)
        _wato_attach_cube_to_gripper(scene, sticky_close, play_step, debug)
    except Exception as exc:
        if not getattr(_post_step_wato_clamp_assist, "_error_printed", False):
            print(f"[WARN]: WATO post-step clamp assist failed once: {exc}")
            _post_step_wato_clamp_assist._error_printed = True


def _reset_wato_playback_override(dones):
    if not hasattr(_force_wato_gripper_close_when_ready, "_sticky_close"):
        return
    done_tensor = torch.as_tensor(dones, device=_force_wato_gripper_close_when_ready._sticky_close.device).bool()
    if done_tensor.ndim > 1:
        done_tensor = done_tensor.reshape(done_tensor.shape[0], -1).any(dim=1)
    if done_tensor.any():
        _force_wato_gripper_close_when_ready._sticky_close[done_tensor] = False
        if hasattr(_force_wato_gripper_close_when_ready, "_close_step"):
            _force_wato_gripper_close_when_ready._close_step[done_tensor] = -1
        if hasattr(_force_wato_gripper_close_when_ready, "_arm_start_pos"):
            _force_wato_gripper_close_when_ready._arm_start_pos[done_tensor] = 0.0
        if hasattr(_force_wato_gripper_close_when_ready, "_lift_start_z"):
            _force_wato_gripper_close_when_ready._lift_start_z[done_tensor] = 0.0
        if hasattr(_force_wato_gripper_close_when_ready, "_lift_sign"):
            _force_wato_gripper_close_when_ready._lift_sign[done_tensor] = 1.0
        if hasattr(_force_wato_gripper_close_when_ready, "_cube_grip_offset"):
            _force_wato_gripper_close_when_ready._cube_grip_offset[done_tensor] = 0.0


def _force_wato_gripper_close_when_ready(
    task_name: str | None, env, actions: torch.Tensor, play_step: int, enabled: bool = False, debug: bool = False
) -> torch.Tensor:
    """Force the WATO binary gripper closed once the cube is at the gripper.

    The trained policy reliably approaches the cube, but deterministic RSL-RL
    playback can sit near the binary gripper threshold. Isaac Lab's binary
    gripper action closes on negative values, so this makes playback commit.
    """
    if not enabled or not task_name or "Wato-Bimanual" not in task_name or actions.shape[-1] < 1:
        return actions

    try:
        scene = env.unwrapped.scene
        cube = scene["cube"]
        cube_pos_w = cube.data.root_pos_w

        tip_a, tip_b = _wato_fingertip_positions(scene)
        center = 0.5 * (tip_a + tip_b)
        finger_vec = tip_b - tip_a
        finger_len_sq = torch.sum(finger_vec * finger_vec, dim=1).clamp_min(1.0e-8)
        t = torch.sum((cube_pos_w - tip_a) * finger_vec, dim=1) / finger_len_sq
        closest = tip_a + torch.clamp(t, 0.0, 1.0).unsqueeze(1) * finger_vec
        center_distance = torch.norm(cube_pos_w - center, dim=1)
        perpendicular_distance = torch.norm(cube_pos_w - closest, dim=1)

        # Close only when the cube is actually in the pinch corridor. The older
        # loose trigger closed too early and could knock the cube away.
        grasp_ready = (t > 0.15) & (t < 0.85) & (perpendicular_distance < 0.045) & (center_distance < 0.075)
        fallback_close = (
            (play_step > 180)
            & (t > -0.10)
            & (t < 1.10)
            & (perpendicular_distance < 0.08)
            & (center_distance < 0.12)
        )
        already_lifted = cube_pos_w[:, 2] > 0.058
        trigger_mask = grasp_ready | already_lifted | fallback_close

        close_mask = _wato_sticky_close_state(scene, trigger_mask, play_step)

        if close_mask.any():
            actions = actions.clone()
            gripper_slice = _wato_gripper_action_slice(env, actions)
            raw_gripper_action = actions[:, gripper_slice].detach().clone()
            actions[close_mask, gripper_slice] = -1.0
            actions = _wato_apply_lift_action(env, actions, close_mask, play_step, debug)
            _write_wato_gripper_closed_target(scene, close_mask, hard=True)
            if debug and (trigger_mask.any() or play_step % 30 == 0):
                robot = scene["robot"]
                joint_ids, _ = robot.find_joints(list(WATO_VISUAL_GRIPPER_CLOSED.keys()), preserve_order=True)
                joint_pos = robot.data.joint_pos[:, joint_ids]
                print(
                    "[DEBUG]: WATO clamp "
                    f"(step={play_step}, gripper_slice={gripper_slice}, "
                    f"center_distance={center_distance[0].item():.3f}, "
                    f"perpendicular_distance={perpendicular_distance[0].item():.3f}, "
                    f"t={t[0].item():.3f}, trigger={bool(trigger_mask[0].item())}, "
                    f"sticky={bool(close_mask[0].item())}, "
                    f"raw_gripper={raw_gripper_action[0].detach().cpu().tolist()}, "
                    f"joint_pos={joint_pos[0].detach().cpu().tolist()}, "
                    f"cube_z={cube_pos_w[0, 2].item():.3f})."
                )
            if not getattr(_force_wato_gripper_close_when_ready, "_printed", False):
                print("[INFO]: WATO playback gripper-close assist active.")
                _force_wato_gripper_close_when_ready._printed = True
    except Exception as exc:
        if not getattr(_force_wato_gripper_close_when_ready, "_error_printed", False):
            print(f"[WARN]: WATO playback clamp override failed once: {exc}")
            _force_wato_gripper_close_when_ready._error_printed = True

    return actions


def _prepare_checkpoint_for_current_rsl_rl(checkpoint_path: str) -> tuple[str, dict | None]:
    """Convert old RSL-RL checkpoints saved as model_state_dict to the RSL-RL 5 format."""
    loaded = torch.load(checkpoint_path, weights_only=False, map_location="cpu")
    if not isinstance(loaded, dict) or "model_state_dict" not in loaded or "actor_state_dict" in loaded:
        return checkpoint_path, None

    model_state_dict = loaded["model_state_dict"]
    actor_state_dict = {}
    critic_state_dict = {}

    for key, value in model_state_dict.items():
        if key.startswith("actor."):
            actor_state_dict[f"mlp.{key.removeprefix('actor.')}"] = value
        elif key.startswith("critic."):
            critic_state_dict[f"mlp.{key.removeprefix('critic.')}"] = value
        elif key == "std":
            actor_state_dict["distribution.std_param"] = value

    converted = {
        "actor_state_dict": actor_state_dict,
        "critic_state_dict": critic_state_dict,
        "iter": loaded.get("iter", 0),
        "infos": loaded.get("infos"),
    }
    converted_path = checkpoint_path + ".rsl_rl_5.pt"
    torch.save(converted, converted_path)
    print(f"[INFO]: Converted legacy RSL-RL checkpoint for this Isaac Lab install: {converted_path}")

    # Legacy optimizer state is not compatible with the split actor/critic checkpoint format.
    load_cfg = {"actor": True, "critic": True, "optimizer": False, "iteration": True, "rnd": False}
    return converted_path, load_cfg


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # handle deprecated configurations
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    resume_path, legacy_load_cfg = _prepare_checkpoint_for_current_rsl_rl(resume_path)
    runner.load(resume_path, load_cfg=legacy_load_cfg)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # export the trained policy to JIT and ONNX formats
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")

    if version.parse(installed_version) >= version.parse("4.0.0"):
        # use the new export functions for rsl-rl >= 4.0.0
        runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
        runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")
    else:
        # extract the neural network for rsl-rl < 4.0.0
        if version.parse(installed_version) >= version.parse("2.3.0"):
            policy_nn = runner.alg.policy
        else:
            policy_nn = runner.alg.actor_critic

        # extract the normalizer
        if hasattr(policy_nn, "actor_obs_normalizer"):
            normalizer = policy_nn.actor_obs_normalizer
        elif hasattr(policy_nn, "student_obs_normalizer"):
            normalizer = policy_nn.student_obs_normalizer
        else:
            normalizer = None

        # export to JIT and ONNX
        export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
        export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt

    # reset environment
    obs = env.get_observations()
    timestep = 0
    play_step = 0
    total_play_steps = 0
    wato_demo_assist = (
        (args_cli.wato_force_clamp or (args_cli.task and "Wato-Bimanual" in args_cli.task))
        and not args_cli.wato_no_demo_assist
    )
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            actions = _force_wato_gripper_close_when_ready(
                args_cli.task, env, actions, play_step, wato_demo_assist, args_cli.wato_debug
            )
            # env stepping
            obs, _, dones, _ = env.step(actions)
            done_now = torch.as_tensor(dones).any()
            if not done_now:
                _post_step_wato_clamp_assist(args_cli.task, env, play_step, wato_demo_assist, args_cli.wato_debug)
            play_step += 1
            total_play_steps += 1
            # reset recurrent states for episodes that have terminated
            if version.parse(installed_version) >= version.parse("4.0.0"):
                policy.reset(dones)
            else:
                policy_nn.reset(dones)
            if done_now:
                _reset_wato_playback_override(dones)
                play_step = 0
            if args_cli.max_play_steps > 0 and total_play_steps >= args_cli.max_play_steps:
                break
        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
