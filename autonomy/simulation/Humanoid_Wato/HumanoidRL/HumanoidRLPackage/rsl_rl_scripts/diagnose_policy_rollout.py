# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Inspect trained policy actions and joint motion during a finite rollout."""

import argparse

from isaaclab.app import AppLauncher

import cli_args  # isort: skip


parser = argparse.ArgumentParser(description="Compare left/right joint usage for a trained RSL-RL policy.")
parser.add_argument("--task", type=str, required=True, help="Name of the Isaac Lab play task.")
parser.add_argument("--num_envs", type=int, default=32, help="Number of environments to simulate.")
parser.add_argument("--steps", type=int, default=1000, help="Number of policy steps to record.")
parser.add_argument("--warmup_steps", type=int, default=50, help="Steps to ignore before collecting statistics.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import importlib.metadata as metadata
import os
import torch

from packaging import version
from rsl_rl.runners import OnPolicyRunner

try:
    # DistillationRunner was added in a later rsl-rl-lib release. Older versions
    # (e.g. the one pinned in some Isaac Sim containers) don't export it, so import
    # it optionally and only fail if a task actually requests class_name="DistillationRunner".
    from rsl_rl.runners import DistillationRunner
except ImportError:
    DistillationRunner = None

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.math import quat_rotate_inverse, yaw_quat
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg


installed_rsl_rl_version = metadata.version("rsl-rl-lib")


class RunningStats:
    def __init__(self, shape: tuple[int, ...], device: str):
        self.count = 0
        self.sum = torch.zeros(shape, device=device)
        self.sumsq = torch.zeros(shape, device=device)
        self.min = torch.full(shape, float("inf"), device=device)
        self.max = torch.full(shape, float("-inf"), device=device)

    def update(self, values: torch.Tensor) -> None:
        flat = values.reshape(-1, values.shape[-1])
        self.count += flat.shape[0]
        self.sum += flat.sum(dim=0)
        self.sumsq += torch.square(flat).sum(dim=0)
        self.min = torch.minimum(self.min, flat.min(dim=0).values)
        self.max = torch.maximum(self.max, flat.max(dim=0).values)

    def mean(self) -> torch.Tensor:
        return self.sum / max(self.count, 1)

    def std(self) -> torch.Tensor:
        mean = self.mean()
        var = self.sumsq / max(self.count, 1) - torch.square(mean)
        return torch.sqrt(torch.clamp(var, min=0.0))

    def range(self) -> torch.Tensor:
        return self.max - self.min


def _abs_mean(values: torch.Tensor) -> torch.Tensor:
    return values.abs().mean(dim=0)


def _print_pair(label: str, left_name: str, right_name: str, joint_names: list[str], action_abs, pos_range, vel_abs):
    left_idx = joint_names.index(left_name)
    right_idx = joint_names.index(right_name)
    action_ratio = action_abs[left_idx] / torch.clamp(action_abs[right_idx], min=1.0e-6)
    range_ratio = pos_range[left_idx] / torch.clamp(pos_range[right_idx], min=1.0e-6)
    vel_ratio = vel_abs[left_idx] / torch.clamp(vel_abs[right_idx], min=1.0e-6)
    print(
        f"{label:<8} "
        f"action_abs L/R={action_abs[left_idx].item():.4f}/{action_abs[right_idx].item():.4f} "
        f"ratio={action_ratio.item():.2f} | "
        f"pos_range L/R={pos_range[left_idx].item():.4f}/{pos_range[right_idx].item():.4f} "
        f"ratio={range_ratio.item():.2f} | "
        f"vel_abs L/R={vel_abs[left_idx].item():.4f}/{vel_abs[right_idx].item():.4f} "
        f"ratio={vel_ratio.item():.2f}",
        flush=True,
    )


def main() -> None:
    if args_cli.checkpoint is None:
        raise ValueError("Please pass --checkpoint with the model_*.pt file to inspect.")

    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    resume_path = retrieve_file_path(args_cli.checkpoint)
    env_cfg.log_dir = os.path.dirname(resume_path)

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    env = RslRlVecEnvWrapper(env)
    base_env = env.unwrapped
    robot = base_env.scene["robot"]

    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        if DistillationRunner is None:
            raise ImportError(
                "agent_cfg.class_name is 'DistillationRunner' but the installed rsl-rl-lib "
                f"version ({installed_rsl_rl_version}) does not provide it. Upgrade rsl-rl-lib "
                "or switch this task's runner config to OnPolicyRunner."
            )
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=base_env.device)

    action_dim = base_env.single_action_space.shape[0]
    joint_names = list(robot.joint_names)
    if action_dim != len(joint_names):
        print(f"[warning] action_dim={action_dim} joint_count={len(joint_names)}", flush=True)

    action_stats = RunningStats((action_dim,), base_env.device)
    joint_pos_stats = RunningStats((len(joint_names),), base_env.device)
    joint_vel_stats = RunningStats((len(joint_names),), base_env.device)
    action_abs_sum = torch.zeros(action_dim, device=base_env.device)
    joint_vel_abs_sum = torch.zeros(len(joint_names), device=base_env.device)
    sample_count = 0

    contact_sensor = base_env.scene.sensors.get("contact_forces")
    foot_contact_sum = None
    foot_height_stats = None
    foot_indices = []
    if contact_sensor is not None:
        foot_indices = [contact_sensor.body_names.index(name) for name in ("Foot_L", "Foot_R")]
        foot_contact_sum = torch.zeros(2, device=base_env.device)
        body_indices = [robot.body_names.index(name) for name in ("Foot_L", "Foot_R")]
        body_pair_indices = {}
        for pair_label, pair_names in {
            "foot": ("Foot_L", "Foot_R"),
            "thigh": ("Rotation_L", "Rotation_R"),
            "calf": ("Calf_L", "Calf_R"),
            "ankle_roll": ("Universal_Roll_L", "Universal_Roll_R"),
        }.items():
            if all(name in robot.body_names for name in pair_names):
                body_pair_indices[pair_label] = [robot.body_names.index(name) for name in pair_names]
        foot_height_stats = RunningStats((2,), base_env.device)
        foot_x_stats = RunningStats((2,), base_env.device)
        foot_y_stats = RunningStats((2,), base_env.device)
        pair_width_stats = {
            pair_label: RunningStats((1,), base_env.device) for pair_label in body_pair_indices.keys()
        }
        foot_rel_vel_x_abs_sum = torch.zeros(2, device=base_env.device)
    else:
        body_indices = []
        body_pair_indices = {}
        foot_x_stats = None
        foot_y_stats = None
        pair_width_stats = {}
        foot_rel_vel_x_abs_sum = None

    obs = env.get_observations()
    for step_idx in range(args_cli.steps):
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            if version.parse(installed_rsl_rl_version) >= version.parse("4.0.0"):
                policy.reset(dones)

        if step_idx < args_cli.warmup_steps:
            continue

        action_stats.update(actions)
        joint_pos_stats.update(robot.data.joint_pos)
        joint_vel_stats.update(robot.data.joint_vel)
        action_abs_sum += _abs_mean(actions)
        joint_vel_abs_sum += _abs_mean(robot.data.joint_vel)
        sample_count += 1

        if contact_sensor is not None:
            forces = contact_sensor.data.net_forces_w[:, foot_indices, :]
            foot_contact_sum += (torch.linalg.norm(forces, dim=-1) > 1.0).float().mean(dim=0)
            foot_heights = robot.data.body_pos_w[:, body_indices, 2]
            foot_height_stats.update(foot_heights)
            foot_pos_w = robot.data.body_pos_w[:, body_indices, :]
            foot_vel_w = robot.data.body_lin_vel_w[:, body_indices, :]
            rel_pos_w = foot_pos_w - robot.data.root_pos_w[:, None, :]
            rel_vel_w = foot_vel_w - robot.data.root_lin_vel_w[:, None, :]
            yaw_quat_w = yaw_quat(robot.data.root_quat_w).repeat_interleave(2, dim=0)
            foot_pos_yaw = quat_rotate_inverse(yaw_quat_w, rel_pos_w.reshape(-1, 3)).reshape(-1, 2, 3)
            foot_vel_yaw = quat_rotate_inverse(yaw_quat_w, rel_vel_w.reshape(-1, 3)).reshape(-1, 2, 3)
            foot_x_stats.update(foot_pos_yaw[:, :, 0])
            foot_y_stats.update(foot_pos_yaw[:, :, 1])
            for pair_label, pair_indices in body_pair_indices.items():
                pair_pos_w = robot.data.body_pos_w[:, pair_indices, :]
                rel_pair_pos_w = pair_pos_w - robot.data.root_pos_w[:, None, :]
                pair_pos_yaw = quat_rotate_inverse(yaw_quat_w, rel_pair_pos_w.reshape(-1, 3)).reshape(-1, 2, 3)
                pair_width_stats[pair_label].update(
                    torch.abs(pair_pos_yaw[:, 0, 1] - pair_pos_yaw[:, 1, 1]).unsqueeze(-1)
                )
            foot_rel_vel_x_abs_sum += foot_vel_yaw[:, :, 0].abs().mean(dim=0)

    action_abs = action_abs_sum / max(sample_count, 1)
    joint_vel_abs = joint_vel_abs_sum / max(sample_count, 1)
    pos_range = joint_pos_stats.range()

    print(f"\ncheckpoint: {resume_path}", flush=True)
    print(f"task: {args_cli.task}", flush=True)
    print(f"recorded_steps: {sample_count} after warmup={args_cli.warmup_steps}", flush=True)
    print(f"joint_names: {joint_names}", flush=True)

    print("\n[left_right_pairs]")
    _print_pair("hip_F", "F_L", "F_R", joint_names, action_abs, pos_range, joint_vel_abs)
    _print_pair("hip_A", "A_L", "A_R", joint_names, action_abs, pos_range, joint_vel_abs)
    _print_pair("hip_R", "R_L", "R_R", joint_names, action_abs, pos_range, joint_vel_abs)
    _print_pair("knee", "Knee_L", "Knee_R", joint_names, action_abs, pos_range, joint_vel_abs)
    _print_pair("ankle_R", "U_R_L", "U_R_R", joint_names, action_abs, pos_range, joint_vel_abs)
    _print_pair("ankle_P", "U_P_L", "U_P_R", joint_names, action_abs, pos_range, joint_vel_abs)

    print("\n[joint_table]")
    print("idx  joint       action_abs  action_std  pos_min  pos_max  pos_range  vel_abs")
    action_std = action_stats.std()
    for idx, name in enumerate(joint_names):
        print(
            f"{idx:>2}   {name:<10} "
            f"{action_abs[idx].item():>10.4f} {action_std[idx].item():>10.4f} "
            f"{joint_pos_stats.min[idx].item():>7.4f} {joint_pos_stats.max[idx].item():>7.4f} "
            f"{pos_range[idx].item():>9.4f} {joint_vel_abs[idx].item():>8.4f}",
            flush=True,
        )

    if foot_contact_sum is not None and foot_height_stats is not None:
        contact_rate = foot_contact_sum / max(sample_count, 1)
        height_range = foot_height_stats.range()
        x_range = foot_x_stats.range()
        y_range = foot_y_stats.range()
        rel_vel_x_abs = foot_rel_vel_x_abs_sum / max(sample_count, 1)
        print("\n[feet]")
        print(
            f"Foot_L contact_rate={contact_rate[0].item():.3f} "
            f"height_range={height_range[0].item():.4f} "
            f"x_range={x_range[0].item():.4f} "
            f"y_range={y_range[0].item():.4f} "
            f"rel_vel_x_abs={rel_vel_x_abs[0].item():.4f}",
            flush=True,
        )
        print(
            f"Foot_R contact_rate={contact_rate[1].item():.3f} "
            f"height_range={height_range[1].item():.4f} "
            f"x_range={x_range[1].item():.4f} "
            f"y_range={y_range[1].item():.4f} "
            f"rel_vel_x_abs={rel_vel_x_abs[1].item():.4f}",
            flush=True,
        )
        for pair_label, width_stats in pair_width_stats.items():
            width_mean = width_stats.mean()[0]
            width_min = width_stats.min[0]
            print(f"{pair_label}_width mean={width_mean.item():.4f} min={width_min.item():.4f}", flush=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
