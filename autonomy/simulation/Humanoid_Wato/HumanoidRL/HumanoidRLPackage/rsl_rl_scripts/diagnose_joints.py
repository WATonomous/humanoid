# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Inspect LEGR joint limits and one-joint-at-a-time action response."""

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Sweep one action dimension at a time and report joint response.")
parser.add_argument("--task", type=str, required=True, help="Name of the Isaac Lab task to inspect.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--steps", type=int, default=20, help="Number of simulation steps per action direction.")
parser.add_argument("--amplitude", type=float, default=0.8, help="Action amplitude for each single-joint sweep.")
parser.add_argument("--settle_steps", type=int, default=5, help="Zero-action steps before each joint sweep.")
parser.add_argument("--fix_root", action="store_true", default=False, help="Fix the root link during the sweep.")
parser.add_argument("--disable_gravity", action="store_true", default=False, help="Disable gravity on the robot during the sweep.")
parser.add_argument("--root_z", type=float, default=None, help="Override the robot default root height.")
parser.add_argument(
    "--report_body_deltas",
    action="store_true",
    default=False,
    help="Report selected body position changes for each swept joint.",
)
parser.add_argument(
    "--joint",
    type=str,
    default=None,
    help="Optional joint index or joint name to sweep. By default all joints are swept.",
)
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg


def _fmt(value: float) -> str:
    return f"{value: .4f}"


def _get_first_attr(obj, names: tuple[str, ...]):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def _print_joint_table(robot) -> None:
    joint_pos = robot.data.joint_pos[0].detach().cpu()
    joint_vel = robot.data.joint_vel[0].detach().cpu()
    default_pos = robot.data.default_joint_pos[0].detach().cpu()
    joint_limits = _get_first_attr(robot.data, ("soft_joint_pos_limits", "joint_pos_limits"))
    if joint_limits is not None:
        joint_limits = joint_limits[0].detach().cpu()

    print("\n[joints]")
    print("idx  name        default   current   velocity       lower     upper")
    for idx, name in enumerate(robot.joint_names):
        if joint_limits is None:
            lower_text = "   n/a"
            upper_text = "   n/a"
        else:
            lower_text = _fmt(joint_limits[idx, 0].item())
            upper_text = _fmt(joint_limits[idx, 1].item())
        print(
            f"{idx:>2}   {name:<10} {_fmt(default_pos[idx].item())} "
            f"{_fmt(joint_pos[idx].item())} {_fmt(joint_vel[idx].item())} "
            f"{lower_text} {upper_text}"
        )


def _step(env, actions, steps: int) -> None:
    for _ in range(steps):
        env.step(actions)


def _measure_joint_response(env, base_env, joint_idx: int, amplitude: float, steps: int, settle_steps: int) -> tuple[float, float]:
    action_dim = base_env.single_action_space.shape[0]
    zero_actions = torch.zeros((base_env.num_envs, action_dim), device=base_env.device, dtype=torch.float32)

    def run_direction(direction: float) -> float:
        env.reset()
        _step(env, zero_actions, settle_steps)
        start = base_env.scene["robot"].data.joint_pos[0, joint_idx].item()
        actions = zero_actions.clone()
        actions[:, joint_idx] = direction * amplitude
        _step(env, actions, steps)
        end = base_env.scene["robot"].data.joint_pos[0, joint_idx].item()
        return end - start

    return run_direction(1.0), run_direction(-1.0)


def _body_delta_response(
    env, base_env, joint_idx: int, amplitude: float, steps: int, settle_steps: int
) -> dict[str, torch.Tensor]:
    action_dim = base_env.single_action_space.shape[0]
    robot = base_env.scene["robot"]
    body_names = ["Calf_L", "Calf_R", "Foot_L", "Foot_R"]
    body_ids = [robot.body_names.index(name) for name in body_names]
    zero_actions = torch.zeros((base_env.num_envs, action_dim), device=base_env.device, dtype=torch.float32)

    env.reset()
    _step(env, zero_actions, settle_steps)
    start = robot.data.body_pos_w[0, body_ids, :].clone()
    actions = zero_actions.clone()
    actions[:, joint_idx] = amplitude
    _step(env, actions, steps)
    end = robot.data.body_pos_w[0, body_ids, :].clone()
    return {name: delta.detach().cpu() for name, delta in zip(body_names, end - start)}


def _joint_indices(robot, action_dim: int) -> list[int]:
    sweep_count = min(action_dim, len(robot.joint_names))
    if args_cli.joint is None:
        return list(range(sweep_count))
    try:
        joint_idx = int(args_cli.joint)
    except ValueError:
        if args_cli.joint not in robot.joint_names:
            raise ValueError(f"Unknown joint name: {args_cli.joint}. Known joints: {robot.joint_names}")
        joint_idx = robot.joint_names.index(args_cli.joint)
    if joint_idx < 0 or joint_idx >= sweep_count:
        raise ValueError(f"Joint index {joint_idx} is outside sweepable range 0..{sweep_count - 1}.")
    return [joint_idx]


def main() -> None:
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    if args_cli.fix_root:
        env_cfg.scene.robot.spawn.articulation_props.fix_root_link = True
    if args_cli.disable_gravity:
        env_cfg.scene.robot.spawn.rigid_props.disable_gravity = True
    if args_cli.root_z is not None:
        x, y, _ = env_cfg.scene.robot.init_state.pos
        env_cfg.scene.robot.init_state.pos = (x, y, args_cli.root_z)

    env = gym.make(args_cli.task, cfg=env_cfg)
    base_env = env.unwrapped
    robot = base_env.scene["robot"]

    if base_env.num_envs != 1:
        raise ValueError("This diagnostic expects --num_envs 1 so the output is readable.")

    action_dim = base_env.single_action_space.shape[0]
    print(f"task: {args_cli.task}", flush=True)
    print(f"num_envs: {base_env.num_envs}", flush=True)
    print(f"action_dim: {action_dim}", flush=True)
    print(f"joint_count: {len(robot.joint_names)}", flush=True)
    print(f"joint_names: {robot.joint_names}", flush=True)
    print(f"body_names: {robot.body_names}", flush=True)
    print(f"configured_action_scale: {base_env.cfg.actions.joint_pos.scale}", flush=True)

    env.reset()
    _print_joint_table(robot)

    if action_dim != len(robot.joint_names):
        print(
            "\n[warning] action_dim does not match joint_count. "
            "The sweep assumes the action order matches robot.joint_names.",
            flush=True,
        )

    sweep_indices = _joint_indices(robot, action_dim)
    print("\n[action_response]")
    print("idx  joint       +action_delta  -action_delta  quick_read")
    for joint_idx in sweep_indices:
        positive_delta, negative_delta = _measure_joint_response(
            env, base_env, joint_idx, args_cli.amplitude, args_cli.steps, args_cli.settle_steps
        )
        responsive = abs(positive_delta) > 1.0e-3 or abs(negative_delta) > 1.0e-3
        opposite = positive_delta * negative_delta < 0.0
        if not responsive:
            read = "NO MOTION"
        elif not opposite:
            read = "same-sign/limit"
        else:
            read = "moves both ways"
        print(
            f"{joint_idx:>2}   {robot.joint_names[joint_idx]:<10} "
            f"{positive_delta:>13.5f} {negative_delta:>14.5f}  {read}",
            flush=True,
        )
        if args_cli.report_body_deltas:
            deltas = _body_delta_response(env, base_env, joint_idx, args_cli.amplitude, args_cli.steps, args_cli.settle_steps)
            print("     body_delta(+action) dx/dy/dz:", flush=True)
            for body_name, delta in deltas.items():
                print(
                    f"       {body_name:<8} "
                    f"{delta[0].item():>8.4f} {delta[1].item():>8.4f} {delta[2].item():>8.4f}",
                    flush=True,
                )

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
