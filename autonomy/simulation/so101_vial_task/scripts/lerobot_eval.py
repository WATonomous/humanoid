# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import argparse
import json
import random
from tqdm import tqdm

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="SO101 vial-task policy eval in Isaac Lab Gym env.")
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--num_envs", type=int, default=None, help="Number of environments to simulate."
)
parser.add_argument(
    "--task",
    type=str,
    default="Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval",
    help="Registered Gym task id (see humanoid_so101_vial_task.tasks).",
)
parser.add_argument("--seed", type=int, default=1984, help="Environment seed")
parser.add_argument("--num_episodes", type=int, default=10, help="Number of episodes to evaluate")
parser.add_argument(
    "--policy_type",
    type=str,
    choices=("lerobot", "groot", "custom_act"),
    default="lerobot",
    help="lerobot: LeRobot checkpoint dir; custom_act: humanoid-act best.pt; groot: GR00T server",
)
parser.add_argument(
    "--policy_path",
    type=str,
    default=None,
    help=(
        "Policy checkpoint path: LeRobot pretrained_model dir (--policy_type lerobot), "
        "or humanoid-act .pt file (--policy_type custom_act)"
    ),
)
parser.add_argument(
    "--rename_map",
    type=str,
    required=False,
    default=None,
    help=(
        'JSON mapping for renaming camera keys to match policy/feature config: key is simulation feature name, value is policy feature name '
        'e.g. \'{"sim_name1": "policy_name1", "sim_name2": "policy_name2"}\'. '
    ),
)
parser.add_argument("--policy_host", type=str, default="localhost", help="GR00T policy server host")
parser.add_argument("--policy_port", type=int, default=5555, help="GR00T policy server port")
parser.add_argument("--action_horizon", type=int, default=16, help="Number of action steps to execute per server query")
parser.add_argument(
    "--lang_instruction",
    type=str,
    default="Pick up the vial and place it in the rack",
    help="Language instruction for the policy",
)
parser.add_argument("--rerun", action="store_true", default=False, help="Enable Rerun visualization")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# always enable cameras to record video
args_cli.enable_cameras = True


# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import gymnasium as gym
import numpy as np
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import humanoid_so101_vial_task.tasks  # noqa: F401
from humanoid_so101_vial_task.utils.keyboard import KeyboardControl
from humanoid_act.eval import LocalCustomACTPolicy
from humanoid_so101_vial_task.utils.lerobot_interface import (
    LeRobotSO101Interface,
    GR00TRemotePolicy,
    LocalLeRobotPolicy,
)


def main():
    keyboard_control = KeyboardControl()

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric
    )
    env_cfg.seed = args_cli.seed

    # Seed all RNGs for reproducible episode resets
    random.seed(args_cli.seed)
    np.random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    torch.cuda.manual_seed_all(args_cli.seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    print(f"[INFO]: Click 'R' to reset the world")

    # cameras
    cameras = {}
    for obj in env.unwrapped.scene.keys():
        if obj.startswith("camera_"):
            camera_cfg = getattr(env.unwrapped.scene.cfg, obj)
            cameras[obj.replace("camera_", "")] = {
                "height": camera_cfg.height,
                "width": camera_cfg.width,
            }
            print(f"[INFO]: Found Camera: {obj.replace('camera_', '')}")
    if len(cameras) == 0:
        print(f"[Info]: No cameras found - videos will not be recorded")

    # lerobot interface (provides sim↔real coordinate transforms)
    rename_map = json.loads(args_cli.rename_map) if args_cli.rename_map else None
    robot_iface = LeRobotSO101Interface(
        device=env.unwrapped.device,
        port=None,
        id="leader_arm_1",
        cameras=cameras,
        fps=30,
        kind="follower",
        rename_map=rename_map,
    )
    print(f"[INFO]: Initializing device with Rerun visualization: {args_cli.rerun}")
    robot_iface.init_device(visualize=args_cli.rerun)

    if args_cli.policy_type == "groot":
        policy = GR00TRemotePolicy(
            robot_iface=robot_iface,
            host=args_cli.policy_host,
            port=args_cli.policy_port,
            action_horizon=args_cli.action_horizon,
            lang_instruction=args_cli.lang_instruction,
        )
        policy.connect()
    elif args_cli.policy_type == "custom_act":
        if not args_cli.policy_path:
            raise ValueError("--policy_path is required when --policy_type custom_act")
        if args_cli.rename_map:
            print("[WARNING] --rename_map is ignored for custom_act (use training camera names)")
        policy = LocalCustomACTPolicy(
            robot_iface=robot_iface,
            checkpoint_path=args_cli.policy_path,
            action_horizon=args_cli.action_horizon,
        )
        policy.connect()
    else:
        if not args_cli.policy_path:
            raise ValueError("--policy_path is required when --policy_type lerobot")
        policy = LocalLeRobotPolicy(
            robot_iface=robot_iface,
            policy_path=args_cli.policy_path,
            task_description=args_cli.lang_instruction,
        )
        policy.connect()

    # reset environment
    obs, _ = env.reset()
    policy.reset()

    # simulate environment
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    initial_action = torch.tensor(
        [-0.2736, -0.6109, -0.0745, 1.5148, -1.6034, -0.1465],
        device=env.unwrapped.device,
    )

    step = 0
    num_episodes = 0
    num_successes = 0
    success_rate = 0.0

    pbar = None

    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():

            if step == 0:
                pbar = tqdm(
                    total=env.unwrapped.max_episode_length,
                    desc=f"Rollout (ep {num_episodes + 1}, success: {success_rate:.1f}%)",
                    unit="step",
                    bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}]'
                )

            if step < 10:  # warmup, not critical but helps
                actions[:] = initial_action

            else:
                joint_positions = obs["policy"]["joint_pos_obs"][0].clone()
                actions[:] = policy.get_action(joint_positions, obs["visual"], log=True)

            obs, rewards, terminated, truncated, info = env.step(actions)

            step += 1

            # Update progress bar
            if pbar is not None:
                pbar.update(1)

            # Check for episode termination
            is_terminated = terminated.item() if terminated.numel() == 1 else terminated.any().item()
            is_truncated = truncated.item() if truncated.numel() == 1 else truncated.any().item()

            if is_terminated or is_truncated:
                if pbar is not None:
                    pbar.close()
                    pbar = None

                num_episodes += 1

                if is_terminated and not is_truncated:
                    num_successes += 1

                success_rate = (num_successes / num_episodes) * 100

                # Reset for next episode
                obs, _ = env.reset()
                policy.reset()
                step = 0

                continue

            # Manual reset with 'R' key
            if keyboard_control.reset_world:
                keyboard_control.reset_world = False
                if pbar is not None:
                    pbar.close()
                    pbar = None

                print(f"[MANUAL RESET] Episode interrupted at step {step}")
                obs, _ = env.reset()
                policy.reset()
                step = 0
                continue

            if num_episodes >= args_cli.num_episodes:
                # Close progress bar if still open
                if pbar is not None:
                    pbar.close()
                    pbar = None
                print(f"[INFO]: Evaluated {args_cli.num_episodes} episodes")
                print(f"[INFO]: Success Rate: {num_successes}/{args_cli.num_episodes} ({success_rate:.1f}%)")
                env.close()
                simulation_app.close()

    env.close()


if __name__ == "__main__":

    main()

    while True:
        simulation_app.update()

    simulation_app.close()
