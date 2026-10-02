"""Full-episode step test: build Humanoid-GarmentFold-Bimanual-Pioneer-v0, reset,
step it 600 times holding the default joint pose, and report whether the
success checker ever fires -- the README's top "still not done" item
(`smoke_test.py` only builds + resets, 30 steps, never ran a real episode).

    isaaclab.sh -p scripts/full_episode_test.py --garment Top_Long_Seen_1 --steps 600 --out /tmp/gf_ep
"""
import argparse
import sys
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--garment", default="Top_Long_Seen_1")
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--out", default="/tmp/gf_ep")
parser.add_argument("--action_mode", choices=["home", "zero"], default="home",
                     help="home: hold default joint pose (no-op baseline). "
                          "zero: send all-zero joint targets (sanity check only).")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True

app = AppLauncher(args).app

import os
import numpy as np
import torch
import gymnasium as gym
from PIL import Image
from isaaclab_tasks.utils import parse_env_cfg
import humanoid_garment_fold  # noqa: F401  registers the env

cfg = parse_env_cfg("Humanoid-GarmentFold-Bimanual-Pioneer-v0", device="cpu")
cfg.sim.use_fabric = False
cfg.garment_name = args.garment

env = gym.make("Humanoid-GarmentFold-Bimanual-Pioneer-v0", cfg=cfg).unwrapped
for _ in range(30):
    env.sim.step(render=True)
env.initialize_obs()
for _ in range(20):
    env.sim.step(render=True)
env.reset()

os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)


def save_frame(suffix: str) -> None:
    try:
        a = np.asarray(env.top_camera.data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a[..., :3].astype(np.uint8)).save(f"{args.out}{suffix}")
    except Exception as e:
        print(f"frame save ({suffix}) failed:", e)


if args.action_mode == "home":
    action = torch.cat([env.robot.data.default_joint_pos[:, env._l_arm],
                         env.robot.data.default_joint_pos[:, env._r_arm]], dim=-1)
else:
    action = torch.zeros((1, 12), device=env.device)

save_frame("_step0000.png")
t0 = time.perf_counter()
first_success_step = None
for st in range(args.steps):
    env.step(action)
    # _get_success's type hint claims a (success, extra) tuple, but it actually
    # returns a single per-env bool tensor (see garment_env.py).
    success = env._get_success()
    if success.item() and first_success_step is None:
        first_success_step = st
        save_frame(f"_success_step{st:04d}.png")
    if (st + 1) % 100 == 0:
        print(f"PROGRESS step={st + 1}/{args.steps} elapsed={time.perf_counter() - t0:.1f}s "
              f"success_so_far={first_success_step is not None}")
        save_frame(f"_step{st + 1:04d}.png")

save_frame(f"_step{args.steps:04d}_final.png")
elapsed = time.perf_counter() - t0
print(f"FULL_EPISODE_DONE steps={args.steps} elapsed={elapsed:.1f}s "
      f"s_per_step={elapsed / args.steps:.3f} first_success_step={first_success_step}")
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
