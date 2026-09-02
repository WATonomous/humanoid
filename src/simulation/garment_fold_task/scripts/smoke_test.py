"""Smoke test: build Humanoid-GarmentFold-Bimanual-Pioneer-v0, reset, screenshot.

    isaaclab.sh -p scripts/smoke_test.py --garment Top_Long_Seen_1 --out /tmp/gf
"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--garment", default="Top_Long_Seen_1")
parser.add_argument("--out", default="/tmp/gf_pioneer")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True

app = AppLauncher(args).app

import os
import numpy as np
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

# aim the dev scene cam at the robot + table
try:
    import torch
    env.scene_camera.set_world_poses_from_view(
        eyes=torch.tensor([[1.9, -2.3, 1.35]], device=env.device),
        targets=torch.tensor([[0.0, -0.1, 0.55]], device=env.device),
    )
except Exception as e:
    print("scene cam aim failed:", e)

# let the NuRec backdrop / lighting warm up
for _ in range(150):
    env.sim.step(render=True)

os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
for suf, cam in [("_top.png", "top_camera"), ("_scene.png", "scene_camera"),
                 ("_lwrist.png", "left_camera"), ("_rwrist.png", "right_camera")]:
    try:
        a = np.asarray(env.__getattribute__(cam).data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a[..., :3].astype(np.uint8)).save(args.out + suf)
        print("saved", args.out + suf)
    except Exception as e:
        print(cam, "failed:", e)

print("SMOKE_OK")
os._exit(0)
