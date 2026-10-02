"""Is a target actually within the arm's reach? Random-samples joint configs
within the arm's real joint limits and checks the closest any of them gets to
--goal via forward kinematics -- no IK involved, so it isolates "is this a
kinematic limit" from "is the IK solver failing." Used to confirm
reach_pose_ik.py's plateau was a genuine reach limit, then to find a
robot_base_pos that actually works (see git history / garment_pioneer_cfg.py).

    isaaclab.sh -p scripts/fk_reach_check.py --goal 0.04 0.01 0.73 --n 800
"""
import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--garment", default="Top_Long_Seen_1")
parser.add_argument("--n", type=int, default=800, help="random joint configs to try")
parser.add_argument("--goal", nargs=3, type=float, default=[0.04, 0.01, 0.73],
                     help="world-frame target, default = the garment's own mesh center")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True

app = AppLauncher(args).app

import os
import random
import torch
import gymnasium as gym
from isaaclab.utils.math import subtract_frame_transforms
from isaaclab_tasks.utils import parse_env_cfg
import humanoid_garment_fold  # noqa: F401
from pioneer_humanoid.bimanual_arm import (
    LEFT_EE_BODY, LEFT_FINGER_TIP_BODIES, compute_gripper_tip_pose_b,
    resolve_body_ids, resolve_joint_name,
)
from pioneer_humanoid.arm_params import LEFT_ARM_JOINTS

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

robot = env.robot
device = env.device
names = [resolve_joint_name(robot, n) for n in LEFT_ARM_JOINTS]
joint_ids = [robot.data.joint_names.index(n) for n in names]
finger_ids = resolve_body_ids(robot, LEFT_FINGER_TIP_BODIES)
body_id = [i for i, n in enumerate(robot.data.body_names) if n == LEFT_EE_BODY][0]

lims = robot.data.joint_pos_limits[0, joint_ids]
print("JOINT_LIMITS", lims.tolist())

root_pose_w = robot.data.root_state_w[:, 0:7]
goal_w = torch.tensor([args.goal], device=device)
goal_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], goal_w)

sim_dt = env.sim.get_physics_dt()
best = {"dist": 1e9, "q": None}
random.seed(0)
for i in range(args.n):
    q = torch.tensor([[random.uniform(float(lims[j, 0]), float(lims[j, 1])) for j in range(6)]], device=device)
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone().index_copy_(1, torch.tensor(joint_ids), q),
        robot.data.default_joint_vel.clone(),
    )
    env.scene.write_data_to_sim()
    env.sim.step(render=False)
    env.scene.update(sim_dt)
    root_pose_w = robot.data.root_state_w[:, 0:7]
    tip_pos_b, _ = compute_gripper_tip_pose_b(robot, root_pose_w, body_id, finger_ids)
    d = (tip_pos_b - goal_b).norm().item()
    if d < best["dist"]:
        best["dist"] = d
        best["q"] = q.squeeze(0).tolist()
    if i % 500 == 0:
        print(f"PROGRESS i={i} best_dist_so_far={best['dist']:.4f}")

print(f"FK_BEST_DIST {best['dist']:.4f}")
print(f"FK_BEST_Q {best['q']}")
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
