"""Reach-pose attempt for the "fold-ready joint pose" TODO. Drives both
grippers toward fixed world-frame targets above the garment, reusing
src/teleop/keyboard_teleop/keyboard_teleop.py's own driving logic verbatim
(persistent target, integrated from small per-step deltas, leashed to stay
within _MAX_LEAD_M of the tip's real current position every step) instead of
writing a new IK loop -- earlier from-scratch attempts (one big target jump,
then a time-based ramp) caused real instability: wrist wind-up, diverging
error. The leash fixes that.

    isaaclab.sh -p scripts/reach_pose_ik.py --garment Top_Long_Seen_1 \
        --lx -0.04 --rx 0.12 --y 0.01 --z 0.73 --steps 400 --out /tmp/reach

STATUS (2026-10-02): solved. The leash alone (first pass) plateaued at
~0.10-0.24m error regardless of target tried, including the garment's exact
measured position -- not an aiming problem. scripts/fk_reach_check.py (pure
forward-kinematics sampling, no IK) confirmed it was a real reach limit at
the old robot_base_pos, then found one that isn't: with the
GarmentPioneerEnvCfg.robot_base_pos fix (see git history), this now
converges to ~1-5mm error.
"""
import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--garment", default="Top_Long_Seen_1")
parser.add_argument("--steps", type=int, default=400)
parser.add_argument("--out", default="/tmp/reach")
# target centerpoint offsets, world frame
parser.add_argument("--lx", type=float, default=-0.10, help="left gripper target world X")
parser.add_argument("--rx", type=float, default=0.10, help="right gripper target world X")
parser.add_argument("--y", type=float, default=0.0, help="both grippers' target world Y")
parser.add_argument("--z", type=float, default=0.80, help="both grippers' target world Z")
# per-step "held key" speed, same units/scale as keyboard_teleop.py's Se3Keyboard command
parser.add_argument("--speed", type=float, default=0.01, help="per-step position delta magnitude (m)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True

app = AppLauncher(args).app

import os
import torch
import numpy as np
import gymnasium as gym
from PIL import Image
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms, compute_pose_error
from isaaclab_tasks.utils import parse_env_cfg
import humanoid_garment_fold  # noqa: F401
from pioneer_humanoid.bimanual_arm import (
    LEFT_EE_BODY, RIGHT_EE_BODY, LEFT_FINGER_TIP_BODIES, RIGHT_FINGER_TIP_BODIES,
    compute_gripper_tip_pose_b, compute_tip_ik_jacobian, resolve_body_ids, resolve_joint_name,
)
from pioneer_humanoid.arm_params import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS

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
os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

try:
    env.scene_camera.set_world_poses_from_view(
        eyes=torch.tensor([[1.9, -2.3, 1.35]], device=device),
        targets=torch.tensor([[0.0, -0.1, 0.55]], device=device),
    )
except Exception as e:
    print("scene cam aim failed:", e)


def save_frame(suffix):
    try:
        a = np.asarray(env.top_camera.data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a[..., :3].astype(np.uint8)).save(f"{args.out}{suffix}")
        a2 = np.asarray(env.left_camera.data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a2[..., :3].astype(np.uint8)).save(f"{args.out}_lwrist{suffix}")
        a3 = np.asarray(env.right_camera.data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a3[..., :3].astype(np.uint8)).save(f"{args.out}_rwrist{suffix}")
        a4 = np.asarray(env.scene_camera.data.output["rgb"][0].cpu().numpy())
        Image.fromarray(a4[..., :3].astype(np.uint8)).save(f"{args.out}_scene{suffix}")
    except Exception as e:
        print("frame save failed:", e)


# Same constants keyboard_teleop.py uses to leash the persistent target to the actual tip.
_MAX_LEAD_M = 0.06


class LeashedArmIK:
    """Port of keyboard_teleop.py's driving loop: a persistent target integrated
    from small per-step deltas, leashed to within _MAX_LEAD_M of the tip's
    actual current position every step. Orientation always = current tip
    orientation (no rotation commanded, matches keyboard_teleop.py idle).
    """

    def __init__(self, arm_joints, ee_body, finger_bodies, goal_pos_w):
        names = [resolve_joint_name(robot, n) for n in arm_joints]
        self.joint_ids = [robot.data.joint_names.index(n) for n in names]
        entity = SceneEntityCfg("robot", joint_names=names, body_names=[ee_body])
        entity.resolve(env.scene)
        self.body_id = entity.body_ids[0]
        self.jacobi_idx = self.body_id - 1 if robot.is_fixed_base else self.body_id
        self.finger_ids = resolve_body_ids(robot, finger_bodies)
        self.ctrl = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=1, device=device,
        )
        self.ctrl.reset(env_ids=torch.arange(1, device=device))
        self.goal_pos_w = goal_pos_w
        self.target_pos_b = None  # seeded from the tip on first step

    def step(self, speed):
        root_pose_w = robot.data.root_state_w[:, 0:7]
        root_pos, root_quat = root_pose_w[:, 0:3], root_pose_w[:, 3:7]
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, self.body_id, self.finger_ids
        )
        if self.target_pos_b is None:
            self.target_pos_b = tip_pos_b.clone()

        # "held key" delta: a fixed-size step toward the goal, same role as one
        # tick of a held W/S/A/D/Q/E key in keyboard_teleop.py.
        goal_pos_b, _ = subtract_frame_transforms(root_pos, root_quat, self.goal_pos_w)
        to_goal = goal_pos_b - self.target_pos_b
        dist = to_goal.norm()
        if float(dist) > 1e-6:
            step_vec = to_goal * min(speed / float(dist), 1.0)
        else:
            step_vec = torch.zeros_like(to_goal)
        self.target_pos_b = self.target_pos_b + step_vec

        # Leash to the tip's real current position -- verbatim from
        # keyboard_teleop.py (orientation part omitted: no rotation commanded here).
        pos_err, _ = compute_pose_error(
            tip_pos_b, tip_quat_b, self.target_pos_b, tip_quat_b, rot_error_type="axis_angle"
        )
        self.target_pos_b = tip_pos_b + pos_err.clamp(-_MAX_LEAD_M, _MAX_LEAD_M)

        self.ctrl.set_command(torch.cat([self.target_pos_b, tip_quat_b], dim=-1), ee_quat=tip_quat_b)
        ee_pos_w = robot.data.body_state_w[:, self.body_id, 0:3]
        ee_pos_b, _ = subtract_frame_transforms(root_pos, root_quat, ee_pos_w)
        jacobian = compute_tip_ik_jacobian(
            robot, robot.root_physx_view.get_jacobians()[:, self.jacobi_idx, :, self.joint_ids],
            ee_pos_b, tip_pos_b,
        )
        joint_pos = robot.data.joint_pos[:, self.joint_ids]
        joint_pos_des = self.ctrl.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=self.joint_ids)
        return tip_pos_b, goal_pos_b


left_goal_w = torch.tensor([[args.lx, args.y, args.z]], device=device)
right_goal_w = torch.tensor([[args.rx, args.y, args.z]], device=device)

left_ik = LeashedArmIK(LEFT_ARM_JOINTS, LEFT_EE_BODY, LEFT_FINGER_TIP_BODIES, left_goal_w)
right_ik = LeashedArmIK(RIGHT_ARM_JOINTS, RIGHT_EE_BODY, RIGHT_FINGER_TIP_BODIES, right_goal_w)

sim_dt = env.sim.get_physics_dt()
save_frame("_step0000.png")
for st in range(args.steps):
    lt, lg = left_ik.step(args.speed)
    rt, rg = right_ik.step(args.speed)
    # bypassing env.step() -- GarmentPioneerEnv._apply_action() would overwrite the
    # joint targets the IK controllers just set. Mirror keyboard_teleop.py's own
    # lower-level loop instead: push targets into the physics buffers, step, then
    # refresh robot.data.* from the new sim state (set_joint_position_target alone
    # writes a buffer that nothing applies without these two calls).
    env.scene.write_data_to_sim()
    env.sim.step(render=True)
    env.scene.update(sim_dt)
    if st % 50 == 0:
        l_err = (lg - lt).norm().item()
        r_err = (rg - rt).norm().item()
        print(f"DBG st={st} left_tip_b={lt.squeeze(0).tolist()} err_to_goal={l_err:.4f}")
        print(f"DBG st={st} right_tip_b={rt.squeeze(0).tolist()} err_to_goal={r_err:.4f}")
    if (st + 1) % 50 == 0:
        save_frame(f"_step{st + 1:04d}.png")

save_frame(f"_step{args.steps:04d}_final.png")
final_l = robot.data.joint_pos[:, left_ik.joint_ids].squeeze(0).tolist()
final_r = robot.data.joint_pos[:, right_ik.joint_ids].squeeze(0).tolist()
print(f"FINAL_LEFT_JOINTS {final_l}")
print(f"FINAL_RIGHT_JOINTS {final_r}")
print("REACH_IK_DONE")
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
