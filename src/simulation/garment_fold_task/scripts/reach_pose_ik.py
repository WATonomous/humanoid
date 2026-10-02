"""Reach-pose IK diagnostic for the "fold-ready joint pose" TODO: drives both
grippers toward fixed world-frame targets above the garment using the same
differential-IK machinery src/teleop/task_space_controller/task_space_ik.py
already uses successfully for this exact robot (DifferentialIKController +
compute_tip_ik_jacobian/compute_gripper_tip_pose_b). Saves camera frames and
prints a position-error trace every 50 steps so progress can be checked both
visually and numerically, and prints the final joint angles reached.

    isaaclab.sh -p scripts/reach_pose_ik.py --garment Top_Long_Seen_1 \
        --lx -0.10 --rx 0.10 --y 0.0 --z 0.80 --steps 200 --out /tmp/reach

STATUS (checked 2026-10-02, not solved, updated after the robot_base_pos.z
fix -- see GarmentPioneerEnvCfg): originally, with the old (wrong) base
height, a target centered on the garment's table position was unreachable --
the position error flatlined hard (e.g. exactly 0.4154 for hundreds of
consecutive steps; a stuck local minimum, not slow convergence) for both
arms at every target distance tried, with final joint angles landing near
+-pi on the wrist joint.

After fixing robot_base_pos.z (the robot's own stand was more than half a
meter below the floor -- unrelated bug, found by inspection, fixed from the
asset's own USD bounding box, not guessed), that specific flatline symptom
is gone, but reaching still isn't solved: a *different* failure shows up at
two target heights tried -- the wrist joint winds up multiple radians and
the position error oscillates/diverges instead of converging, on both arms.

Ruled out one cause: the orientation command here is always the tip's
*current* orientation (meant as a no-op), re-set every step -- there's no
commanded orientation error by construction, so a quaternion double-cover
sign flip (q vs -q) isn't the direct cause, despite looking like one at
first (an earlier version of this note said otherwise; corrected).

Tried commanding the goal with a ramp -- ``--steps``/2 to interpolate
linearly from the arm's actual starting tip position to the goal, instead of
the full jump from step 0 (this is closer to how
``src/teleop/keyboard_teleop/keyboard_teleop.py`` actually drives this same
DifferentialIKController: one small per-keypress delta, never one big
target). This measurably helped the *approach*: error tracks the moving
sub-target cleanly (down to ~0.02-0.05) for roughly the first 65% of the
ramp. But instability still recurs once near the actual final target --
error grows again (up to ~0.87) specifically in that region, not from the
jump itself (there wasn't one this time). That points more precisely at the
target configuration itself being near a kinematic singularity for this
arm/pose (where damped-least-squares IK is known to blow up even with small
steps, since the Jacobian's condition number explodes near a singularity),
rather than purely a large-single-jump or redundant-DOF issue. Not
confirmed by computing the Jacobian's condition number directly -- the next
real step, before trying a different target region or a different IK
method/damping.
"""
import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--garment", default="Top_Long_Seen_1")
parser.add_argument("--steps", type=int, default=200)
parser.add_argument("--out", default="/tmp/reach")
# target centerpoint offsets, world frame (robot base sits at world (0,-0.63,0.68))
parser.add_argument("--lx", type=float, default=-0.10, help="left gripper target world X")
parser.add_argument("--rx", type=float, default=0.10, help="right gripper target world X")
parser.add_argument("--y", type=float, default=0.0, help="both grippers' target world Y")
parser.add_argument("--z", type=float, default=0.80, help="both grippers' target world Z")
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
from isaaclab.utils.math import subtract_frame_transforms
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


class ArmIK:
    def __init__(self, arm_joints, ee_body, finger_bodies):
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

    def step(self, target_pos_w, target_quat_b):
        root_pose_w = robot.data.root_state_w[:, 0:7]
        target_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], target_pos_w, target_quat_b
        )
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, self.body_id, self.finger_ids
        )
        self.ctrl.set_command(torch.cat([target_pos_b, tip_quat_b], dim=-1), ee_quat=tip_quat_b)
        ee_pos_w = robot.data.body_state_w[:, self.body_id, 0:3]
        ee_pos_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pos_w)
        jacobian = compute_tip_ik_jacobian(
            robot, robot.root_physx_view.get_jacobians()[:, self.jacobi_idx, :, self.joint_ids],
            ee_pos_b, tip_pos_b,
        )
        joint_pos = robot.data.joint_pos[:, self.joint_ids]
        joint_pos_des = self.ctrl.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=self.joint_ids)
        return tip_pos_b


left_ik = ArmIK(LEFT_ARM_JOINTS, LEFT_EE_BODY, LEFT_FINGER_TIP_BODIES)
right_ik = ArmIK(RIGHT_ARM_JOINTS, RIGHT_EE_BODY, RIGHT_FINGER_TIP_BODIES)

left_goal_w = torch.tensor([[args.lx, args.y, args.z]], device=device)
right_goal_w = torch.tensor([[args.rx, args.y, args.z]], device=device)
hold_quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=device)

# Commanding the full, far-off goal from step 0 (what this script originally did) let the
# wrist joint wind up a full 2*pi and the error diverge -- keyboard_teleop.py never does this;
# it only ever moves the IK target a small amount per keypress. Reproduce that here: ramp the
# commanded target linearly from the arm's actual starting tip position to the goal over the
# first half of the run, instead of a single big jump. Small per-step target deltas keep the
# solver away from the redundant-DOF (wrist-roll) drift a single large jump seems to trigger.
root_pose_w0 = robot.data.root_state_w[:, 0:7]
left_start_b, _ = compute_gripper_tip_pose_b(robot, root_pose_w0, left_ik.body_id, left_ik.finger_ids)
right_start_b, _ = compute_gripper_tip_pose_b(robot, root_pose_w0, right_ik.body_id, right_ik.finger_ids)
root_pos0, root_quat0 = root_pose_w0[:, 0:3], root_pose_w0[:, 3:7]
from isaaclab.utils.math import combine_frame_transforms
left_start_w, _ = combine_frame_transforms(root_pos0, root_quat0, left_start_b)
right_start_w, _ = combine_frame_transforms(root_pos0, root_quat0, right_start_b)
ramp_steps = max(1, args.steps // 2)

sim_dt = env.sim.get_physics_dt()
save_frame("_step0000.png")
for st in range(args.steps):
    frac = min(1.0, (st + 1) / ramp_steps)
    left_target_w = left_start_w + frac * (left_goal_w - left_start_w)
    right_target_w = right_start_w + frac * (right_goal_w - right_start_w)
    lt = left_ik.step(left_target_w, hold_quat)
    rt = right_ik.step(right_target_w, hold_quat)
    # bypassing env.step() -- GarmentPioneerEnv._apply_action() would overwrite
    # the joint targets the IK controllers just set with its own `self.actions`.
    # Mirror task_space_ik.py's lower-level loop instead: push targets into the
    # physics buffers, step, then refresh robot.data.* from the new sim state.
    # (Calling env.sim.step() alone, as an earlier version of this script did,
    # silently never moves anything -- set_joint_position_target only writes a
    # buffer; nothing pushes it into physics or refreshes the read-back tensors
    # without these two calls.)
    env.scene.write_data_to_sim()
    env.sim.step(render=True)
    env.scene.update(sim_dt)
    if st % 50 == 0:
        root_pose_w = robot.data.root_state_w[:, 0:7]
        lp_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], left_target_w, hold_quat)
        rp_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], right_target_w, hold_quat)
        lg_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], left_goal_w, hold_quat)
        rg_b, _ = subtract_frame_transforms(root_pose_w[:, 0:3], root_pose_w[:, 3:7], right_goal_w, hold_quat)
        l_err = (lp_b - lt).norm().item()
        r_err = (rp_b - rt).norm().item()
        l_goal_err = (lg_b - lt).norm().item()
        r_goal_err = (rg_b - rt).norm().item()
        print(f"DBG st={st} frac={frac:.3f} left_tip_b={lt.squeeze(0).tolist()} err_to_subtarget={l_err:.4f} err_to_FINAL_GOAL={l_goal_err:.4f}")
        print(f"DBG st={st} frac={frac:.3f} right_tip_b={rt.squeeze(0).tolist()} err_to_subtarget={r_err:.4f} err_to_FINAL_GOAL={r_goal_err:.4f}")
    if (st + 1) % 25 == 0:
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
