"""Custom observations for the pick-and-place task."""
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_pose_in_robot_root_frame(
    env: "ManagerBasedRLEnv",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Object pose (pos 3 + quat wxyz 4) in the robot root frame."""
    robot = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    pos_b, quat_b = subtract_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w,
        obj.data.root_pos_w, obj.data.root_quat_w,
    )
    return torch.cat([pos_b, quat_b], dim=-1)


def object_grasped(
    env: "ManagerBasedRLEnv",
    lift_height_threshold: float,
    gripper_closed_frac: float = 0.25,
    table_top_z: float = 0.05,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    gripper_joint_names: tuple = ("joint7l", "joint8l"),
    gripper_closed_targets: tuple = (-0.05, 0.05),
) -> torch.Tensor:
    """True once the object is lifted clear of the table while the gripper is
    closed -- proxy for the orchestrator (pick_place_gen/orchestrator.py)
    reaching its post-LIFT phase. Mimic-only: used as the "grasp" subtask
    termination signal (see pick_place_bimanual_mimic_env.py); not wired into
    the base task's ObservationsCfg.
    """
    robot = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]

    joint_names = list(robot.data.joint_names)
    grip_ids = [joint_names.index(n) for n in gripper_joint_names]
    grip_pos = robot.data.joint_pos[:, grip_ids]
    closed_targets = torch.tensor(gripper_closed_targets, device=env.device, dtype=grip_pos.dtype)
    # Fraction each finger has travelled from open (0) toward its fully-closed
    # target. Gripping an object stops the fingers partway (they contact the
    # object before reaching the fully-closed target), so requiring them to
    # reach the target within a small tolerance never fires. Instead require
    # each finger to have closed at least `gripper_closed_frac` of the way,
    # which is robust to object width. (Measured over the recorded demos'
    # lifted window: the binding finger closes only to 0.34-0.53 of the
    # target, so the default 0.25 sits safely below that with margin for
    # replay divergence while still requiring a clear grip.)
    closed_frac = grip_pos / closed_targets
    gripper_closed = (closed_frac >= gripper_closed_frac).all(dim=-1)

    obj_z = obj.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    lifted = obj_z > (table_top_z + lift_height_threshold)

    return (gripper_closed & lifted).unsqueeze(-1)
