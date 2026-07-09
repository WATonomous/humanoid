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
