# Copyright (c) 2024-2025, Muammer Bay (LycheeAI), Louis Le Lay
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Observation terms for the push-block task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import subtract_frame_transforms

from .utils import block_center_w

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def block_center_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Position of the block's geometric center in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    center_w = block_center_w(object)
    center_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], center_w
    )
    return center_b


def block_orientation(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Block root orientation quaternion (w, x, y, z) in the world frame."""
    object: RigidObject = env.scene[object_cfg.name]
    return object.data.root_quat_w


def block_lin_vel(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Block root linear velocity in the world frame."""
    object: RigidObject = env.scene[object_cfg.name]
    return object.data.root_lin_vel_w


def ee_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """End-effector position in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    ee_b, _ = subtract_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], ee_w)
    return ee_b


def ramp_geometry(
    env: ManagerBasedRLEnv,
    ramp_base_x: float,
    ramp_top_x: float,
    floor_z: float,
    target: tuple[float, float],
) -> torch.Tensor:
    """Constant ramp/box geometry in the env frame: ramp base x, ramp top x,
    interior floor height, floor target point (xy) and the up-the-ramp
    direction vector (xy)."""
    vals = torch.tensor(
        [ramp_base_x, ramp_top_x, floor_z, target[0], target[1], 1.0, 0.0],
        device=env.device,
    )
    return vals.unsqueeze(0).expand(env.num_envs, -1)
