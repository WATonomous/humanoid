# Copyright (c) 2024-2025, Muammer Bay (LycheeAI), Louis Le Lay
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Termination terms for the push-block task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

from .utils import block_center_w

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

__all__ = ["block_off_course"]


def block_off_course(
    env: ManagerBasedRLEnv,
    x_min: float,
    x_max: float,
    y_limit: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Terminate when the block is pushed off the approach corridor: too far
    sideways, knocked back toward the robot, or past the box."""
    object: RigidObject = env.scene[object_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    return (center[:, 0] < x_min) | (center[:, 0] > x_max) | (center[:, 1].abs() > y_limit)
