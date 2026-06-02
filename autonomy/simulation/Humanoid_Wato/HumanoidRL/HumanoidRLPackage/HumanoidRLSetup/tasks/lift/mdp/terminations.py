<<<<<<< HEAD
<<<<<<< HEAD
=======
"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

>>>>>>> eff69ae8 (refine-rl-and-add-rl-env)
=======
>>>>>>> 0b589f1c (lerobot-pick-rl)
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_reached_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Terminate if object reaches the target pos alr

    Args:
        threshold: The threshold for the object to be considered reaching the goal position. Defaults to 0.02.

    """
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)

    des_pos_b = command[:, :3]
    # convert to world frame
    des_pos_w, _ = combine_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)
    curr_pos_w = object.data.root_pos_w[:, :3]

    distance = torch.norm(des_pos_w - curr_pos_w, dim=1)

    return distance < threshold
