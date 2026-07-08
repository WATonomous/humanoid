"""Functions specific to the in-hand dexterous manipulation environments."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from .commands import InHandReOrientationCommand


def max_consecutive_success(env: ManagerBasedRLEnv, num_success: int, command_name: str) -> torch.Tensor:
    """Check if the task has been completed consecutively for a certain number of times.

    Args:
        env: The environment object.
        num_success: Threshold for the number of consecutive successes required.
        command_name: The command term to be used for extracting the goal.
    """
    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)

    return command_term.metrics["consecutive_success"] >= num_success


def object_away_from_goal(
    env: ManagerBasedRLEnv,
    threshold: float,
    command_name: str,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Check if object has gone far from the goal.

    The object is considered to be out-of-reach if the distance between the goal and the object is greater
    than the threshold.

    Args:
        env: The environment object.
        threshold: The threshold for the distance between the robot and the object.
        command_name: The command term to be used for extracting the goal.
        object_cfg: The configuration for the scene entity. Default is "cube".
    """
    # extract useful elements
    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)
    asset = env.scene[object_cfg.name]

    # object pos
    asset_pos_e = asset.data.root_pos_w - env.scene.env_origins
    goal_pos_e = command_term.command[:, :3]

    return torch.norm(asset_pos_e - goal_pos_e, p=2, dim=1) > threshold


def object_away_from_robot(
    env: ManagerBasedRLEnv,
    threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Check if object has gone far from the robot.

    The object is considered to be out-of-reach if the distance between the robot and the object is greater
    than the threshold.

    Args:
        env: The environment object.
        threshold: The threshold for the distance between the robot and the object.
        asset_cfg: The configuration for the robot entity. Default is "robot".
        object_cfg: The configuration for the object entity. Default is "cube".
    """
    # extract useful elements
    robot = env.scene[asset_cfg.name]
    object = env.scene[object_cfg.name]

    # compute distance
    dist = torch.norm(robot.data.root_pos_w - object.data.root_pos_w, dim=1)

    return dist > threshold


def orientation_stagnation(
    env: ManagerBasedRLEnv,
    command_name: str,
    orientation_error_threshold: float,
    stagnant_steps: int,
    object_ang_vel_threshold: float | None = None,
    joint_vel_threshold: float | None = None,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Terminate when orientation error stays above threshold for too many steps.

    Velocity gates are optional. Clamped grasps often keep joint velocities above a
    tight threshold due to actuator tracking, so orientation-only detection is the default.
    """
    if not hasattr(env, "_orientation_stagnant_counter"):
        env._orientation_stagnant_counter = torch.zeros(env.num_envs, device=env.device, dtype=torch.int32)

    counter = env._orientation_stagnant_counter
    # episode_length_buf is incremented before termination compute; value 1 means a fresh episode.
    counter[env.episode_length_buf == 1] = 0

    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)
    is_stagnant = command_term.metrics["orientation_error"] > orientation_error_threshold

    if object_ang_vel_threshold is not None:
        object_asset: RigidObject = env.scene[object_cfg.name]
        object_ang_vel = torch.linalg.norm(object_asset.data.root_ang_vel_w, dim=1)
        is_stagnant = is_stagnant & (object_ang_vel < object_ang_vel_threshold)

    if joint_vel_threshold is not None:
        robot: Articulation = env.scene[robot_cfg.name]
        joint_vel = torch.linalg.norm(robot.data.joint_vel, dim=1)
        is_stagnant = is_stagnant & (joint_vel < joint_vel_threshold)

    counter[:] = torch.where(is_stagnant, counter + 1, torch.zeros_like(counter))

    return counter >= stagnant_steps
