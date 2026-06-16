import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

_MCP_A_JOINT_NAMES = ("MCP_A_1", "MCP_A_2", "MCP_A_3", "MCP_A_4")

if TYPE_CHECKING:
    from .commands import InHandReOrientationCommand


def success_bonus(
    env: ManagerBasedRLEnv, command_name: str, object_cfg: SceneEntityCfg = SceneEntityCfg("cube")
) -> torch.Tensor:
    """Bonus reward for successfully reaching the goal.

    The object is considered to have reached the goal when the object orientation is within the threshold.
    The reward is 1.0 if the object has reached the goal, otherwise 0.0.

    Args:
        env: The environment object.
        command_name: The command term to be used for extracting the goal.
        object_cfg: The configuration for the scene entity. Default is "cube".
    """
    # extract useful elements
    asset: RigidObject = env.scene[object_cfg.name]
    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)

    # obtain the goal orientation
    goal_quat_w = command_term.command[:, 3:7]
    # obtain the threshold for the orientation error
    threshold = command_term.cfg.orientation_success_threshold
    # calculate the orientation error
    dtheta = math_utils.quat_error_magnitude(asset.data.root_quat_w, goal_quat_w)

    return dtheta <= threshold


def track_pos_l2(
    env: ManagerBasedRLEnv, command_name: str, object_cfg: SceneEntityCfg = SceneEntityCfg("cube")
) -> torch.Tensor:
    """Reward for tracking the object position using the L2 norm.

    The reward is the distance between the object position and the goal position.

    Args:
        env: The environment object.
        command_term: The command term to be used for extracting the goal.
        object_cfg: The configuration for the scene entity. Default is "cube".
    """
    # extract useful elements
    asset: RigidObject = env.scene[object_cfg.name]
    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)

    # obtain the goal position
    goal_pos_e = command_term.command[:, 0:3]
    # obtain the object position in the environment frame
    object_pos_e = asset.data.root_pos_w - env.scene.env_origins

    return torch.norm(goal_pos_e - object_pos_e, p=2, dim=-1)


def track_orientation_inv_l2(
    env: ManagerBasedRLEnv,
    command_name: str,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    rot_eps: float = 1e-3,
) -> torch.Tensor:
    """Reward for tracking the object orientation using the inverse of the orientation error.

    The reward is the inverse of the orientation error between the object orientation and the goal orientation.

    Args:
        env: The environment object.
        command_name: The command term to be used for extracting the goal.
        object_cfg: The configuration for the scene entity. Default is "cube".
        rot_eps: The threshold for the orientation error. Default is 1e-3.
    """
    asset: RigidObject = env.scene[object_cfg.name]
    command_term: InHandReOrientationCommand = env.command_manager.get_term(command_name)

    # obtain the goal orientation
    goal_quat_w = command_term.command[:, 3:7]
    dtheta = math_utils.quat_error_magnitude(asset.data.root_quat_w, goal_quat_w)

    return 1.0 / (dtheta + rot_eps)


def object_ang_vel_toward_goal(
    env: ManagerBasedRLEnv,
    command_name: str,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward the component of object angular velocity that reduces orientation error.

    Positive when the cube is spinning toward the goal orientation, negative when
    spinning away. Gives an immediate per-step gradient for rotation attempts rather
    than waiting for the orientation error metric to actually change.
    """
    asset: RigidObject = env.scene[object_cfg.name]
    command_term: "InHandReOrientationCommand" = env.command_manager.get_term(command_name)

    goal_quat_w = command_term.command[:, 3:7]
    curr_quat_w = asset.data.root_quat_w
    ang_vel_w = asset.data.root_ang_vel_w  # (N, 3)

    # q_err = q_goal * conj(q_curr): quaternion to rotate from current → goal.
    # Its imaginary part gives the world-frame axis one should rotate about, scaled
    # by sin(θ/2) — large and well-conditioned at the ~2 rad errors seen early in training.
    q_err = math_utils.quat_mul(goal_quat_w, math_utils.quat_conjugate(curr_quat_w))
    err_axis_w = q_err[:, 1:]  # (N, 3)

    # Clamp to zero: only reward spinning *toward* goal, never penalise spinning away.
    # Without the clamp, negative values teach the policy to suppress all rotation.
    return torch.sum(ang_vel_w * err_axis_w, dim=-1).clamp(min=0.0)


def object_held_bonus(
    env: ManagerBasedRLEnv,
    command_name: str,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    hold_threshold: float = 0.10,
) -> torch.Tensor:
    """Per-step bonus (1.0) when object is within hold_threshold of the goal position.

    Provides a dense holding signal that is zero when the cube has drifted away and 1.0
    when it is within reach — much stronger gradient than the continuous L2 penalty.
    """
    asset: RigidObject = env.scene[object_cfg.name]
    command_term: "InHandReOrientationCommand" = env.command_manager.get_term(command_name)
    goal_pos_e = command_term.command[:, :3]
    object_pos_e = asset.data.root_pos_w - env.scene.env_origins
    dist = torch.norm(goal_pos_e - object_pos_e, p=2, dim=-1)
    return (dist < hold_threshold).float()


def mcp_a_spread_activity(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    spread_limit: float = 0.15,
) -> torch.Tensor:
    """Reward finger abduction motion and deflection (encourages MCP_A spread use)."""
    robot: Articulation = env.scene[asset_cfg.name]
    spread_ids = []
    for name in _MCP_A_JOINT_NAMES:
        spread_ids.extend(robot.find_joints(name)[0])

    spread_vel = torch.linalg.norm(robot.data.joint_vel[:, spread_ids], dim=1)
    spread_pos = torch.mean(torch.abs(robot.data.joint_pos[:, spread_ids]), dim=1) / spread_limit
    return spread_vel + 0.5 * spread_pos
