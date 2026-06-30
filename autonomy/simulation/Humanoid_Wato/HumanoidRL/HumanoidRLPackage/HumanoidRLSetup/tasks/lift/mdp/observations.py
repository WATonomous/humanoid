from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import quat_apply, subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    
    # World frame position
    object_pos_w = object.data.root_pos_w[:, :3]
    # Convert to robot body frame (to body frame use subtract_frame_transform() instead of combine_frame_transform())
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w, object_pos_w)

    return object_pos_b


def ee_to_object(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Vector from controlled TCP to cube in world frame."""
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    return object.data.root_pos_w[:, :3] - ee_pos_w


def _finger_tip_positions_w(robot, fingertip_cfg: SceneEntityCfg, tip_local_offsets: list[tuple[float, float, float]]):
    body_pos = robot.data.body_pos_w[:, fingertip_cfg.body_ids, :]
    body_quat = robot.data.body_quat_w[:, fingertip_cfg.body_ids, :]
    offsets = torch.tensor(tip_local_offsets, device=body_pos.device, dtype=body_pos.dtype)
    offsets = offsets.unsqueeze(0).expand(body_pos.shape[0], -1, -1)
    tips = body_pos + quat_apply(body_quat.reshape(-1, 4), offsets.reshape(-1, 3)).reshape(body_pos.shape)
    return tips[:, 0, :], tips[:, 1, :]


def fingertip_center_to_object(
    env: ManagerBasedRLEnv,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Vector from midpoint between distal fingertips to cube."""
    robot = env.scene[fingertip_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    center = 0.5 * (tip_a + tip_b)
    return object.data.root_pos_w[:, :3] - center


def fingertips_to_object(
    env: ManagerBasedRLEnv,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Vectors from each distal fingertip to cube, concatenated as [tip_a_to_cube, tip_b_to_cube]."""
    robot = env.scene[fingertip_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    cube_pos = object.data.root_pos_w[:, :3]
    return torch.cat((cube_pos - tip_a, cube_pos - tip_b), dim=-1)


def gripper_opening(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    open_width: float,
) -> torch.Tensor:
    """Normalized gripper opening: 0 closed, 1 fully open."""
    robot = env.scene[asset_cfg.name]
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    if joint_pos.shape[1] < 2:
        return torch.zeros(joint_pos.shape[0], 1, device=joint_pos.device)
    opening = torch.abs(joint_pos[:, 0] - joint_pos[:, 1])
    return torch.clamp(opening / open_width, 0.0, 1.0).unsqueeze(-1)


def grasp_geometry(
    env: ManagerBasedRLEnv,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    gripper_cfg: SceneEntityCfg,
    open_width: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Seven compact grasp features: each fingertip-to-cube vector plus normalized gripper opening."""
    return torch.cat(
        (
            fingertips_to_object(env, fingertip_cfg, tip_local_offsets, object_cfg),
            gripper_opening(env, gripper_cfg, open_width),
        ),
        dim=-1,
    )
