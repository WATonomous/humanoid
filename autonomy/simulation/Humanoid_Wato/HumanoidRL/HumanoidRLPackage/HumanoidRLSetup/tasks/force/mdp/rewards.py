from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import combine_frame_transforms, quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# Links allowed to establish contact for flexible pushing (arm + palm + fingers + thumb).
DEFAULT_CONTACT_BODY_NAMES = [
    "forearm_v8_.*",
    "elbow_link_1_v6_.*",
    "elbow_link_2_v7_.*",
    "PALM_GAVIN_1DoF_Hinge_v2_.*",
    "DIP_INDEX_v1_.*",
    "DIP_MIDDLE_v1_.*",
    "DIP_RING_v1_.*",
    "DIP_PINKY_v1_.*",
    "MCP_INDEX_v1_.*",
    "MCP_MIDDLE_v1_.*",
    "MCP_RING_v1_.*",
    "MCP_PINKY_v1_.*",
    "IP_THUMB_v1_.*",
    "MCP_THUMB_v1_.*",
]


def _parse_impulse_command(command: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Split impulse command tensor into position, force (base frame), and active flag."""
    return command[:, :3], command[:, 3:6], command[:, 6]


def _command_targets(
    env: ManagerBasedRLEnv, command_name: str, asset: Articulation
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    command = env.command_manager.get_command(command_name)
    des_pos_b, des_force_b, impulse_active = _parse_impulse_command(command)
    des_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b
    )
    des_force_w = quat_apply(asset.data.root_state_w[:, 3:7], des_force_b)
    return des_pos_w, des_force_w, impulse_active


def _contact_forces(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Net contact forces for the selected robot links. Shape: (N, B, 3)."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    return contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]  # type: ignore[index]


def _body_positions_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.body_state_w[:, asset_cfg.body_ids, :3]  # type: ignore[index]


def region_proximity_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Minimum distance from any candidate link to the target region center."""
    asset: Articulation = env.scene[asset_cfg.name]
    des_pos_w, _, _ = _command_targets(env, command_name, asset)
    body_pos_w = _body_positions_w(env, asset_cfg)
    dists = torch.norm(body_pos_w - des_pos_w.unsqueeze(1), dim=-1)
    return dists.min(dim=1).values


def region_proximity_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward bringing any candidate link close to the target region."""
    return 1.0 - torch.tanh(region_proximity_error(env, command_name, asset_cfg) / std)


def multi_link_contact_force_tracking(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    region_radius: float = 0.12,
    contact_threshold: float = 0.5,
) -> torch.Tensor:
    """Reward the best-matching wall contact force among links near the target region."""
    asset: Articulation = env.scene[asset_cfg.name]
    des_pos_w, des_force_w, impulse_active = _command_targets(env, command_name, asset)

    body_pos_w = _body_positions_w(env, asset_cfg)
    contact_forces = _contact_forces(env, sensor_cfg)

    dists = torch.norm(body_pos_w - des_pos_w.unsqueeze(1), dim=-1)
    in_region = (dists < region_radius).float()
    in_contact = (torch.norm(contact_forces, dim=-1) > contact_threshold).float()
    active = (impulse_active > 0.5).float()

    force_error = torch.norm(contact_forces - des_force_w.unsqueeze(1), dim=-1)
    link_reward = (1.0 - torch.tanh(force_error / std)) * in_region * in_contact
    best_reward = link_reward.max(dim=1).values
    return best_reward * active


def multi_link_contact_force_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    region_radius: float = 0.12,
    contact_threshold: float = 0.5,
) -> torch.Tensor:
    """Penalize force mismatch for the closest matching contact link in the target region."""
    asset: Articulation = env.scene[asset_cfg.name]
    des_pos_w, des_force_w, impulse_active = _command_targets(env, command_name, asset)

    body_pos_w = _body_positions_w(env, asset_cfg)
    contact_forces = _contact_forces(env, sensor_cfg)

    dists = torch.norm(body_pos_w - des_pos_w.unsqueeze(1), dim=-1)
    in_region = (dists < region_radius).float()
    in_contact = (torch.norm(contact_forces, dim=-1) > contact_threshold).float()
    active = (impulse_active > 0.5).float()

    force_error = torch.norm(contact_forces - des_force_w.unsqueeze(1), dim=-1)
    large = 1.0e3
    masked_error = torch.where(in_region * in_contact > 0.5, force_error, torch.full_like(force_error, large))
    best_error = masked_error.min(dim=1).values
    best_error = torch.where(best_error >= large, torch.zeros_like(best_error), best_error)
    return best_error * active
