"""Shared EE tracking errors for intercept command metrics and rewards."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_apply, quat_error_magnitude, quat_mul

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.intercept_layout import (
    COMMAND_HIT_SLICE,
    COMMAND_POS_SLICE,
    COMMAND_QUAT_SLICE,
    COMMAND_TIME_SLICE,
    COMMAND_VEL_SLICE,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

DEFAULT_RACKET_BODY_NAMES = [
    "forearm_v8_.*",
    "DIP_INDEX_v1_.*",
]


def parse_intercept_command(
    command: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Split privileged intercept command into pose, velocity, hit pulse, and time-to-hit."""
    return (
        command[:, COMMAND_POS_SLICE],
        command[:, COMMAND_QUAT_SLICE],
        command[:, COMMAND_VEL_SLICE],
        command[:, COMMAND_HIT_SLICE],
        command[:, COMMAND_TIME_SLICE],
    )


def command_targets_w(
    command: torch.Tensor, asset: Articulation
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """World-frame position, orientation, velocity, hit pulse, and lead time."""
    des_pos_b, des_quat_b, des_vel_b, hit_moment_active, lead_time_left = parse_intercept_command(command)
    des_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b
    )
    des_quat_w = quat_mul(asset.data.root_state_w[:, 3:7], des_quat_b)
    des_vel_w = quat_apply(asset.data.root_state_w[:, 3:7], des_vel_b)
    return des_pos_w, des_quat_w, des_vel_w, hit_moment_active, lead_time_left


def _body_positions_w(asset: Articulation, body_ids: list[int]) -> torch.Tensor:
    return asset.data.body_state_w[:, body_ids, :3]


def _body_orientations_w(asset: Articulation, body_ids: list[int]) -> torch.Tensor:
    return asset.data.body_state_w[:, body_ids, 3:7]


def _body_lin_vel_w(asset: Articulation, body_ids: list[int]) -> torch.Tensor:
    return asset.data.body_lin_vel_w[:, body_ids, :]


def best_racket_tracking_errors(
    command: torch.Tensor,
    asset: Articulation,
    body_ids: list[int],
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Position, orientation, and velocity errors for the closest racket proxy link."""
    des_pos_w, des_quat_w, des_vel_w, _, _ = command_targets_w(command, asset)
    body_pos_w = _body_positions_w(asset, body_ids)
    body_quat_w = _body_orientations_w(asset, body_ids)
    body_vel_w = _body_lin_vel_w(asset, body_ids)

    dists = torch.norm(body_pos_w - des_pos_w.unsqueeze(1), dim=-1)
    body_idx = dists.argmin(dim=1)
    env_ids = torch.arange(command.shape[0], device=command.device)

    pos_err = dists[env_ids, body_idx]
    ori_err = quat_error_magnitude(body_quat_w[env_ids, body_idx], des_quat_w)
    vel_err = torch.norm(body_vel_w[env_ids, body_idx] - des_vel_w, dim=-1)
    return pos_err, ori_err, vel_err, body_idx


def best_racket_tracking_errors_from_env(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Convenience wrapper using the env command manager and scene entity body ids."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    pos_err, ori_err, vel_err, _ = best_racket_tracking_errors(
        command, asset, asset_cfg.body_ids  # type: ignore[arg-type]
    )
    return pos_err, ori_err, vel_err


def best_racket_state_w(
    command: torch.Tensor,
    asset: Articulation,
    body_ids: list[int],
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """World-frame position and linear velocity of the closest racket proxy link."""
    pos_err, _, _, body_idx = best_racket_tracking_errors(command, asset, body_ids)
    env_ids = torch.arange(command.shape[0], device=command.device)
    body_pos_w = _body_positions_w(asset, body_ids)
    body_vel_w = _body_lin_vel_w(asset, body_ids)
    return body_pos_w[env_ids, body_idx], body_vel_w[env_ids, body_idx], pos_err


def best_racket_state_from_env(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    return best_racket_state_w(command, asset, asset_cfg.body_ids)  # type: ignore[arg-type]


def strike_axis_w(command: torch.Tensor, asset: Articulation) -> tuple[torch.Tensor, torch.Tensor]:
    """Unit strike direction and commanded speed magnitude [m/s] in world frame."""
    _, _, des_vel_w, _, _ = command_targets_w(command, asset)
    cmd_speed = torch.norm(des_vel_w, dim=-1)
    strike_dir = des_vel_w / cmd_speed.unsqueeze(-1).clamp(min=1.0e-6)
    return strike_dir, cmd_speed


def impact_urgency_weight(lead_time_left: torch.Tensor, urgency_time_constant: float, prep_floor: float) -> torch.Tensor:
    """Weight in [prep_floor, 1]; always some signal during countdown, peak at impact."""
    urgency = torch.exp(-lead_time_left / max(urgency_time_constant, 1.0e-3))
    return prep_floor + (1.0 - prep_floor) * urgency
