from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ee_tracking import (
    DEFAULT_RACKET_BODY_NAMES,
    best_racket_tracking_errors_from_env,
    impact_urgency_weight,
    parse_intercept_command,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _racket_proxy_speed(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]  # type: ignore[index]
    return torch.norm(lin_vel_w, dim=-1).max(dim=1).values


def intercept_proximity_timed_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    urgency_time_constant: float = 0.8,
    prep_floor: float = 0.15,
) -> torch.Tensor:
    """Move toward intercept; strongest reward as ``time_to_hit → 0`` (discourages arriving early)."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    proximity = 1.0 - torch.tanh(pos_err / std)
    weight = impact_urgency_weight(lead_time_left, urgency_time_constant, prep_floor)
    return proximity * weight


def ee_position_approach_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
    urgency_time_constant: float = 0.8,
) -> torch.Tensor:
    """Fine position tracking with urgency peaking at impact."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    pos_term = torch.exp(-torch.square(pos_err / max(pos_std, 1.0e-6)))
    weight = impact_urgency_weight(lead_time_left, urgency_time_constant, prep_floor=0.0)
    return pos_term * weight


def early_at_target_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    zone_radius: float,
    min_lead_time_remaining: float,
) -> torch.Tensor:
    """Penalize sitting at the intercept while there is still time on the countdown."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    in_zone = (pos_err < zone_radius).float()
    too_early = (lead_time_left > min_lead_time_remaining).float()
    return in_zone * too_early


def ee_impact_position_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
) -> torch.Tensor:
    """Reward being at the intercept on the hit pulse (position only)."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    at_hit = (hit_moment_active > 0.5).float()
    pos_term = torch.exp(-torch.square(pos_err / max(pos_std, 1.0e-6)))
    return pos_term * at_hit


def ee_impact_velocity_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    vel_std: float,
    hit_radius: float = 0.13,
) -> torch.Tensor:
    """Reward matching commanded swing speed on the hit pulse, only when at the intercept."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, vel_err = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    at_hit = (hit_moment_active > 0.5).float()
    in_zone = (pos_err < hit_radius).float()
    vel_term = torch.exp(-torch.square(vel_err / max(vel_std, 1.0e-6)))
    return vel_term * at_hit * in_zone


def ee_impact_orientation_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    ori_std: float,
    hit_radius: float = 0.13,
) -> torch.Tensor:
    """Reward matching commanded orientation on the hit pulse, only when at the intercept."""
    command = env.command_manager.get_command(command_name)
    pos_err, ori_err, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    at_hit = (hit_moment_active > 0.5).float()
    in_zone = (pos_err < hit_radius).float()
    ori_term = torch.exp(-torch.square(ori_err / max(ori_std, 1.0e-6)))
    return ori_term * at_hit * in_zone


def racket_speed_penalty_far_from_target(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    distance_threshold: float,
) -> torch.Tensor:
    """Penalize fast racket motion when not near the intercept."""
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    speed = _racket_proxy_speed(env, asset_cfg)
    far = (pos_err > distance_threshold).float()
    return speed * far
