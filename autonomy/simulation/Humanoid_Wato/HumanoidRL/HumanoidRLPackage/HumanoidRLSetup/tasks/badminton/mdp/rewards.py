from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ee_tracking import (
    DEFAULT_RACKET_BODY_NAMES,
    best_racket_state_from_env,
    best_racket_tracking_errors_from_env,
    parse_intercept_command,
    strike_axis_w,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _in_swing_window(lead_time_left: torch.Tensor, approach_window_s: float) -> torch.Tensor:
    """1.0 only during the timed launch window before impact (not while waiting at intercept)."""
    return ((lead_time_left > 0.0) & (lead_time_left <= approach_window_s)).float()


def early_at_target_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    zone_radius: float,
    min_lead_time_remaining: float,
) -> torch.Tensor:
    """Penalize camping at the intercept long before impact (ready pose elsewhere is fine)."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    in_zone = (pos_err < zone_radius).float()
    too_early = (lead_time_left > min_lead_time_remaining).float()
    return in_zone * too_early


def coarse_aim_toward_intercept_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    approach_window_s: float,
) -> torch.Tensor:
    """Weak aim hint while waiting (outside launch window): where to strike, not go there yet."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    outside_window = (lead_time_left > approach_window_s).float()
    proximity = 1.0 - torch.tanh(pos_err / std)
    return proximity * outside_window


def timed_swing_approach_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    approach_window_s: float,
    speed_std: float,
    hit_radius: float,
    range_std: float,
) -> torch.Tensor:
    """Reward strike-speed motion toward the intercept during the launch window.

    Must be closing distance (``range_std``) — not reward for waving toward the target from far away.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    _, curr_vel_w, pos_err = best_racket_state_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    strike_dir, cmd_speed = strike_axis_w(command, asset)
    speed_along = (curr_vel_w * strike_dir).sum(dim=-1).clamp(min=0.0)

    in_window = _in_swing_window(lead_time_left, approach_window_s)
    still_approaching = (pos_err > hit_radius).float()
    # Suppress reward when still far: avoids "twitch toward target" without arriving.
    range_gate = torch.exp(-torch.square(pos_err / max(range_std, 1.0e-6)))
    speed_term = torch.exp(-torch.square((speed_along - cmd_speed) / max(speed_std, 1.0e-6)))
    return speed_term * range_gate * in_window * still_approaching


def ee_impact_position_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
) -> torch.Tensor:
    """Reward being at the intercept on the hit pulse."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    at_hit = (hit_moment_active > 0.5).float()
    pos_term = torch.exp(-torch.square(pos_err / max(pos_std, 1.0e-6)))
    return pos_term * at_hit


def ee_impact_swing_through_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
    speed_std: float,
    hit_radius: float,
) -> torch.Tensor:
    """At impact: at the intercept with commanded speed along the strike axis."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    _, curr_vel_w, pos_err = best_racket_state_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    strike_dir, cmd_speed = strike_axis_w(command, asset)
    speed_along = (curr_vel_w * strike_dir).sum(dim=-1)

    at_hit = (hit_moment_active > 0.5).float()
    in_zone = (pos_err < hit_radius).float()
    pos_term = torch.exp(-torch.square(pos_err / max(pos_std, 1.0e-6)))
    speed_term = torch.exp(-torch.square((speed_along - cmd_speed) / max(speed_std, 1.0e-6)))
    return pos_term * speed_term * at_hit * in_zone


def ee_impact_orientation_hit_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    ori_std: float,
    hit_radius: float = 0.13,
) -> torch.Tensor:
    """Optional: orientation match at impact when already in the zone."""
    command = env.command_manager.get_command(command_name)
    pos_err, ori_err, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, hit_moment_active, _ = parse_intercept_command(command)

    at_hit = (hit_moment_active > 0.5).float()
    in_zone = (pos_err < hit_radius).float()
    ori_term = torch.exp(-torch.square(ori_err / max(ori_std, 1.0e-6)))
    return ori_term * at_hit * in_zone


def racket_speed_penalty_outside_swing_window(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    distance_threshold: float,
    approach_window_s: float,
) -> torch.Tensor:
    """Penalize idle jitter when far from the intercept and outside the launch window."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    lin_vel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]  # type: ignore[index]
    speed = torch.norm(lin_vel_w, dim=-1).max(dim=1).values

    far = (pos_err > distance_threshold).float()
    outside_window = (lead_time_left > approach_window_s).float()
    return speed * far * outside_window
