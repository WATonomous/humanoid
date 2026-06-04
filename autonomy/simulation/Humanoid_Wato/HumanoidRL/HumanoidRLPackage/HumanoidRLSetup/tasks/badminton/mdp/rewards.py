from __future__ import annotations

<<<<<<< HEAD
<<<<<<< HEAD
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ee_tracking import (
    DEFAULT_RACKET_BODY_NAMES,
    best_racket_tracking_errors_from_env,
    parse_intercept_command,
)
=======
import torch
=======
>>>>>>> bfee0731 (improve-badminton-rl)
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
<<<<<<< HEAD
from isaaclab.utils.math import combine_frame_transforms
>>>>>>> 97ddcbcd (rl-badminton)
=======

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ee_tracking import (
    DEFAULT_RACKET_BODY_NAMES,
    best_racket_tracking_errors_from_env,
    impact_urgency_weight,
    parse_intercept_command,
)
>>>>>>> bfee0731 (improve-badminton-rl)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

<<<<<<< HEAD
<<<<<<< HEAD

def early_at_target_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    zone_radius: float,
    min_lead_time_remaining: float,
) -> torch.Tensor:
    """Penalize waiting at the intercept before impact (does not reward moving closer)."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    in_zone = (pos_err < zone_radius).float()
    too_early = (lead_time_left > min_lead_time_remaining).float()
    return in_zone * too_early


def _ee_tracking_terms(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
    vel_std: float,
    ori_std: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Exp kernels for position, full 3D velocity, and orientation errors."""
    pos_err, ori_err, vel_err = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)

    pos_term = torch.exp(-torch.square(pos_err / max(pos_std, 1.0e-6)))
    vel_term = torch.exp(-torch.square(vel_err / max(vel_std, 1.0e-6)))
    ori_term = torch.exp(-torch.square(ori_err / max(ori_std, 1.0e-6)))
    return pos_term, vel_term, ori_term, pos_err


def ee_state_tracking_timed_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    pos_std: float,
    vel_std: float,
    ori_std: float,
    timing_std: float = 0.45,
    hit_bonus: float = 2.0,
) -> torch.Tensor:
    """Paper-style EE tracking: pos + full 3D vel + orientation, strongest near impact.

    Product of exp kernels (all must match for high reward). Not a proximity reward:
    far from target → ``pos_term ≈ 0``; wrong velocity vector → ``vel_term ≈ 0``;
    on target early → ``exp(-t/τ) ≈ 0``.
    """
    command = env.command_manager.get_command(command_name)
    pos_term, vel_term, ori_term, _ = _ee_tracking_terms(
        env, command_name, asset_cfg, pos_std, vel_std, ori_std
    )
    _, _, _, hit_moment_active, lead_time_left = parse_intercept_command(command)

    tracking = pos_term * vel_term * ori_term
    urgency = torch.exp(-lead_time_left / max(timing_std, 1.0e-3))
    at_hit = (hit_moment_active > 0.5).float()
    return tracking * (urgency + hit_bonus * at_hit)
=======
# Proxy for the racket head until a racket link is added to arm.usd.
DEFAULT_RACKET_BODY_NAMES = [
    "forearm_v8_.*",
    "DIP_INDEX_v1_.*",
]
=======
>>>>>>> bfee0731 (improve-badminton-rl)

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
<<<<<<< HEAD
    """Large bonus for being in the sweet spot on the exact shuttle-arrival step."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, hit_moment_active = _command_target_w(env, command_name, asset)
    error = intercept_proximity_error(env, command_name, asset_cfg)
    in_sweet_spot = (error < hit_radius).float()
<<<<<<< HEAD
    return in_sweet_spot * (window_active > 0.5).float()
>>>>>>> 97ddcbcd (rl-badminton)
=======
    return in_sweet_spot * (hit_moment_active > 0.5).float()
=======
    """Penalize sitting at the intercept while there is still time on the countdown."""
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    in_zone = (pos_err < zone_radius).float()
    too_early = (lead_time_left > min_lead_time_remaining).float()
    return in_zone * too_early
>>>>>>> bfee0731 (improve-badminton-rl)


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

<<<<<<< HEAD
    Uses world-frame body linear velocity and a privileged strike axis (base → intercept).
    """
    asset: Articulation = env.scene[asset_cfg.name]
    _, hit_moment_active = _command_target_w(env, command_name, asset)
    in_zone = (intercept_proximity_error(env, command_name, asset_cfg) < hit_radius).float()
    swing_speed = _best_racket_swing_component(env, command_name, asset_cfg)
    # Only reward forward motion along the strike axis (ignore pulling back).
    forward_swing = torch.clamp(swing_speed, min=0.0)
    return torch.tanh(forward_swing / speed_std) * in_zone * (hit_moment_active > 0.5).float()
>>>>>>> bf63d8b3 (rl-badminton)
=======
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
>>>>>>> bfee0731 (improve-badminton-rl)
