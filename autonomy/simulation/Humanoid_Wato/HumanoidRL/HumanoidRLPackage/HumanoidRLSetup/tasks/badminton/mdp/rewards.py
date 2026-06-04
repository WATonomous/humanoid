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
    best_racket_state_from_env,
    best_racket_tracking_errors_from_env,
    parse_intercept_command,
    strike_axis_w,
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
<<<<<<< HEAD
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
=======
    """Penalize camping at the intercept long before impact (ready pose elsewhere is fine)."""
>>>>>>> 00aee69e (improve-badminton-rl)
    command = env.command_manager.get_command(command_name)
    pos_err, _, _ = best_racket_tracking_errors_from_env(env, command_name, asset_cfg)
    _, _, _, _, lead_time_left = parse_intercept_command(command)

    in_zone = (pos_err < zone_radius).float()
    too_early = (lead_time_left > min_lead_time_remaining).float()
    return in_zone * too_early
>>>>>>> bfee0731 (improve-badminton-rl)


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

<<<<<<< HEAD
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
=======
    strike_dir, cmd_speed = strike_axis_w(command, asset)
    speed_along = (curr_vel_w * strike_dir).sum(dim=-1)

>>>>>>> 00aee69e (improve-badminton-rl)
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
<<<<<<< HEAD
    return speed * far
>>>>>>> bfee0731 (improve-badminton-rl)
=======
    outside_window = (lead_time_left > approach_window_s).float()
    return speed * far * outside_window
>>>>>>> 00aee69e (improve-badminton-rl)
