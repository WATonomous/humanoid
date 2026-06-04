from __future__ import annotations

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
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms
>>>>>>> 97ddcbcd (rl-badminton)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

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


def _parse_intercept_command(
    command: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Split privileged intercept command into pos, hit pulse, and time-to-hit."""
    return command[:, :3], command[:, 3], command[:, 4]


def _command_target_w(
    env: ManagerBasedRLEnv, command_name: str, asset: Articulation
) -> tuple[torch.Tensor, torch.Tensor]:
    command = env.command_manager.get_command(command_name)
    des_pos_b, hit_moment_active, _ = _parse_intercept_command(command)
    des_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b
    )
    return des_pos_w, hit_moment_active


def _body_positions_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.body_state_w[:, asset_cfg.body_ids, :3]  # type: ignore[index]


def _body_lin_vel_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """World-frame linear velocity of candidate racket links (no extra sensors)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]  # type: ignore[index]


def _swing_direction_w(env: ManagerBasedRLEnv, command_name: str, asset: Articulation) -> torch.Tensor:
    """Privileged strike direction: from base through the intercept point."""
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b
    )
    delta = des_pos_w - asset.data.root_pos_w
    return delta / torch.norm(delta, dim=-1, keepdim=True).clamp(min=1.0e-6)


def _best_racket_speed(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Max linear speed [m/s] among racket proxy links."""
    return torch.norm(_body_lin_vel_w(env, asset_cfg), dim=-1).max(dim=1).values


def _best_racket_swing_component(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Max signed speed [m/s] along the privileged strike direction (world frame)."""
    asset: Articulation = env.scene[asset_cfg.name]
    swing_dir_w = _swing_direction_w(env, command_name, asset)
    lin_vel_w = _body_lin_vel_w(env, asset_cfg)
    swing_component = (lin_vel_w * swing_dir_w.unsqueeze(1)).sum(dim=-1)
    return swing_component.max(dim=1).values


def intercept_proximity_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Minimum distance from any candidate link to the intercept point."""
    asset: Articulation = env.scene[asset_cfg.name]
    des_pos_w, _ = _command_target_w(env, command_name, asset)
    body_pos_w = _body_positions_w(env, asset_cfg)
    dists = torch.norm(body_pos_w - des_pos_w.unsqueeze(1), dim=-1)
    return dists.min(dim=1).values


def intercept_proximity_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Prep reward: move toward the intercept before the shuttle arrives."""
    return 1.0 - torch.tanh(intercept_proximity_error(env, command_name, asset_cfg) / std)


def timed_intercept_proximity(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Contact-moment reward: be at the intercept when the shuttle arrives (hit pulse)."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, hit_moment_active = _command_target_w(env, command_name, asset)
    proximity = intercept_proximity_tanh(env, std, command_name, asset_cfg)
    return proximity * (hit_moment_active > 0.5).float()


def timed_hit_bonus(
    env: ManagerBasedRLEnv,
    hit_radius: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
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


def timed_swing_speed(
    env: ManagerBasedRLEnv,
    speed_std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward racket speed at contact (uses articulation body velocity, no contact sensor)."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, hit_moment_active = _command_target_w(env, command_name, asset)
    speed = _best_racket_speed(env, asset_cfg)
    return torch.tanh(speed / speed_std) * (hit_moment_active > 0.5).float()


def timed_swing_through_target(
    env: ManagerBasedRLEnv,
    speed_std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    hit_radius: float = 0.08,
) -> torch.Tensor:
    """Reward swinging *through* the intercept at contact, not just resting there.

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
