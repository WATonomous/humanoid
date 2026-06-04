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


def _parse_intercept_command(command: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Split intercept command into position (base frame), window active, and lead time."""
    return command[:, :3], command[:, 3], command[:, 4]


def _command_target_w(
    env: ManagerBasedRLEnv, command_name: str, asset: Articulation
) -> tuple[torch.Tensor, torch.Tensor]:
    command = env.command_manager.get_command(command_name)
    des_pos_b, window_active, _ = _parse_intercept_command(command)
    des_pos_w, _ = combine_frame_transforms(
        asset.data.root_state_w[:, :3], asset.data.root_state_w[:, 3:7], des_pos_b
    )
    return des_pos_w, window_active


def _body_positions_w(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.body_state_w[:, asset_cfg.body_ids, :3]  # type: ignore[index]


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
    """Phase 1: spatial reward for reaching the intercept point (always active)."""
    return 1.0 - torch.tanh(intercept_proximity_error(env, command_name, asset_cfg) / std)


def timed_intercept_proximity(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Phase 2: spatial reward gated on the active hit window."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, window_active = _command_target_w(env, command_name, asset)
    proximity = intercept_proximity_tanh(env, std, command_name, asset_cfg)
    return proximity * (window_active > 0.5).float()


def timed_hit_bonus(
    env: ManagerBasedRLEnv,
    hit_radius: float,
    command_name: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Large bonus when a racket proxy link enters the sweet spot during the hit window."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, window_active = _command_target_w(env, command_name, asset)
    error = intercept_proximity_error(env, command_name, asset_cfg)
    in_sweet_spot = (error < hit_radius).float()
    return in_sweet_spot * (window_active > 0.5).float()
>>>>>>> 97ddcbcd (rl-badminton)
