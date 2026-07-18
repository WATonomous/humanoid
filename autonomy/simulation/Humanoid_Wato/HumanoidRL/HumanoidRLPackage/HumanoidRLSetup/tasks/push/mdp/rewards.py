"""Reward terms for pushing the block up the ramp onto the box's interior floor.

All positions are expressed in the per-env frame (world minus ``env_origins``);
the push direction is the env-frame ``+x`` axis (the box is spawned with its
ramp's up-direction aligned to ``+x``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer

from .utils import BLOCK_HALF_SIZE, block_center_w

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def block_push_progress(
    env: ManagerBasedRLEnv,
    max_speed: float = 0.5,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Signed block velocity along the push axis (+x): rewards advancing up the
    ramp, penalizes sliding back down."""
    object: RigidObject = env.scene[object_cfg.name]
    return object.data.root_lin_vel_w[:, 0].clamp(-max_speed, max_speed)


def block_backslide(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Backslide speed (positive when the block moves in -x). Use with a
    negative weight for an extra asymmetric penalty on losing ramp progress."""
    object: RigidObject = env.scene[object_cfg.name]
    return (-object.data.root_lin_vel_w[:, 0]).clamp(min=0.0)


def block_to_target(
    env: ManagerBasedRLEnv,
    std: float,
    target: tuple[float, float],
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Tanh-kernel reward for reducing the xy distance from the block center to
    a target point on the interior floor (env frame)."""
    object: RigidObject = env.scene[object_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    target_t = torch.tensor(target, device=env.device)
    distance = torch.norm(center[:, :2] - target_t, dim=1)
    return 1.0 - torch.tanh(distance / std)


def ee_behind_block(
    env: ManagerBasedRLEnv,
    std: float,
    push_offset: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward the end-effector for staying in contact range BEHIND the block
    relative to the push direction (at ``center - push_offset * x_hat``)."""
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    push_point = block_center_w(object)
    push_point[:, 0] -= push_offset
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    distance = torch.norm(ee_w - push_point, dim=1)
    return 1.0 - torch.tanh(distance / std)


def ee_reposition_behind_block(
    env: ManagerBasedRLEnv,
    std: float,
    push_offset: float,
    ramp_mouth: tuple[float, float],
    corridor_x_max: float,
    corridor_y: float,
    gate_std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Shaping reward for *routing around* a block that is off the direct ramp
    approach (beside/behind the box), so the arm can push it toward the ramp
    mouth instead of straight up ``+x``.

    Rewards the end-effector for reaching the point just BEHIND the block along
    the block->ramp-mouth direction (``center - push_offset * dir_to_mouth``),
    i.e. the side from which a push drives the block toward the ramp entrance.
    A spatial gate multiplies the reward by how far the block sits *outside* the
    direct-push corridor, so this term is ~0 whenever the block is already on
    the ramp approach -- it never fights the working straight-push behavior and
    only engages when ``ee_behind_block``/``push_progress`` alone would misguide.
    Intended to carry a nonzero weight only at wider curriculum stages.
    """
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    ee = ee_frame.data.target_pos_w[..., 0, :] - env.scene.env_origins

    mouth = torch.tensor(ramp_mouth, device=env.device)
    to_mouth = mouth - center[:, :2]
    dir_to_mouth = to_mouth / (torch.norm(to_mouth, dim=1, keepdim=True) + 1e-6)
    target_xy = center[:, :2] - push_offset * dir_to_mouth
    distance = torch.norm(ee[:, :2] - target_xy, dim=1)
    reach = 1.0 - torch.tanh(distance / std)

    # relevance gate: block's xy distance outside the ramp-approach corridor
    dx = (center[:, 0] - corridor_x_max).clamp(min=0.0)
    dy = (center[:, 1].abs() - corridor_y).clamp(min=0.0)
    off_corridor = torch.sqrt(dx.square() + dy.square())
    gate = torch.tanh(off_corridor / gate_std)
    return reach * gate


def block_on_floor(
    env: ManagerBasedRLEnv,
    x_min: float,
    x_max: float,
    y_half: float,
    floor_z: float,
    z_tol: float,
    rest_speed: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Success bonus: block center inside the interior-floor footprint, at the
    floor height, and at rest (settled on the elevated flat floor)."""
    object: RigidObject = env.scene[object_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    in_x = (center[:, 0] > x_min) & (center[:, 0] < x_max)
    in_y = center[:, 1].abs() < y_half
    on_floor = (center[:, 2] - (floor_z + BLOCK_HALF_SIZE)).abs() < z_tol
    at_rest = torch.norm(object.data.root_lin_vel_w, dim=1) < rest_speed
    return (in_x & in_y & on_floor & at_rest).float()


def block_scoop_penalty(
    env: ManagerBasedRLEnv,
    ramp_base_x: float,
    ramp_top_x: float,
    floor_z: float,
    box_x_max: float,
    box_y_half: float,
    height_tol: float,
    dist_std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Location-aware anti-scoop term: block height above the LOCAL support
    surface (table / ramp / interior floor), weighted by the xy distance from
    the ramp+box footprint. ~Zero on the ramp and interior floor (the block
    MUST gain height there), positive when lifted in open table space.
    Use with a negative weight."""
    object: RigidObject = env.scene[object_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    x, y, z = center[:, 0], center[:, 1], center[:, 2]
    # local support-surface height: 0 before the ramp, linear up the ramp, floor after
    frac = ((x - ramp_base_x) / (ramp_top_x - ramp_base_x)).clamp(0.0, 1.0)
    surface_h = frac * floor_z
    # upper-clamped: an unbounded excess lets a single physics contact glitch
    # (e.g. a block launched by a bad contact resolution) produce a reward
    # outlier large enough to blow up the value function target (observed:
    # value_function loss 5 -> 3.7e5 -> 9.5e15 -> inf within 4 iterations).
    excess = (z - BLOCK_HALF_SIZE - surface_h - height_tol).clamp(min=0.0, max=0.2)
    # xy distance outside the ramp+box footprint rectangle (0 inside)
    dx = (ramp_base_x - 0.02 - x).clamp(min=0.0) + (x - box_x_max).clamp(min=0.0)
    dy = (y.abs() - box_y_half).clamp(min=0.0)
    outside_dist = torch.sqrt(dx.square() + dy.square())
    weight = torch.tanh(outside_dist / dist_std)
    return excess * weight


def block_lateral_deviation(
    env: ManagerBasedRLEnv,
    y_tol: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Sideways drift of the block off the ramp approach corridor. Use with a
    negative weight."""
    object: RigidObject = env.scene[object_cfg.name]
    center = block_center_w(object) - env.scene.env_origins
    return (center[:, 1].abs() - y_tol).clamp(min=0.0)
