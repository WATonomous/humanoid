"""Custom reset events for the pick-and-place task."""
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def reset_objects_min_separation(
    env: "ManagerBasedEnv",
    env_ids: torch.Tensor,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    yaw_range: tuple[float, float],
    z_values: list[float],
    min_separation: float,
    asset_cfgs: list[SceneEntityCfg],
    max_resamples: int = 25,
):
    """Sample poses for several objects jointly, enforcing pairwise XY separation.

    Positions are env-local; z is absolute per object (resting height on the
    table). Velocities are zeroed.
    """
    device = env.device
    n_env = len(env_ids)
    n_obj = len(asset_cfgs)

    xy = torch.zeros(n_env, n_obj, 2, device=device)
    lo = torch.tensor([x_range[0], y_range[0]], device=device)
    hi = torch.tensor([x_range[1], y_range[1]], device=device)
    for k in range(n_obj):
        xy[:, k] = sample_uniform(lo, hi, (n_env, 2), device=device)
    for _ in range(max_resamples):
        if n_obj < 2:
            break
        bad = torch.zeros(n_env, n_obj, dtype=torch.bool, device=device)
        for a in range(n_obj):
            for b in range(a + 1, n_obj):
                too_close = (xy[:, a] - xy[:, b]).norm(dim=-1) < min_separation
                bad[:, b] |= too_close
        if not bad.any():
            break
        resampled = sample_uniform(lo, hi, (n_env, n_obj, 2), device=device)
        xy = torch.where(bad.unsqueeze(-1), resampled, xy)

    zeros = torch.zeros(n_env, device=device)
    for k, asset_cfg in enumerate(asset_cfgs):
        asset: RigidObject = env.scene[asset_cfg.name]
        yaw = sample_uniform(
            torch.tensor(yaw_range[0], device=device),
            torch.tensor(yaw_range[1], device=device),
            (n_env,), device=device,
        )
        quat = quat_from_euler_xyz(zeros, zeros, yaw)
        pos = torch.zeros(n_env, 3, device=device)
        pos[:, 0:2] = xy[:, k]
        pos[:, 2] = z_values[k]
        pos += env.scene.env_origins[env_ids]
        asset.write_root_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
        asset.write_root_velocity_to_sim(
            torch.zeros(n_env, 6, device=device), env_ids=env_ids
        )
