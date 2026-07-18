"""Termination terms for the pick-and-place task.

Mimic-only for now (see pick_place_bimanual_mimic_env.py): the base task's
TerminationsCfg does not use these. Wiring a live success DoneTerm into the
manager that generate_demos.py's env uses would be actively wrong there --
that driver treats ANY termination as episode failure
(`orch._fail("env_terminated")`), so a success term firing mid-trajectory
(e.g. the instant the object is released and settles) would flip a real
success into a recorded failure. Isaac Lab Mimic's own tooling
(annotate_demos.py, generate_dataset.py) avoids this by extracting
`terminations.success` and setting it to None before building the env, then
calling `.func(env, **params)` manually -- so it never runs inside the live
TerminationManager either. Keep it that way.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_placed_success(
    env: "ManagerBasedRLEnv",
    xy_tolerance: float,
    z_tolerance: float,
    max_lin_vel: float,
    object_half_height: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    target_cfg: SceneEntityCfg | None = None,
    target_support_offset: float = 0.0,
    target_xy: tuple[float, float] | None = None,
    target_support_z: float | None = None,
) -> torch.Tensor:
    """Object is within tolerance of its place target and has (instantaneously)
    settled.

    Mirrors pick_place_gen/generate_demos.py:check_success, minus the
    multi-step settle-step counter -- that debounce lives in the standalone
    driver's post-release hold loop; here we do a single-instant check (the
    velocity threshold already makes this robust to a still-bouncing object).

    The place target is given either by a live scene asset (`target_cfg` --
    the stack base cube or the tray, whose current pose gives the target XY
    and support height) or a fixed analytic XY/height (`target_xy` /
    `target_support_z`, for targets with no moving scene asset to read).
    Exactly one of the two must be provided by the caller.
    """
    obj: RigidObject = env.scene[object_cfg.name]
    obj_pos = obj.data.root_pos_w - env.scene.env_origins
    obj_vel = obj.data.root_lin_vel_w

    if target_cfg is not None:
        target: RigidObject = env.scene[target_cfg.name]
        target_pos = target.data.root_pos_w - env.scene.env_origins
        target_xy_t = target_pos[:, :2]
        target_z = target_pos[:, 2] + target_support_offset + object_half_height
    else:
        target_xy_t = torch.tensor(target_xy, device=env.device).expand(obj_pos.shape[0], 2)
        target_z = torch.full((obj_pos.shape[0],), target_support_z + object_half_height, device=env.device)

    xy_err = torch.linalg.norm(obj_pos[:, :2] - target_xy_t, dim=-1)
    z_err = (obj_pos[:, 2] - target_z).abs()
    vel = torch.linalg.norm(obj_vel, dim=-1)

    return (xy_err < xy_tolerance) & (z_err < z_tolerance) & (vel < max_lin_vel)
