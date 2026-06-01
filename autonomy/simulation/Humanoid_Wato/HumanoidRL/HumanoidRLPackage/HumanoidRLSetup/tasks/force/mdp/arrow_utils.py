from __future__ import annotations

import torch

import isaaclab.utils.math as math_utils


def resolve_force_to_arrow(
    force_w: torch.Tensor,
    default_scale: tuple[float, float, float],
    length_scale: float = 0.08,
    min_display_force: float = 0.25,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert a world-frame force vector into arrow scale and orientation (x-axis aligned)."""
    num_envs = force_w.shape[0]
    device = force_w.device

    magnitude = torch.norm(force_w, dim=1)
    direction = force_w / magnitude.clamp(min=1e-6).unsqueeze(-1)

    ref = torch.zeros(num_envs, 3, device=device)
    ref[:, 0] = 1.0
    dot = (ref * direction).sum(dim=-1).clamp(-1.0, 1.0)
    angle = torch.acos(dot)
    axis = torch.cross(ref, direction, dim=-1)
    axis_norm = torch.norm(axis, dim=-1)

    quat = torch.zeros(num_envs, 4, device=device)
    quat[:, 0] = 1.0
    valid = axis_norm > 1e-6
    if valid.any():
        quat[valid] = math_utils.quat_from_angle_axis(angle[valid], axis[valid] / axis_norm[valid].unsqueeze(-1))

    flipped = dot < -0.999
    if flipped.any():
        y_axis = torch.zeros(flipped.sum(), 3, device=device)
        y_axis[:, 1] = 1.0
        quat[flipped] = math_utils.quat_from_angle_axis(
            torch.full((flipped.sum(),), torch.pi, device=device),
            y_axis,
        )

    arrow_scale = torch.tensor(default_scale, device=device).repeat(num_envs, 1)
    display_mag = magnitude.clamp(min=min_display_force)
    arrow_scale[:, 0] *= display_mag * length_scale

    return arrow_scale, quat
