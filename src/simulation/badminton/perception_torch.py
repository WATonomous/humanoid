"""Batched torch port of perception.py for the GPU training env.

Same math as the numpy reference (ShuttleEkf / traj_features), vectorized
over a batch of envs and kept free of mjlab/warp imports so the parity test
in tests/test_perception.py can run it on CPU torch anywhere.

All functions are pure over tensors; the caller (the mjlab task) owns the
state tensors x (B, 6) and P (B, 6, 6) and resets rows on episode reset.
"""

from __future__ import annotations

import torch

G_VEC = (0.0, 0.0, -9.81)


def _g_vec(g: float, like: torch.Tensor) -> torch.Tensor:
    return torch.tensor([0.0, 0.0, -g], dtype=like.dtype, device=like.device)


def drag_accel(v: torch.Tensor, k: float, g: float) -> torch.Tensor:
    """a = g_vec - k |v| v. v: (..., 3)."""
    speed = v.norm(dim=-1, keepdim=True)
    return _g_vec(g, v) - k * speed * v


def rk4_step(p: torch.Tensor, v: torch.Tensor, k: float, dt: float,
             g: float) -> tuple[torch.Tensor, torch.Tensor]:
    a1 = drag_accel(v, k, g)
    v2 = v + 0.5 * dt * a1
    a2 = drag_accel(v2, k, g)
    v3 = v + 0.5 * dt * a2
    a3 = drag_accel(v3, k, g)
    v4 = v + dt * a3
    a4 = drag_accel(v4, k, g)
    p_next = p + dt / 6.0 * (v + 2 * v2 + 2 * v3 + v4)
    v_next = v + dt / 6.0 * (a1 + 2 * a2 + 2 * a3 + a4)
    return p_next, v_next


def traj_features(p: torch.Tensor, v: torch.Tensor, k: float, n_traj: int,
                  traj_dt: float, sub_dt: float, g: float) -> torch.Tensor:
    """(B, n_traj, 3) future positions at traj_dt spacing from (B, 3) state."""
    per = int(round(traj_dt / sub_dt))
    out = []
    for _ in range(n_traj):
        for _ in range(per):
            p, v = rk4_step(p, v, k, sub_dt, g)
        out.append(p)
    return torch.stack(out, dim=1)


def drag_jacobian(v: torch.Tensor, k: float) -> torch.Tensor:
    """(B, 3, 3) = -k (|v| I + v v^T / |v|), zero rows where v ~ 0."""
    B = v.shape[0]
    speed = v.norm(dim=-1)                                    # (B,)
    eye = torch.eye(3, dtype=v.dtype, device=v.device).expand(B, 3, 3)
    outer = v.unsqueeze(-1) * v.unsqueeze(-2)                 # (B, 3, 3)
    safe = speed.clamp_min(1e-9)
    A = -k * (speed[:, None, None] * eye + outer / safe[:, None, None])
    return torch.where((speed > 1e-9)[:, None, None], A, torch.zeros_like(A))


def ekf_init(z0: torch.Tensor, sigma_p0: float,
             sigma_v0: float) -> tuple[torch.Tensor, torch.Tensor]:
    """First measurement -> (x, P). z0: (B, 3)."""
    B = z0.shape[0]
    x = torch.cat([z0, torch.zeros_like(z0)], dim=-1)
    P = torch.zeros(B, 6, 6, dtype=z0.dtype, device=z0.device)
    idx = torch.arange(3, device=z0.device)
    P[:, idx, idx] = sigma_p0**2
    P[:, idx + 3, idx + 3] = sigma_v0**2
    return x, P


def ekf_predict(x: torch.Tensor, P: torch.Tensor, dt: float, k: float,
                g: float, sigma_acc: float) -> tuple[torch.Tensor, torch.Tensor]:
    p, v = x[:, :3], x[:, 3:]
    p_next, v_next = rk4_step(p, v, k, dt, g)
    x_next = torch.cat([p_next, v_next], dim=-1)
    B = x.shape[0]
    F = torch.eye(6, dtype=x.dtype, device=x.device).expand(B, 6, 6).clone()
    idx = torch.arange(3, device=x.device)
    F[:, idx, idx + 3] = dt
    F[:, 3:, 3:] += dt * drag_jacobian(v, k)
    Q = torch.zeros_like(P)
    Q[:, idx, idx] = (0.5 * sigma_acc * dt**2) ** 2
    Q[:, idx + 3, idx + 3] = (sigma_acc * dt) ** 2
    return x_next, F @ P @ F.transpose(-1, -2) + Q


def ekf_update(x: torch.Tensor, P: torch.Tensor, z: torch.Tensor,
               sigma_meas: float) -> tuple[torch.Tensor, torch.Tensor]:
    eye3 = torch.eye(3, dtype=x.dtype, device=x.device)
    S = P[:, :3, :3] + sigma_meas**2 * eye3
    K = P[:, :, :3] @ torch.linalg.inv(S)                     # (B, 6, 3)
    innov = (z - x[:, :3]).unsqueeze(-1)                      # (B, 3, 1)
    x_next = x + (K @ innov).squeeze(-1)
    KH = torch.zeros_like(P)
    KH[:, :, :3] = K
    eye6 = torch.eye(6, dtype=x.dtype, device=x.device)
    return x_next, (eye6 - KH) @ P


def feature_layout(p: torch.Tensor, v: torch.Tensor, k: float, n_traj: int,
                   traj_dt: float, sub_dt: float, g: float) -> torch.Tensor:
    """(B, 6 + 3 n_traj): [p, v, flattened trajectory prior] — matches
    perception.PerceptionSim._layout."""
    traj = traj_features(p, v, k, n_traj, traj_dt, sub_dt, g)
    return torch.cat([p, v, traj.flatten(1)], dim=-1)
