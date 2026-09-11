"""Shuttle aerodynamics: point mass + quadratic drag.

a = g - k * |v| * v,  k = g / v_t^2

drag_accel is a pure function reused in three places: the RK4 reference
integrator here, the mujoco xfrc_applied hook (scene/), and the interception
predictor (predictor.py). It works on numpy arrays and on jax arrays (no
in-place ops, no branching on values).
"""

from __future__ import annotations

import os

import numpy as np
import yaml

GRAVITY = 9.81

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS_PATH = os.path.join(_HERE, "scene", "params.yaml")


def load_params(path: str = DEFAULT_PARAMS_PATH) -> dict:
    """Load params.yaml and derive k from v_t. Returns a plain dict."""
    with open(path) as f:
        params = yaml.safe_load(f)
    g = params["gravity"]
    v_t = params["shuttle"]["v_t"]
    params["shuttle"]["k"] = g / v_t**2
    return params


def k_from_vt(v_t: float, g: float = GRAVITY) -> float:
    return g / v_t**2


def drag_accel(v, k: float, g: float = GRAVITY):
    """Acceleration a = g_vec - k*|v|*v. v shape (3,) or (..., 3)."""
    speed = (v * v).sum(axis=-1, keepdims=True) ** 0.5
    g_vec = np.array([0.0, 0.0, -g])
    return g_vec - k * speed * v


def rk4_step(p, v, k: float, dt: float, g: float = GRAVITY):
    """One RK4 step of the (p, v) ODE. dt may be negative (back-integration)."""
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


def simulate(
    p0,
    v0,
    k: float,
    dt: float = 1e-3,
    t_max: float = 10.0,
    g: float = GRAVITY,
    stop_at_ground: bool = True,
    ground_z: float = 0.0,
):
    """Forward RK4 rollout. Returns (t, P, V) arrays, shapes (N,), (N, 3), (N, 3).

    If stop_at_ground, the rollout ends at the first sample with z < ground_z
    (that sample is included so callers can interpolate the landing point).
    """
    p = np.asarray(p0, dtype=float)
    v = np.asarray(v0, dtype=float)
    n = int(round(t_max / dt)) + 1
    T = np.empty(n)
    P = np.empty((n, 3))
    V = np.empty((n, 3))
    T[0], P[0], V[0] = 0.0, p, v
    for i in range(1, n):
        p, v = rk4_step(p, v, k, dt, g)
        T[i], P[i], V[i] = i * dt, p, v
        if stop_at_ground and p[2] < ground_z:
            return T[: i + 1], P[: i + 1], V[: i + 1]
    return T, P, V


def back_integrate(p_star, v_star, k: float, t_star: float, dt: float = 1e-3,
                   g: float = GRAVITY, max_speed: float | None = None):
    """Integrate backwards from (p*, v*) for t* seconds.

    Returns (p0, v0, traj) where traj = (t, P, V) is the *forward-time*
    reference trajectory from launch (t=0) to the intercept (t=t*).
    Reversed-time drag is anti-damped so error grows backwards; over <= 1.5 s
    at receive speeds RK4 at 1 ms is fine (gate 3.1 verifies). Quadratic drag
    diverges in finite backward time, so with max_speed set the integration
    bails out and returns (None, None, None) once |v| exceeds it.
    """
    p = np.asarray(p_star, dtype=float)
    v = np.asarray(v_star, dtype=float)
    cap2 = np.inf if max_speed is None else float(max_speed) ** 2
    n = int(round(t_star / dt))
    P = np.empty((n + 1, 3))
    V = np.empty((n + 1, 3))
    P[n], V[n] = p, v
    for i in range(n, 0, -1):
        p, v = rk4_step(p, v, k, -dt, g)
        if v @ v > cap2:
            return None, None, None
        P[i - 1], V[i - 1] = p, v
    T = np.arange(n + 1) * dt
    return P[0].copy(), V[0].copy(), (T, P, V)


def landing_point(t, P, ground_z: float = 0.0):
    """Linear interpolation of the ground crossing of a rollout that ended below
    ground_z. Returns (t_land, p_land)."""
    if P[-1, 2] >= ground_z:
        raise ValueError("trajectory does not reach the ground")
    z0, z1 = P[-2, 2], P[-1, 2]
    frac = (z0 - ground_z) / (z0 - z1)
    t_land = t[-2] + frac * (t[-1] - t[-2])
    p_land = P[-2] + frac * (P[-1] - P[-2])
    return t_land, p_land


def fit_k(times, points, k0: float = 0.21, g: float = GRAVITY):
    """Least-squares fit of k (and the initial state) to tracked positions.

    times: (N,) sample times starting near 0; points: (N, 3) positions from
    film. Fits p0, v0, k jointly on the RK4 model. Returns (k_fit, v_t_fit,
    result) where result is the scipy OptimizeResult for residual inspection.
    """
    from scipy.optimize import least_squares

    times = np.asarray(times, dtype=float)
    points = np.asarray(points, dtype=float)
    v0_guess = (points[1] - points[0]) / (times[1] - times[0])

    def rollout(p0, v0, k):
        out = np.empty_like(points)
        p, v = p0.copy(), v0.copy()
        t_prev = times[0]
        for i, t in enumerate(times):
            n = int(round((t - t_prev) / 1e-3))
            for _ in range(n):
                p, v = rk4_step(p, v, k, 1e-3, g)
            t_prev += n * 1e-3
            out[i] = p
        return out

    def residual(x):
        p0, v0, k = x[:3], x[3:6], x[6]
        return (rollout(p0, v0, k) - points).ravel()

    x0 = np.concatenate([points[0], v0_guess, [k0]])
    result = least_squares(residual, x0, x_scale="jac")
    k_fit = float(result.x[6])
    v_t_fit = float(np.sqrt(g / k_fit))
    return k_fit, v_t_fit, result


def solve_u_out(p_from, target_xy, params, clearance=None):
    """Shooting solve: outgoing velocity from p_from landing on target_xy.

    Scans elevation, bisects speed per elevation to hit the target range, and
    keeps the lowest-speed solution whose net crossing clears by >= clearance.
    Returns (u_out, speed, v_out). Falls back to a 45° lob if nothing clears.
    """
    p = params
    k = p["shuttle"]["k"]
    clearance = p["control"]["return_net_clearance"] if clearance is None else clearance
    net_top = p["net"]["height_top"]
    p_from = np.asarray(p_from, dtype=float)
    tgt = np.array([target_xy[0], target_xy[1], 0.0])
    dir_xy = tgt[:2] - p_from[:2]
    dist_xy = float(np.linalg.norm(dir_xy))
    dir_xy /= dist_xy

    best = None
    best_score = None
    for elev_deg in np.arange(20.0, 66.0, 5.0):
        el = np.radians(elev_deg)
        u = np.array([dir_xy[0] * np.cos(el), dir_xy[1] * np.cos(el), np.sin(el)])

        def landing_range(speed):
            t, P, V = simulate(p_from, speed * u, k, dt=2e-3, t_max=4.0)
            if P[-1, 2] >= 0:
                return None, None
            _, p_land = landing_point(t, P)
            rng = (p_land[:2] - p_from[:2]) @ dir_xy
            return rng, (t, P, V)

        lo_s, hi_s = 2.0, 35.0
        rng_hi, _ = landing_range(hi_s)
        if rng_hi is None or rng_hi < dist_xy:
            continue
        for _ in range(24):
            mid = 0.5 * (lo_s + hi_s)
            rng, traj = landing_range(mid)
            if rng is None or rng < dist_xy:
                lo_s = mid
            else:
                hi_s = mid
        speed = hi_s
        rng, traj = landing_range(speed)
        if traj is None:
            continue
        t, P, V = traj
        # net crossing outbound (arm side y<0 to far side y>0)
        y = P[:, 1]
        cross = np.nonzero((y[:-1] < 0) & (y[1:] >= 0))[0]
        if len(cross) == 0:
            continue
        i = cross[0]
        frac = -y[i] / (y[i + 1] - y[i])
        z_cross = P[i, 2] + frac * (P[i + 1, 2] - P[i, 2])
        if z_cross < net_top + clearance:
            continue
        # v1 accepts whatever outgoing speed contact produces, so prefer a
        # steepish stroke (~50 deg) that clears the net even when slow
        score = -(elev_deg - 50.0) ** 2
        if best is None or score > best_score:
            best = (u, speed)
            best_score = score
    if best is None:
        el = np.radians(45.0)
        u = np.array([dir_xy[0] * np.cos(el), dir_xy[1] * np.cos(el), np.sin(el)])
        return u, 10.0, 10.0 * u
    u, speed = best
    return u, speed, speed * u
