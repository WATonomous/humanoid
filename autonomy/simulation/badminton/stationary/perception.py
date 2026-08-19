"""Simulated perception: noisy shuttle tracking + trajectory prior.

The policy must not learn flight prediction implicitly (it would distill a
second, wrong copy of the physics). Instead the trajectory is an input:

  teacher (privileged): traj_features from the TRUE shuttle state — the exact
      drag-model rollout the launcher and predictor already use.
  student: traj_features from an EKF fed one noisy position measurement per
      control tick. Early in the flight the velocity estimate is poor, so the
      predicted trajectory jumps tick to tick; it converges as measurements
      accumulate — the same behavior the real perception model shows as it
      refines its fit over time.

Both produce the same feature layout, so the teacher and student differ only
in what the features are computed from.

This module is the single-env numpy reference. perception_torch.py is the
batched port used by the GPU training env; test_perception.py checks the two
agree.
"""

from __future__ import annotations

import numpy as np

import aero

I3 = np.eye(3)


def traj_features(p, v, k: float, n_traj: int, traj_dt: float,
                  sub_dt: float, g: float = aero.GRAVITY) -> np.ndarray:
    """(n_traj, 3) future positions at traj_dt spacing from state (p, v).

    RK4 at sub_dt with every (traj_dt/sub_dt)-th sample recorded. No ground
    stop: points below the floor stay below it, which itself tells the policy
    where the flight ends.
    """
    per = int(round(traj_dt / sub_dt))
    p = np.asarray(p, dtype=float).copy()
    v = np.asarray(v, dtype=float).copy()
    out = np.empty((n_traj, 3))
    for i in range(n_traj):
        for _ in range(per):
            p, v = aero.rk4_step(p, v, k, sub_dt, g)
        out[i] = p
    return out


def drag_jacobian(v, k: float) -> np.ndarray:
    """d(drag_accel)/dv = -k (|v| I + v v^T / |v|); zero at v = 0."""
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        return np.zeros((3, 3))
    return -k * (speed * I3 + np.outer(v, v) / speed)


class ShuttleEkf:
    """EKF over the drag flight model, position-only measurements.

    State x = (p, v). Mean propagates with the exact RK4 step; the covariance
    uses the Euler linearization F = [[I, dt I], [0, I + dt A]] with
    A = drag_jacobian(v). Process noise is white acceleration sigma_acc.
    """

    def __init__(self, params: dict):
        pp = params["perception"]
        self.k = params["shuttle"]["k"]
        self.g = params["gravity"]
        self.sigma_meas = pp["sigma_meas"]
        self.sigma_p0 = pp["sigma_p0"]
        self.sigma_v0 = pp["sigma_v0"]
        self.sigma_acc = pp["sigma_acc"]
        self.x = np.zeros(6)
        self.P = np.zeros((6, 6))

    def init(self, z0) -> None:
        """First measurement: position from z0, velocity uninformed."""
        self.x = np.concatenate([np.asarray(z0, dtype=float), np.zeros(3)])
        self.P = np.diag([self.sigma_p0**2] * 3 + [self.sigma_v0**2] * 3)

    def predict(self, dt: float) -> None:
        p, v = self.x[:3], self.x[3:]
        self.x = np.concatenate(aero.rk4_step(p, v, self.k, dt, self.g))
        F = np.eye(6)
        F[:3, 3:] = dt * I3
        F[3:, 3:] += dt * drag_jacobian(v, self.k)
        Q = np.zeros((6, 6))
        Q[:3, :3] = (0.5 * self.sigma_acc * dt**2) ** 2 * I3
        Q[3:, 3:] = (self.sigma_acc * dt) ** 2 * I3
        self.P = F @ self.P @ F.T + Q

    def update(self, z) -> None:
        z = np.asarray(z, dtype=float)
        S = self.P[:3, :3] + self.sigma_meas**2 * I3
        K = self.P[:, :3] @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x[:3])
        KH = np.zeros((6, 6))
        KH[:, :3] = K
        self.P = (np.eye(6) - KH) @ self.P

    @property
    def p_hat(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def v_hat(self) -> np.ndarray:
        return self.x[3:].copy()


class PerceptionSim:
    """Per-episode driver: measure(true_p) each control tick -> estimate.

    Owns the measurement noise RNG and the EKF; call reset() at episode start
    and tick(true_p, dt) once per control tick. features() returns the
    (p_hat, v_hat, trajectory prior) block for the student observation;
    true_features(p, v) returns the same layout from the true state for the
    teacher.
    """

    def __init__(self, params: dict, seed: int = 0):
        pp = params["perception"]
        self.ekf = ShuttleEkf(params)
        self.k = params["shuttle"]["k"]
        self.g = params["gravity"]
        self.n_traj = pp["n_traj"]
        self.traj_dt = pp["traj_dt"]
        self.sub_dt = pp["traj_sub_dt"]
        self.sigma_meas = pp["sigma_meas"]
        self.rng = np.random.default_rng(seed)
        self._started = False

    @property
    def dim(self) -> int:
        return 6 + 3 * self.n_traj

    def reset(self) -> None:
        self._started = False

    def tick(self, true_p, dt: float) -> None:
        z = np.asarray(true_p, dtype=float) \
            + self.rng.normal(0.0, self.sigma_meas, 3)
        if not self._started:
            self.ekf.init(z)
            self._started = True
            return
        self.ekf.predict(dt)
        self.ekf.update(z)

    def features(self) -> np.ndarray:
        assert self._started, "tick() before features()"
        return self._layout(self.ekf.p_hat, self.ekf.v_hat)

    def true_features(self, p, v) -> np.ndarray:
        return self._layout(np.asarray(p, dtype=float),
                            np.asarray(v, dtype=float))

    def _layout(self, p, v) -> np.ndarray:
        traj = traj_features(p, v, self.k, self.n_traj, self.traj_dt,
                             self.sub_dt, self.g)
        return np.concatenate([p, v, traj.ravel()])
