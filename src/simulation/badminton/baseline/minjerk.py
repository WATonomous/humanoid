"""Minimum-jerk (quintic) profiles for TRACK and SWING."""

from __future__ import annotations

import numpy as np


def quintic_coeffs(x0, v0, a0, xT, vT, aT, T: float) -> np.ndarray:
    """Per-axis quintic coefficients for boundary conditions at t=0 and t=T.

    Inputs are scalars or (n,) arrays; returns coeffs shape (6, n)."""
    x0, v0, a0 = np.atleast_1d(x0, v0, a0)
    xT, vT, aT = np.atleast_1d(xT, vT, aT)
    c0, c1, c2 = x0, v0, a0 / 2.0
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    dx = xT - c0 - c1 * T - c2 * T2
    dv = vT - c1 - 2 * c2 * T
    da = aT - 2 * c2
    c3 = (10 * dx - 4 * dv * T + 0.5 * da * T2) / T3
    c4 = (-15 * dx + 7 * dv * T - da * T2) / T4
    c5 = (6 * dx - 3 * dv * T + 0.5 * da * T2) / T5
    return np.stack([c0, c1, c2, c3, c4, c5])


def quintic_eval(coeffs: np.ndarray, t: float):
    """Evaluate (x, v, a) at time t for coeffs from quintic_coeffs."""
    c0, c1, c2, c3, c4, c5 = coeffs
    x = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
    v = c1 + 2 * c2 * t + 3 * c3 * t**2 + 4 * c4 * t**3 + 5 * c5 * t**4
    a = 2 * c2 + 6 * c3 * t + 12 * c4 * t**2 + 20 * c5 * t**3
    return x, v, a


class MinJerk:
    """Rest-to-rest (or rest-to-moving) minimum-jerk segment."""

    def __init__(self, x0, xT, T: float, v0=None, vT=None):
        x0 = np.asarray(x0, dtype=float)
        xT = np.asarray(xT, dtype=float)
        z = np.zeros_like(x0)
        v0 = z if v0 is None else np.asarray(v0, dtype=float)
        vT = z if vT is None else np.asarray(vT, dtype=float)
        self.T = max(float(T), 1e-3)
        self.coeffs = quintic_coeffs(x0, v0, z, xT, vT, z, self.T)
        self.xT, self.vT = xT, vT

    def at(self, t: float):
        """(x, v) at clamped time t; holds the endpoint after T."""
        x, v, _ = self.at3(t)
        return x, v

    def at3(self, t: float):
        """(x, v, a) at clamped time t; holds the endpoint after T."""
        if t >= self.T:
            return self.xT.copy(), self.vT.copy(), np.zeros_like(self.xT)
        t = max(t, 0.0)
        return quintic_eval(self.coeffs, t)
