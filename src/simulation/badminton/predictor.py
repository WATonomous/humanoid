"""Interception predictor: a pure function over (shuttle state, params).

Forward-integrates the drag model from the given state, intersects the
predicted trajectory with the workspace W, and selects the intercept point
closest to a nominal sweet point. Used by the scripted baseline now; the same
function becomes a policy observation later, so it takes state in and returns
data out with no side effects.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import aero
import launcher


@dataclass
class Prediction:
    ok: bool
    p_star: np.ndarray | None = None   # p̂*: chosen intercept point
    t_star: float | None = None        # t̂*: time from now until intercept
    v_in: np.ndarray | None = None     # predicted shuttle velocity at p̂*
    n_candidates: int = 0              # trajectory samples inside W
    # every in-W sample, ranked as the default selection would (best first):
    # rows of (index into the arrays below), plus the raw arrays themselves
    cand_p: np.ndarray | None = None   # (n, 3) positions, ranked
    cand_t: np.ndarray | None = None   # (n,) times, ranked
    cand_v: np.ndarray | None = None   # (n, 3) velocities, ranked


_DEFAULT = object()


def predict(p, v, params: dict, workspace: launcher.Workspace | None = None,
            sweet_point=_DEFAULT, horizon: float = 2.5,
            require_coverage: bool = True) -> Prediction:
    """Predict the intercept for shuttle state (p, v) at the current instant.

    sweet_point defaults to launcher.sweet_point from params; pass an explicit
    point to bias selection elsewhere (gate 4 uses the episode's own p*).
    require_coverage prefers candidates with many workspace configs nearby
    (robustly reachable); disable it to recover an exact known crossing.
    Returns ok=False if the predicted trajectory never enters W.
    """
    k = params["shuttle"]["k"]
    dt = params["integrator"]["dt"]
    w = workspace or _default_workspace()
    if sweet_point is _DEFAULT:
        sweet = np.array(params["launcher"]["sweet_point"])
    else:
        sweet = np.asarray(sweet_point, dtype=float)

    t, P, V = aero.simulate(p, v, k, dt=dt, t_max=horizon, stop_at_ground=True)
    inside = w.contains_many(P)
    n_in = int(inside.sum())
    if n_in == 0:
        return Prediction(ok=False, n_candidates=0)
    idx = np.nonzero(inside)[0]
    if require_coverage:
        # candidates at the thin edge of the cloud (few configs nearby) are
        # only marginally reachable; restrict to well-covered points
        counts = np.array([len(w.tree.query_ball_point(P[i], w.radius))
                           for i in idx])
        for min_count in (20, 8):
            solid = counts >= min_count
            if solid.any():
                idx = idx[solid]
                break
    d2 = ((P[idx] - sweet) ** 2).sum(axis=1)
    order = idx[np.argsort(d2)]
    # thin the ranked list: adjacent trajectory samples are mm apart, and a
    # caller trying candidates in order wants distinct places, not dt-steps
    ranked = [order[0]]
    for i in order[1:]:
        if all(np.linalg.norm(P[i] - P[j]) > 0.08 for j in ranked):
            ranked.append(i)
        if len(ranked) >= 8:
            break
    ranked = np.array(ranked)
    best = ranked[0]
    return Prediction(ok=True, p_star=P[best].copy(), t_star=float(t[best]),
                      v_in=V[best].copy(), n_candidates=n_in,
                      cand_p=P[ranked].copy(), cand_t=t[ranked].copy(),
                      cand_v=V[ranked].copy())


_W_CACHE: dict = {}


def _default_workspace() -> launcher.Workspace:
    if "w" not in _W_CACHE:
        _W_CACHE["w"] = launcher.load_workspace()
    return _W_CACHE["w"]
