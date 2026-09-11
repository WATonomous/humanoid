"""Gate 3: inverse launcher (hittability by construction)."""

import mujoco
import numpy as np
import pytest

import aero
import launcher
import mjsim

N_SPECS = 1000


@pytest.fixture(scope="module")
def params():
    return aero.load_params()


@pytest.fixture(scope="module")
def workspace():
    if not launcher.workspace_exists():
        launcher.build_workspace(verbose=False)
    return launcher.load_workspace()


@pytest.fixture(scope="module")
def specs(workspace):
    return launcher.sample_specs(N_SPECS, seed=11, workspace=workspace)


def test_forward_verification(specs, params):
    """Gate 3.1: every generated launch, simulated in mujoco with contacts
    off and the arm untouched, arrives within 2 cm of p* at t*."""
    sim = mjsim.load()
    sim.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    dt = params["integrator"]["dt"]
    worst = 0.0
    for s in specs:
        sim.reset(p0=s.p0, v0=s.v0)
        for _ in range(int(round(s.t_star / dt))):
            sim.step()
        err = float(np.linalg.norm(sim.shuttle_pos - s.p_star))
        worst = max(worst, err)
        assert err < 0.02, f"launch misses p* by {err*1000:.1f} mm"
    print(f"worst forward-verification error: {worst*1000:.1f} mm")


def test_coverage(specs, workspace):
    """Gate 3.2: p* scatters across W instead of collapsing to a corner."""
    P = np.array([s.p_star for s in specs])
    span_w = workspace.box_hi - workspace.box_lo
    span_p = P.max(axis=0) - P.min(axis=0)
    for axis, name in enumerate("xyz"):
        ratio = span_p[axis] / span_w[axis]
        assert ratio > 0.5, f"{name} span only {ratio:.2f} of W"
    # all four x-z quadrants around the box center are populated
    center = (workspace.box_hi + workspace.box_lo) / 2
    qx = P[:, 0] > center[0]
    qz = P[:, 2] > center[2]
    for gx in (False, True):
        for gz in (False, True):
            frac = np.mean((qx == gx) & (qz == gz))
            assert frac > 0.02, f"x-z quadrant ({gx},{gz}) only {frac:.1%}"


def test_all_intercepts_inside_W(specs, workspace):
    inside = workspace.contains_many(np.array([s.p_star for s in specs]))
    assert inside.all()


def test_inbound_legality(specs, params):
    """Gate 3.3: every trajectory clears the net inbound with margin."""
    net_z = params["net"]["height_top"] + params["launcher"]["net_margin"]
    for s in specs:
        t, P, V = s.ref_traj
        y = P[:, 1]
        crossings = np.nonzero(np.diff(np.sign(y)) != 0)[0]
        assert len(crossings) == 1, "trajectory recrosses the net plane"
        i = crossings[0]
        frac = y[i] / (y[i] - y[i + 1])
        z_cross = P[i, 2] + frac * (P[i + 1, 2] - P[i, 2])
        assert z_cross >= net_z, f"net cleared by only {z_cross - params['net']['height_top']:.3f} m"


def test_launch_band(specs, params):
    """Launch states sit in the configured far-court band."""
    lp = params["launcher"]
    for s in specs:
        assert lp["launch_y_range"][0] <= s.p0[1] <= lp["launch_y_range"][1]
        assert lp["launch_z_range"][0] <= s.p0[2] <= lp["launch_z_range"][1]
        assert abs(s.p0[0]) <= lp["launch_x_max"]
        assert np.linalg.norm(s.v0) <= lp["max_launch_speed"]
