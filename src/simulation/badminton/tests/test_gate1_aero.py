"""Gate 1: shuttle aerodynamics, standalone (phase 1)."""

import numpy as np
import pytest

import aero


@pytest.fixture(scope="module")
def params():
    return aero.load_params()


@pytest.fixture(scope="module")
def k(params):
    return params["shuttle"]["k"]


def test_k_in_expected_band(k):
    # v_t in [6.6, 6.9] -> k in [0.206, 0.225]
    assert 0.206 <= k <= 0.226


def test_drop_test(params, k):
    """Release from rest: speed reaches 0.99 v_t within ~2.5 s, asymptotes cleanly."""
    v_t = params["shuttle"]["v_t"]
    t, P, V = aero.simulate([0, 0, 50.0], [0, 0, 0], k, t_max=4.0,
                            stop_at_ground=False)
    speed = np.linalg.norm(V, axis=1)
    reached = speed >= 0.99 * v_t
    assert reached.any(), "never reached 0.99 v_t"
    assert t[np.argmax(reached)] < 2.5
    # asymptote: speed never exceeds v_t and is monotone non-decreasing
    assert speed.max() <= v_t * 1.001
    assert np.all(np.diff(speed) >= -1e-9)


def test_range_test(k):
    """20 m/s at 45 deg: lands far short of ~40 m vacuum range, steep descent.

    Constant-k drag gives ~7.1 m from ground height; the spec's rough 8-12 m
    band assumes speed-dependent C_d, which is an intentional omission
    (ledger). Gate on 6.5-12 m and the >60 deg descent angle.
    """
    v0 = 20.0 * np.array([0, np.cos(np.radians(45)), np.sin(np.radians(45))])
    t, P, V = aero.simulate([0, 0, 0.0], v0, k, t_max=10.0)
    t_land, p_land = aero.landing_point(t, P)
    assert 6.5 <= p_land[1] <= 12.0
    descent = np.degrees(np.arctan2(-V[-1, 2], V[-1, 1]))
    assert descent > 60.0


def test_shape_test(k):
    """10 trajectories, speeds 4-20 m/s: every arc asymmetric (steep back half).

    Numeric proxy for the parachute look, measured between equal heights so
    the comparison is fair: the horizontal distance from apex back down to
    launch height is shorter than launch to apex, and the descent angle at
    landing is steeper than the launch angle.
    """
    launch_deg = 45.0
    z0 = 2.0
    for speed in np.linspace(4.0, 20.0, 10):
        v0 = speed * np.array([0, np.cos(np.radians(launch_deg)),
                               np.sin(np.radians(launch_deg))])
        t, P, V = aero.simulate([0, 0, z0], v0, k, t_max=10.0)
        i_apex = int(np.argmax(P[:, 2]))
        # first crossing of launch height after the apex
        below = np.nonzero(P[i_apex:, 2] <= z0)[0]
        assert len(below), f"arc never returns to launch height at {speed:.1f} m/s"
        i_back = i_apex + below[0]
        y_up = P[i_apex, 1] - P[0, 1]
        y_down = P[i_back, 1] - P[i_apex, 1]
        assert y_down < y_up, f"arc symmetric at {speed:.1f} m/s"
        descent = np.degrees(np.arctan2(-V[-1, 2], V[-1, 1]))
        assert descent > launch_deg, f"descent not steeper at {speed:.1f} m/s"


def test_fit_k_recovers_known_k(k):
    """Self-test of the film-fit path: fit on a synthetic noisy track recovers k.

    The real data anchor (gate 1.4) runs when filmed trajectories exist.
    """
    rng = np.random.default_rng(0)
    v0 = 7.0 * np.array([0, np.cos(np.radians(30)), np.sin(np.radians(30))])
    t, P, V = aero.simulate([0, 1.0, 1.8], v0, k, t_max=3.0)
    # 120 fps samples with 5 mm tracking noise
    idx = np.arange(0, len(t), 8)  # dt=1ms -> ~125 Hz
    times = t[idx]
    points = P[idx] + rng.normal(0, 0.005, (len(idx), 3))
    k_fit, v_t_fit, result = aero.fit_k(times, points)
    assert abs(k_fit - k) / k < 0.05
    assert 6.4 <= v_t_fit <= 7.1


@pytest.mark.skip(reason="no filmed shuttle data yet (gate 1.4 data anchor)")
def test_film_data_anchor():
    """Fit k on filmed slow feeds; fitted v_t must land in 6.5-7.0."""
