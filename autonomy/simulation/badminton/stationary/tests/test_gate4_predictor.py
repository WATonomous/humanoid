"""Gate 4: interception predictor accuracy on launcher episodes."""

import numpy as np
import pytest

import aero
import launcher
import predictor

N_EPISODES = 1000


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
    return launcher.sample_specs(N_EPISODES, seed=23, workspace=workspace)


def test_predictor_matches_episode_intercept(specs, params, workspace):
    """With the episode's own p* as the sweet point and clean state at t=0,
    the predictor recovers (p*, t*) to < 1 cm / < 5 ms. Coverage preference
    (a selection policy for the controller) is disabled: this row tests the
    trajectory + W-intersection mechanism, not intercept choice."""
    for s in specs:
        pred = predictor.predict(s.p0, s.v0, params, workspace=workspace,
                                 sweet_point=s.p_star,
                                 require_coverage=False)
        assert pred.ok, "predictor found no intercept on a launcher episode"
        p_err = float(np.linalg.norm(pred.p_star - s.p_star))
        t_err = abs(pred.t_star - s.t_star)
        assert p_err < 0.01, f"p̂* error {p_err*1000:.1f} mm"
        assert t_err < 0.005, f"t̂* error {t_err*1000:.1f} ms"


def test_predictor_default_sweet_point_stays_in_W(specs, params, workspace):
    """With the default sweet point the chosen intercept may differ from the
    episode's p*, but it must lie inside W for every episode."""
    n_no_intercept = 0
    for s in specs:
        pred = predictor.predict(s.p0, s.v0, params, workspace=workspace)
        if not pred.ok:
            n_no_intercept += 1
            continue
        assert workspace.contains(pred.p_star), "intercept outside W"
        assert pred.t_star > 0
    # launcher episodes are hittable by construction; count occurrences
    assert n_no_intercept == 0, f"{n_no_intercept} episodes with no intercept"


def test_predictor_no_intercept_flag(params, workspace):
    """A shuttle flying away from the arm returns the no-intercept flag."""
    pred = predictor.predict([0.0, 5.0, 1.5], [0.0, 8.0, 3.0], params,
                             workspace=workspace)
    assert not pred.ok
    assert pred.n_candidates == 0
