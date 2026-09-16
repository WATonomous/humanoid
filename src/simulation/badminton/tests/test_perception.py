"""Perception gate: the trajectory prior is honest and converges.

1. traj_features reproduces the RK4 reference flight.
2. The EKF tracks a real flight from noisy 50 Hz position measurements.
3. The student's predicted trajectory is uncertain early and converges as
   measurements accumulate (the twitchy prior the policy must tolerate).
4. The torch batched port matches the numpy reference (skips without torch).
"""

import numpy as np
import pytest

import aero
import perception

P0 = np.array([0.5, 5.0, 2.0])
V0 = np.array([-1.0, -7.0, 1.5])
DT_TICK = 0.02   # 50 Hz control tick


@pytest.fixture(scope="module")
def params():
    return aero.load_params()


def _truth(params, t_max=1.2):
    k = params["shuttle"]["k"]
    t, P, V = aero.simulate(P0, V0, k, dt=1e-3, t_max=t_max,
                            stop_at_ground=False)
    per = int(round(DT_TICK / 1e-3))
    return P[::per], V[::per]


def test_traj_features_match_reference(params):
    k = params["shuttle"]["k"]
    pp = params["perception"]
    pts = perception.traj_features(P0, V0, k, pp["n_traj"], pp["traj_dt"],
                                   pp["traj_sub_dt"])
    t, P, V = aero.simulate(P0, V0, k, dt=1e-3,
                            t_max=pp["n_traj"] * pp["traj_dt"] + 1e-6,
                            stop_at_ground=False)
    for i in range(pp["n_traj"]):
        j = int(round((i + 1) * pp["traj_dt"] / 1e-3))
        err = np.linalg.norm(pts[i] - P[j])
        assert err < 1e-3, f"prior point {i}: {err*1000:.2f} mm off reference"


def test_ekf_converges(params):
    Pt, Vt = _truth(params)
    sim = perception.PerceptionSim(params, seed=3)
    sim.reset()
    for i in range(len(Pt)):
        sim.tick(Pt[i], DT_TICK)
    p_err = np.linalg.norm(sim.ekf.p_hat - Pt[-1])
    v_err = np.linalg.norm(sim.ekf.v_hat - Vt[-1])
    # steady-state position error sits just below sigma_meas (30 mm); the
    # prior's accuracy comes from the velocity estimate, tested below
    assert p_err < 0.03, f"position estimate {p_err*1000:.0f} mm off"
    assert v_err < 0.3, f"velocity estimate {v_err:.2f} m/s off"


def test_prior_converges_over_flight(params):
    """Error of the predicted position 0.4 s ahead, measured against truth,
    shrinks by >5x between the 3rd tick and half a second in. Averaged over
    seeds so a lucky early draw cannot pass a broken filter."""
    k = params["shuttle"]["k"]
    Pt, Vt = _truth(params)
    j_ahead = int(round(0.4 / DT_TICK))
    early_i, late_i = 2, 25

    def pred_err(sim, i):
        f = sim.features()
        p_hat, v_hat = f[:3], f[3:6]
        pts = perception.traj_features(p_hat, v_hat, k, 4, 0.1, 0.025)
        return np.linalg.norm(pts[3] - Pt[i + j_ahead])

    early, late = [], []
    for seed in range(8):
        sim = perception.PerceptionSim(params, seed=seed)
        sim.reset()
        for i in range(late_i + 1):
            sim.tick(Pt[i], DT_TICK)
            if i == early_i:
                early.append(pred_err(sim, i))
        late.append(pred_err(sim, late_i))
    early, late = np.mean(early), np.mean(late)
    assert late < 0.05, f"converged prior still {late*1000:.0f} mm off at 0.4 s ahead"
    assert early > 5 * late, \
        f"prior does not converge: early {early*1000:.0f} mm vs late {late*1000:.0f} mm"


def test_torch_port_matches_numpy(params):
    torch = pytest.importorskip("torch")
    import perception_torch as pt

    k = params["shuttle"]["k"]
    g = params["gravity"]
    pp = params["perception"]
    Pt, _ = _truth(params)

    sim = perception.PerceptionSim(params, seed=5)
    sim.reset()
    # identical measurement sequence for both implementations
    rng = np.random.default_rng(5)
    Z = Pt + rng.normal(0.0, pp["sigma_meas"], Pt.shape)
    sim.rng = np.random.default_rng(5)   # PerceptionSim draws the same Z

    z0 = torch.tensor(Z[0], dtype=torch.float64).unsqueeze(0)
    x, Pcov = pt.ekf_init(z0, pp["sigma_p0"], pp["sigma_v0"])
    sim.tick(Pt[0], DT_TICK)
    for i in range(1, len(Pt)):
        sim.tick(Pt[i], DT_TICK)
        x, Pcov = pt.ekf_predict(x, Pcov, DT_TICK, k, g, pp["sigma_acc"])
        z = torch.tensor(Z[i], dtype=torch.float64).unsqueeze(0)
        x, Pcov = pt.ekf_update(x, Pcov, z, pp["sigma_meas"])

    assert np.allclose(x[0].numpy(), sim.ekf.x, atol=1e-9)
    assert np.allclose(Pcov[0].numpy(), sim.ekf.P, atol=1e-9)

    feats_np = sim.features()
    feats_t = pt.feature_layout(x[:, :3], x[:, 3:], k, pp["n_traj"],
                                pp["traj_dt"], pp["traj_sub_dt"], g)
    assert np.allclose(feats_t[0].numpy(), feats_np, atol=1e-8)
