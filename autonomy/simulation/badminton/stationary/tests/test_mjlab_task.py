"""mjlab task gate: the Warp env reproduces the CPU sim's physics.

Runs on CPU Warp (slow but exact w.r.t. the task's own wiring), so it works
on any machine with the train extras installed; skips when mjlab is absent.
Throughput and GPU-float drift are checked on the training machine.
"""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mjlab")

import mjsim  # noqa: E402


@pytest.fixture(scope="module")
def env():
    import badminton_mjlab  # noqa: F401
    from badminton_mjlab.env_cfg import make_env_cfg
    from mjlab.envs import ManagerBasedRlEnv

    cfg = make_env_cfg()
    cfg.scene.num_envs = 2
    return ManagerBasedRlEnv(cfg=cfg, device="cpu")


def _teacher_slices(env):
    # dict order: joint_pos(6), joint_vel(6), face_state(6), actions(6),
    # shuttle(6 + 3*n_traj), intercept(4)
    return slice(24, 30)


def test_obs_dims(env):
    obs, _ = env.reset()
    n_traj = env.command_manager.get_term("perception")._n_traj
    assert obs["student"].shape == (2, 24 + 6 + 3 * n_traj)
    assert obs["teacher"].shape == (2, 24 + 6 + 3 * n_traj + 4)


def test_flight_parity_vs_cpu_sim(env):
    """Zero-action rollout: the Warp shuttle (drag + orientation hooks in
    BadmintonAction) stays within 2 cm of the CPU reference sim at 0.6 s."""
    obs, _ = env.reset()
    pv = obs["teacher"][0, _teacher_slices(env)].numpy()
    p0, v0 = pv[:3].copy(), pv[3:].copy()

    sim = mjsim.load()
    sim.reset(p0=p0, v0=v0)   # arm at the ready keyframe, same as the env
    n_ticks = 30              # 0.6 s at 50 Hz
    act = torch.zeros(2, 6)
    for _ in range(n_ticks):
        obs, *_ = env.step(act)
        for _ in range(env.cfg.decimation):
            sim.step()
    pv = obs["teacher"][0, _teacher_slices(env)].numpy()
    err = np.linalg.norm(pv[:3] - sim.shuttle_pos)
    assert err < 0.02, f"flight divergence {err * 1000:.1f} mm at 0.6 s"


def test_cork_orientation_tracks_velocity(env):
    """The kinematic orientation lag runs on-device: after free flight the
    shuttle body +z (cork axis) aligns with the velocity direction."""
    env.reset()
    act = torch.zeros(2, 6)
    for _ in range(20):   # 0.4 s, several times orient_tau
        env.step(act)
    shuttle = env.scene["shuttle"]
    idx = shuttle.data.indexing
    # check the faster shuttle: near the apex the direction turns quickly
    # and the first-order lag legitimately trails it
    from badminton_mjlab.shuttle_action import quat_z_axis
    q = shuttle.data.data.qpos[:, idx.free_joint_q_adr[3:7]]
    v = shuttle.data.data.qvel[:, idx.free_joint_v_adr[:3]]
    i = int(v.norm(dim=-1).argmax())
    z = quat_z_axis(q[i].unsqueeze(0))[0]
    v_hat = v[i] / v[i].norm()
    cos = float((z * v_hat).sum())
    assert cos > 0.9, f"cork axis vs velocity: cos {cos:.3f}"
    w = shuttle.data.data.qvel[i, idx.free_joint_v_adr[3:6]]
    # zeroed each substep; only float32 integration dust may remain
    assert float(w.norm()) < 1e-6, "angular velocity must stay zeroed"


def test_ekf_converges_in_env(env):
    obs, _ = env.reset()
    act = torch.zeros(2, 6)
    for _ in range(20):
        obs, *_ = env.step(act)
    sl = _teacher_slices(env)
    true_pv = obs["teacher"][:, sl]
    ekf_pv = obs["student"][:, sl]
    p_err = (true_pv[:, :3] - ekf_pv[:, :3]).norm(dim=-1).max()
    v_err = (true_pv[:, 3:] - ekf_pv[:, 3:]).norm(dim=-1).max()
    assert float(p_err) < 0.08, f"EKF position {float(p_err) * 1000:.0f} mm off"
    assert float(v_err) < 0.6, f"EKF velocity {float(v_err):.2f} m/s off"


def test_tasks_registered():
    import badminton_mjlab  # noqa: F401
    from mjlab.tasks.registry import list_tasks

    tasks = list_tasks()
    assert "Mjlab-Badminton-Receive-Teacher" in tasks
    assert "Mjlab-Badminton-Receive-Student" in tasks
