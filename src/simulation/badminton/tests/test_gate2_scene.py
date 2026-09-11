"""Gate 2: mujoco scene parity, bounce calibration, net, mesh alignment."""

import os

import mujoco
import numpy as np
import pytest

import aero
import mjsim

RNG = np.random.default_rng(7)


@pytest.fixture(scope="module")
def params():
    return aero.load_params()


@pytest.fixture()
def sim():
    return mjsim.load()


def test_scene_sanity(sim, params):
    m = sim.model
    assert m.nu == 6
    assert m.npair == 4
    assert m.opt.timestep == params["integrator"]["dt"]
    assert abs(float(m.body_subtreemass[m.body("racket").id])
               - params["racket"]["total_mass"]) < 0.01
    # face reachable band at home pose
    assert 0.6 <= sim.face_pos[2] <= 2.0


def test_parity_50_launches(sim, params):
    """Gate 2.1: mujoco flight matches the RK4 reference within 2 cm at 1.5 s."""
    k, dt = sim.k, params["integrator"]["dt"]
    t_check = 1.5
    n_steps = int(round(t_check / dt))
    for trial in range(50):
        speed = RNG.uniform(4.0, 20.0)
        ang = np.radians(RNG.uniform(10.0, 70.0))
        yaw = np.radians(RNG.uniform(-30.0, 30.0))
        v0 = speed * np.array([np.cos(ang) * np.sin(yaw),
                               np.cos(ang) * np.cos(yaw),
                               np.sin(ang)])
        # launch high enough to stay airborne for the full 1.5 s window
        # (max fall in 1.5 s is bounded by v_t), and away from the arm so
        # no contact interferes
        p0 = np.array([RNG.uniform(-1, 1), RNG.uniform(1.0, 3.0),
                       RNG.uniform(9.0, 10.0)])
        sim.reset(p0=p0, v0=v0)
        for _ in range(n_steps):
            sim.step()
        p_mj = sim.shuttle_pos
        t, P, V = aero.simulate(p0, v0, k, dt=dt, t_max=t_check + dt,
                                stop_at_ground=False)
        p_ref = P[n_steps]
        err = np.linalg.norm(p_mj - p_ref)
        assert err < 0.02, f"trial {trial}: parity error {err*1000:.1f} mm"


def _bounce_rig_xml(params):
    fs = params["racket"]["face_size"]
    solref = " ".join(str(x) for x in params["contact"]["face_solref"])
    solimp = " ".join(str(x) for x in params["contact"]["face_solimp"])
    # the face pair contacts the cork sphere (full shuttle mass behind it)
    r = params["shuttle"]["cork_radius"]
    m = params["shuttle"]["mass"]
    return f"""
<mujoco>
  <option timestep="{params["integrator"]["dt"]}" gravity="0 0 -{params["gravity"]}" integrator="{params["integrator"]["method"]}"/>
  <worldbody>
    <geom name="face" type="box" size="{fs[0]/2} {fs[1]/2} {fs[2]/2}"
          pos="0 0 0.5" contype="0" conaffinity="0"/>
    <body name="shuttle" pos="0 0 0.7">
      <freejoint/>
      <geom name="shuttle_col" type="sphere" size="{r}" mass="{m}"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="shuttle_col" geom2="face" solref="{solref}" solimp="{solimp}"/>
  </contact>
</mujoco>"""


def _drop(model, x, y, v_in, max_steps=600):
    """Drop the rig shuttle at (x, y) with speed v_in onto the face.

    Returns (e, contacted, tunneled)."""
    data = mujoco.MjData(model)
    data.qpos[0], data.qpos[1] = x, y
    data.qvel[2] = -v_in
    v_low, touched = 0.0, False
    for _ in range(max_steps):
        mujoco.mj_step(model, data)
        if data.ncon:
            touched = True
        v_low = min(v_low, data.qvel[2])
        if data.qpos[2] < 0.45:
            return 0.0, touched, True
        if touched and data.ncon == 0 and data.qvel[2] > 0:
            return data.qvel[2] / -v_low, True, False
    return 0.0, touched, False


def test_bounce_restitution(params):
    """Gate 2.2a: apex restitution e in [0.7, 0.8] on a static face."""
    model = mujoco.MjModel.from_xml_string(_bounce_rig_xml(params))
    e, contacted, tunneled = _drop(model, 0.0, 0.0, 3.0)
    assert contacted and not tunneled
    assert 0.7 <= e <= 0.8, f"e = {e:.3f}"


def test_bounce_sweep_no_tunneling(params):
    """Gate 2.2b: 200 trials at 5 and 8 m/s closing speed; contact every
    trial, zero tunneling. Contact points scatter across the face."""
    model = mujoco.MjModel.from_xml_string(_bounce_rig_xml(params))
    fs = params["racket"]["face_size"]
    n_fail_contact = n_tunnel = 0
    for i in range(200):
        v = 5.0 if i % 2 == 0 else 8.0
        # scatter across the face, 1 cm inside the edge
        x = RNG.uniform(-fs[0] / 2 + 0.01, fs[0] / 2 - 0.01)
        y = RNG.uniform(-fs[1] / 2 + 0.01, fs[1] / 2 - 0.01)
        e, contacted, tunneled = _drop(model, x, y, v)
        n_fail_contact += not contacted
        n_tunnel += tunneled
    assert n_fail_contact == 0, f"{n_fail_contact} trials without contact"
    assert n_tunnel == 0, f"{n_tunnel} trials tunneled"


def test_net_stops_shuttle(sim):
    """Gate 2.3: shuttle into the net at 3-6 m/s dies plausibly."""
    net_top = sim.params["net"]["height_top"]
    for speed in [3.0, 4.5, 6.0]:
        # level shot from 0.5 m out so drag can't drop it under the band
        p0 = np.array([0.0, 0.5, 1.3])
        v0 = np.array([0.0, -speed, 0.0])
        sim.reset(p0=p0, v0=v0)
        max_speed_after = 0.0
        hit_net = False
        for _ in range(2500):
            sim.step()
            if sim.net_gid in sim.contacts():
                hit_net = True
            if hit_net:
                max_speed_after = max(max_speed_after,
                                      np.linalg.norm(sim.shuttle_vel))
            p = sim.shuttle_pos
            if p[2] < 0.05:
                break
        assert hit_net, f"never touched the net at {speed} m/s"
        # no pass-through: dies on the incoming side, near the net
        assert p[1] > -0.15, f"passed through the net at {speed} m/s (y={p[1]:.2f})"
        assert p[1] < 1.0, f"bounced implausibly far at {speed} m/s (y={p[1]:.2f})"
        # no explosion
        assert max_speed_after < speed * 1.5 + 2.0


def test_mesh_alignment(params):
    """Gate 2.4 numeric version: face box inside the string-bed silhouette;
    shuttle sphere envelops the skirt ring with the cork outside."""
    trimesh = pytest.importorskip("trimesh")
    assets = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "scene", "assets")
    fs = params["racket"]["face_size"]

    racket = trimesh.load(os.path.join(assets, "racket_vis.obj"),
                          force="mesh")
    # hoop inner ellipse from the mesh bounds (hoop tube included)
    hx = racket.bounds[1][0]           # outer half width
    hy = racket.bounds[1][1]           # outer half length (head side, y > 0)
    tube = 0.007
    ax, ay = hx - 2 * tube, hy - 2 * tube
    # every face-box corner inside the inner ellipse (padding through the
    # thickness is expected and allowed)
    corner = (fs[0] / 2 / ax) ** 2 + (fs[1] / 2 / ay) ** 2
    assert corner < 1.0, f"face box corner outside string bed ({corner:.2f})"

    shuttle = trimesh.load(os.path.join(assets, "shuttle_vis.obj"),
                           force="mesh")
    r = params["shuttle"]["radius"]
    verts = shuttle.vertices
    ring = verts[verts[:, 2] < 0.005]
    assert len(ring) > 0
    assert np.linalg.norm(ring, axis=1).max() <= r + 1e-6, \
        "skirt ring pokes out of the collision sphere"
    # the cork sphere surface must sit at the mesh cork tip so a cork-first
    # arrival contacts the face where the mesh shows it
    cork_tip = verts[np.argmax(verts[:, 2])]
    cork_c = np.array([0.0, 0.0, params["shuttle"]["cork_center_z"]])
    cork_r = params["shuttle"]["cork_radius"]
    tip_gap = abs(np.linalg.norm(cork_tip - cork_c) - cork_r)
    assert tip_gap < 2e-3, \
        f"cork sphere surface {tip_gap*1000:.1f} mm off the mesh cork tip"


def test_shuttle_face_contact_in_scene(sim):
    """Contact between shuttle and the mounted racket registers in the full
    scene (pairs wired correctly after the arm attach)."""
    # fire the shuttle straight at the face in its ready (keyframe) pose;
    # reset() zeroes the arm, so hold it at the ready pose explicitly
    arm_q = sim.arm_qpos
    face_p = sim.face_pos
    n = sim.face_normal
    # 4 m/s: fast enough that gravity drop over the 0.3 m approach (~3 cm)
    # stays well inside the face half-height
    p0 = face_p + 0.3 * n
    sim.reset(p0=p0, v0=-4.0 * n, arm_q=arm_q)
    hit = False
    for _ in range(1500):
        sim.step()
        if sim.face_contact():
            hit = True
            break
    assert hit, "shuttle never contacted the racket face in the scene"
