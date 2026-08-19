"""Shared episode runner + metrics for gate 5 (used by run_gate5_eval.py,
replay logging, and the gate-5 pytest)."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field

import numpy as np

import launcher
import mjsim
from baseline.fsm import Controller


@dataclass
class EpisodeResult:
    contact: bool = False
    min_dist: float = np.inf            # min face-shuttle distance
    contact_point_face: list = field(default_factory=lambda: [np.nan] * 3)
    contact_rel_speed: float = np.nan
    net_cross_z: float = np.nan         # return crossing height (nan: none)
    cleared_net: bool = False
    landing: list = field(default_factory=lambda: [np.nan] * 2)
    ferr_at_that: float = np.nan        # ||face - p̂*|| at t̂*
    cross_terr: float = np.nan          # plane-crossing time - t̂*
    cross_perp: float = np.nan          # cross-track offset at plane crossing
    limit_hits: int = 0
    self_collision_steps: int = 0       # ticks with deep arm self-collision
    t_star: float = np.nan


def run_episode(sim, ctrl, spec, record=False, horizon_s: float = 2.5):
    """Run one launcher episode under the FSM. Returns (EpisodeResult, traj)
    where traj is (qpos_history, qvel_history) if record else None."""
    m = sim.model
    r = EpisodeResult(t_star=spec.t_star)
    sim.reset(p0=spec.p0, v0=spec.v0, arm_q=ctrl.q_home)
    ctrl.reset()
    n_steps = int(round(horizon_s / m.opt.timestep))
    qpos_h = np.empty((n_steps, m.nq)) if record else None
    qvel_h = np.empty((n_steps, m.nv)) if record else None
    lo = m.jnt_range[sim.arm_jids, 0] + 1e-4
    hi = m.jnt_range[sim.arm_jids, 1] - 1e-4
    prev_s = None
    steps = 0
    net_top = sim.params["net"]["height_top"]
    for i in range(n_steps):
        ctrl.update()
        sim.step()
        t = (i + 1) * m.opt.timestep
        if record:
            qpos_h[i] = sim.data.qpos
            qvel_h[i] = sim.data.qvel
        steps = i + 1
        d = float(np.linalg.norm(sim.face_pos - sim.shuttle_pos))
        r.min_dist = min(r.min_dist, d)
        q = sim.arm_qpos
        if np.any(q <= lo) or np.any(q >= hi):
            r.limit_hits += 1
        if i % ctrl.steps_per_tick == 0 and \
                ctrl._collides(q, min_dist=-ctrl.col_depth):
            r.self_collision_steps += 1
        if ctrl.t_hat_abs is not None and abs(t - ctrl.t_hat_abs) < \
                m.opt.timestep * 0.6 and np.isnan(r.ferr_at_that):
            r.ferr_at_that = float(np.linalg.norm(sim.face_pos - ctrl.p_hat))
        if (ctrl.p_hat is not None and ctrl.state in ("SWING", "FOLLOW")
                and np.isnan(r.cross_terr)):
            rel = sim.face_pos - ctrl.p_hat
            s_along = float(rel @ ctrl.u_out)
            if prev_s is not None and prev_s < 0 <= s_along:
                r.cross_terr = t - ctrl.t_hat_abs
                r.cross_perp = float(np.linalg.norm(rel - s_along * ctrl.u_out))
            prev_s = s_along
        if sim.face_contact() and not r.contact:
            r.contact = True
            rel_v = sim.shuttle_vel - sim.face_vel
            r.contact_rel_speed = float(np.linalg.norm(rel_v))
            for ci in range(sim.data.ncon):
                c = sim.data.contact[ci]
                if sim.face_gid in (c.geom1, c.geom2):
                    R = sim.data.site_xmat[sim.face_sid].reshape(3, 3)
                    r.contact_point_face = list(
                        R.T @ (c.pos - sim.face_pos))
                    break
        if r.contact and np.isnan(r.net_cross_z) and sim.shuttle_pos[1] > 0:
            r.net_cross_z = float(sim.shuttle_pos[2])
            r.cleared_net = r.net_cross_z > net_top
        p = sim.shuttle_pos
        if p[2] < 0.03:
            r.landing = list(p[:2])
            break
    traj = (qpos_h[:steps].copy(), qvel_h[:steps].copy()) if record else None
    return r, traj


def summarize(results: list[EpisodeResult]) -> dict:
    n = len(results)
    contacts = [r for r in results if r.contact]
    ferr = np.array([r.ferr_at_that for r in results])
    terr = np.array([r.cross_terr for r in results])
    out = {
        "episodes": n,
        "contact_rate": len(contacts) / n,
        "net_clearance_of_contacts":
            (np.mean([r.cleared_net for r in contacts]) if contacts else 0.0),
        "median_face_pos_err_at_t_star":
            float(np.nanmedian(ferr)),
        "median_face_timing_err":
            float(np.nanmedian(np.abs(terr))),
        "median_cross_track": float(np.nanmedian(
            [r.cross_perp for r in results])),
        "limit_hit_episodes": int(sum(r.limit_hits > 0 for r in results)),
        "self_collision_episodes":
            int(sum(r.self_collision_steps > 0 for r in results)),
        "median_min_dist": float(np.median([r.min_dist for r in results])),
        "median_contact_rel_speed":
            float(np.nanmedian([r.contact_rel_speed for r in contacts]))
            if contacts else float("nan"),
    }
    return out


def run_eval(n_episodes: int, seed: int = 0, record: bool = False,
             out_dir: str | None = None, verbose: bool = True):
    """Run the gate-5 eval. Returns (summary, results). With out_dir set,
    writes metrics.json, per-episode metrics, ranked worst list, and (with
    record) qpos/qvel trajectories for replay.py."""
    sim = mjsim.load()
    w = launcher.load_workspace()
    ctrl = Controller(sim, w)
    specs = launcher.sample_specs(n_episodes, seed=seed, workspace=w)
    results, trajs = [], []
    for i, spec in enumerate(specs):
        r, traj = run_episode(sim, ctrl, spec, record=record)
        results.append(r)
        trajs.append(traj)
        if verbose and (i + 1) % 50 == 0:
            done = [x for x in results if x.contact]
            print(f"  {i+1}/{n_episodes} contact_rate={len(done)/(i+1):.2f}")
    summary = summarize(results)

    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "metrics.json"), "w") as f:
            json.dump({"summary": summary,
                       "episodes": [asdict(r) for r in results]}, f, indent=1)
        # rank worst: non-contacts first (by min_dist desc), then by ferr
        order = sorted(
            range(len(results)),
            key=lambda i: (results[i].contact,
                           -results[i].min_dist if not results[i].contact
                           else -(results[i].ferr_at_that or 0)))
        np.save(os.path.join(out_dir, "worst_ranked.npy"), np.array(order))
        launcher.save_specs(specs, os.path.join(out_dir, "specs.npz"))
        if record:
            np.savez_compressed(
                os.path.join(out_dir, "trajs.npz"),
                **{f"qpos_{i}": t[0] for i, t in enumerate(trajs)},
                **{f"qvel_{i}": t[1] for i, t in enumerate(trajs)})
    return summary, results
