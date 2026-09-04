"""Bank-wide policy eval: hit rate vs where the intercept point sits.

  uv run scripts/eval_rl.py --checkpoint-file <model.pt> [--episodes 4096]

Rolls the policy over the launcher bank and reports contact rate overall
and binned by p*'s distance in front of the chest plane (y - arm.base_y)
and by height, to test whether body-line intercepts are the miss cluster.
A hit that lands on the very tick the episode terminates is not counted
(the reset clears the latch first); this undercounts by a negligible
amount.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import asdict

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import badminton_mjlab  # noqa: F401  (registers the tasks)
from badminton_mjlab import mdp
import aero
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

TASK = "Mjlab-Badminton-Receive-Teacher"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint-file", required=True)
    ap.add_argument("--episodes", type=int, default=4096)
    ap.add_argument("--num-envs", type=int, default=1024)
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()

    env_cfg = load_env_cfg(TASK)
    env_cfg.scene.num_envs = args.num_envs
    agent_cfg = load_rl_cfg(TASK)
    env = ManagerBasedRlEnv(cfg=env_cfg, device=args.device)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner_cls = load_runner_cls(TASK) or MjlabOnPolicyRunner
    runner = runner_cls(env, asdict(agent_cfg), device=args.device)
    runner.load(args.checkpoint_file, load_cfg={"actor": True}, strict=True,
                map_location=args.device)
    policy = runner.get_inference_policy(device=args.device)

    base_y = aero.load_params()["arm"]["base_y"]
    store = env.unwrapped._badminton
    obs = env.get_observations()
    feas = env.unwrapped.command_manager.get_term("feasibility")
    uenv = env.unwrapped
    from mjlab.managers.scene_entity_config import SceneEntityCfg
    fcfg = SceneEntityCfg("robot", site_names=("face_center",))
    fcfg.resolve(uenv.scene)
    robot = uenv.scene["robot"]

    def face_pose():
        pos = robot.data.site_pos_w[:, fcfg.site_ids].squeeze(1)
        mat = robot.data.data.site_xmat[:, robot.data.indexing.site_ids]
        mat = mat[:, fcfg.site_ids].squeeze(1).reshape(-1, 3, 3)
        return pos, mat

    n = uenv.num_envs
    dev = uenv.device
    # where the shuttle passes the racket, in the face frame (u, v): the
    # shuttle position at its closest approach to the face centre, frozen at
    # the contact tick for hits (a hit rebounds before it can cross the face
    # plane, and the infinite plane is crossed far from the racket whenever
    # the face rotates during the swing, so a plane-crossing test is useless).
    # For misses the closest approach says where the racket would have had
    # to be. Also kept: the closest-approach distance itself.
    shuttle = uenv.scene["shuttle"]
    sidx = shuttle.data.indexing
    cork_z = float(aero.load_params()["shuttle"]["cork_center_z"])

    def cork_pos():
        # cork collision-sphere centre: the shuttle origin is ~7 cm up the
        # skirt, so with the shuttle oblique to the face the origin's
        # in-plane offset is not where the cork struck
        q = shuttle.data.data.qpos[:, sidx.free_joint_q_adr]
        pos, w, x, y, z = q[:, :3], q[:, 3], q[:, 4], q[:, 5], q[:, 6]
        zaxis = torch.stack([2 * (x * z + w * y), 2 * (y * z - w * x),
                             1 - 2 * (x * x + y * y)], dim=-1)
        return pos + cork_z * zaxis

    near_uv = torch.full((n, 2), float("nan"), device=dev)
    near_d = torch.full((n,), float("inf"), device=dev)
    # predicted landing of the return, at the hit tick: distance to the
    # launch origin (the run-12 target zone) and net-clearance flag
    land_err = torch.full((n,), float("nan"), device=dev)
    land_ok = torch.zeros(n, dtype=torch.bool, device=dev)

    rows = []  # (front_dist, z, x, hit, u, v, d_min, land_err, land_ok)
    qv_rows, tau_rows, duty_rows = [], [], []  # per-episode, per-joint
    with torch.no_grad():
        while len(rows) < args.episodes:
            prev_p = store["p_star"].clone()
            prev_hit = store["hit"].clone()
            prev_qv = feas._qvel_peak.clone()
            prev_tau = feas._tau_peak.clone()
            prev_duty = (feas._over / feas._ticks.clamp_min(1.0)).clone()
            prev_uv = near_uv.clone()
            prev_d = near_d.clone()
            prev_land = land_err.clone()
            prev_ok = land_ok.clone()
            obs, _, dones, _ = env.step(policy(obs))
            done = dones.bool()
            # closest approach so far; stop updating once the face has hit
            fpos, fmat = face_pose()
            cpos = cork_pos()
            local = torch.einsum("nij,ni->nj", fmat, cpos - fpos)
            d = local.norm(dim=-1)
            closer = (d < near_d) & (~prev_hit) & (~done)
            near_d = torch.where(closer, d, near_d)
            near_uv[closer] = local[closer, :2]
            first = store["first"] & (~done)
            if bool(first.any()):
                spos, svel = mdp._shuttle_state(uenv)
                xy, ok = mdp.predict_landing(spos[first], svel[first])
                land_err[first] = (
                    (xy - store["p0_xy"][first]).norm(dim=-1))
                land_ok[first] = ok
            done_ids = done.nonzero(as_tuple=False).squeeze(-1)
            for i in done_ids.tolist():
                p = prev_p[i].cpu().numpy()
                uv = prev_uv[i].cpu().numpy()
                rows.append((p[1] - base_y, p[2], p[0], bool(prev_hit[i]),
                             float(uv[0]), float(uv[1]), float(prev_d[i]),
                             float(prev_land[i]), float(prev_ok[i])))
            near_uv[done] = float("nan")
            near_d[done] = float("inf")
            land_err[done] = float("nan")
            land_ok[done] = False
            if len(done_ids):
                qv_rows.append(prev_qv[done_ids].cpu().numpy())
                tau_rows.append(prev_tau[done_ids].cpu().numpy())
                duty_rows.append(prev_duty[done_ids].cpu().numpy())
    r = np.array(rows[: args.episodes], dtype=float)
    front, z, x, hit = r[:, 0], r[:, 1], r[:, 2], r[:, 3].astype(bool)
    uv, dmin = r[:, 4:6], r[:, 6]
    land, lok = r[:, 7], r[:, 8].astype(bool)
    has_land = np.isfinite(land)
    if has_land.any():
        print(f"return lands on far court with net clearance: "
              f"{lok[has_land].mean():.3f} of hits; landing error to launch "
              f"origin, cleared returns: median "
              f"{np.median(land[has_land & lok]):.2f} m, p90 "
              f"{np.percentile(land[has_land & lok], 90):.2f} m")
    print(f"closest approach to face centre, hits: median {np.median(dmin[hit]) * 100:.1f} cm"
          f"   misses: median {np.median(dmin[~hit]) * 100:.1f} cm,"
          f" p90 {np.percentile(dmin[~hit], 90) * 100:.1f} cm")
    print(f"\nepisodes: {len(r)}   overall hit rate: {hit.mean():.3f}")

    def table(label, v, edges):
        print(f"\n{label}")
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (v >= lo) & (v < hi)
            if m.sum() == 0:
                continue
            print(f"  [{lo:5.2f}, {hi:5.2f})  n={m.sum():5d}  hit {hit[m].mean():.3f}")

    table("hit rate by p* distance in front of chest plane (m)", front,
          [0.0, 0.15, 0.25, 0.35, 0.45, 0.6, 2.0])
    table("hit rate by p* height (m)", z, [0.0, 0.9, 1.1, 1.3, 1.5, 1.7, 3.0])
    table("hit rate by p* lateral x (m)", x, [-2.0, -0.4, -0.2, 0.0, 0.2, 0.4, 2.0])
    qv = np.concatenate(qv_rows)[: args.episodes]
    tau = np.concatenate(tau_rows)[: args.episodes]
    duty = np.concatenate(duty_rows)[: args.episodes]
    arm = aero.load_params()["arm"]
    peak, rated = arm["torque_limits"], arm["torque_rated"]
    print("\nfeasibility per joint (per-episode peaks; median / p95 / max)")
    print("  joint  |qvel| rad/s            |tau| Nm  (clamp)   at-clamp eps  duty>rated (mean)")
    for j in range(qv.shape[1]):
        q = np.percentile(qv[:, j], [50, 95, 100])
        t = np.percentile(tau[:, j], [50, 95, 100])
        at_clamp = (tau[:, j] >= 0.99 * peak[j]).mean()
        print(f"  j{j + 1}    {q[0]:5.1f} / {q[1]:5.1f} / {q[2]:5.1f}"
              f"    {t[0]:5.2f} / {t[1]:5.2f} / {t[2]:5.2f} ({peak[j]:5.2f})"
              f"   {at_clamp:5.1%}        {duty[:, j].mean():5.1%} (rated {rated[j]})")
    np.save("runs/eval_pstar_hits.npy", r)  # cols: front,z,x,hit,u,v,d_min,land_err,land_ok
    np.savez("runs/eval_feasibility.npz", qvel_peak=qv, tau_peak=tau, duty=duty)
    print("\nraw rows saved to runs/eval_pstar_hits.npy")


if __name__ == "__main__":
    main()
