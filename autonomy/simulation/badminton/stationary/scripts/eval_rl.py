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
    obs, _ = env.get_observations()
    rows = []  # (front_dist, z, x, hit)
    with torch.no_grad():
        while len(rows) < args.episodes:
            prev_p = store["p_star"].clone()
            prev_hit = store["hit"].clone()
            obs, _, dones, _ = env.step(policy(obs))
            done_ids = dones.nonzero(as_tuple=False).squeeze(-1)
            for i in done_ids.tolist():
                p = prev_p[i].cpu().numpy()
                rows.append((p[1] - base_y, p[2], p[0], bool(prev_hit[i])))
    r = np.array(rows[: args.episodes])
    front, z, x, hit = r[:, 0], r[:, 1], r[:, 2], r[:, 3].astype(bool)
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
    np.save("runs/eval_pstar_hits.npy", r)
    print("\nraw rows saved to runs/eval_pstar_hits.npy")


if __name__ == "__main__":
    main()
