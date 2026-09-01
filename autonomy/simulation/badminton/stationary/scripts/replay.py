"""Replay logged eval episodes in the viewer, worst-first.

"why did we whiff episode 3172" becomes a 20-second question: episodes are
ranked worst-first in worst_ranked.npy (non-contacts first by miss distance).

Usage:
    uv run python scripts/replay.py runs/gate5 --rank 0     # worst episode
    uv run python scripts/replay.py runs/gate5 --episode 17 # specific episode

Keys: space pause, . single step, [ ] slow down / speed up, r restart,
n next-worst episode, q quit.
"""

import argparse
import json
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher
import mjsim


class Replayer:
    def __init__(self, run_dir):
        self.run_dir = run_dir
        self.sim = mjsim.load()
        self.order = np.load(os.path.join(run_dir, "worst_ranked.npy"))
        self.specs = launcher.load_specs(os.path.join(run_dir, "specs.npz"))
        with open(os.path.join(run_dir, "metrics.json")) as f:
            self.metrics = json.load(f)["episodes"]
        self.trajs = np.load(os.path.join(run_dir, "trajs.npz"))
        self.paused = False
        self.single = False
        self.dilation = 0.25
        self.frame = 0
        self.rank = 0

    def load_rank(self, rank):
        self.rank = rank % len(self.order)
        self.ep = int(self.order[self.rank])
        self.qpos = self.trajs[f"qpos_{self.ep}"]
        self.frame = 0
        m = self.metrics[self.ep]
        s = self.specs[self.ep]
        print(f"rank {self.rank}: episode {self.ep} "
              f"contact={m['contact']} min_dist={m['min_dist']:.3f} "
              f"ferr={m['ferr_at_that']:.3f} t*={s.t_star:.2f} "
              f"p*={np.round(s.p_star, 2)}")

    def key_cb(self, keycode):
        c = chr(keycode) if 32 <= keycode < 127 else None
        if keycode == 32:
            self.paused = not self.paused
        elif c == ".":
            self.single = True
        elif c == "[":
            self.dilation = max(0.05, self.dilation / 2)
        elif c == "]":
            self.dilation = min(1.0, self.dilation * 2)
        elif c in ("r", "R"):
            self.frame = 0
        elif c in ("n", "N"):
            self.load_rank(self.rank + 1)

    def run(self, start_rank=0, episode=None):
        if episode is not None:
            self.order = np.array([episode])
        self.load_rank(start_rank)
        sim = self.sim
        with mujoco.viewer.launch_passive(sim.model, sim.data,
                                          key_callback=self.key_cb) as v:
            v.cam.lookat[:] = self.specs[self.ep].p_star
            v.cam.azimuth, v.cam.elevation, v.cam.distance = 150, -15, 3.0
            while v.is_running():
                t0 = time.perf_counter()
                if not self.paused or self.single:
                    self.single = False
                    sim.data.qpos[:] = self.qpos[self.frame]
                    mujoco.mj_forward(sim.model, sim.data)
                    self.frame = (self.frame + 1) % len(self.qpos)
                v.sync()
                budget = sim.model.opt.timestep / self.dilation
                dt = time.perf_counter() - t0
                if dt < budget:
                    time.sleep(budget - dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--rank", type=int, default=0)
    ap.add_argument("--episode", type=int)
    args = ap.parse_args()
    Replayer(args.run_dir).run(start_rank=args.rank, episode=args.episode)


if __name__ == "__main__":
    main()
