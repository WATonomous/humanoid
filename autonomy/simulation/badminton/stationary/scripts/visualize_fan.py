"""Gate 3.4 fan plot: render launcher trajectories over the court.

Side view (y-z) and top view (y-x) of 50 sampled episodes, with the net, the
W cloud silhouette, and the intercept points marked. Saved as a PNG; step
through the same fan interactively in sim_lab (the W overlay renders there).

Usage: uv run python scripts/visualize_fan.py [-n 50] [-o fan.png]
"""

import argparse
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import aero
import launcher


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=50)
    ap.add_argument("-o", default="fan.png")
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args()

    params = aero.load_params()
    if not launcher.workspace_exists():
        launcher.build_workspace()
    w = launcher.load_workspace()
    specs = launcher.sample_specs(args.n, seed=args.seed, workspace=w)

    fig, (ax_side, ax_top) = plt.subplots(2, 1, figsize=(12, 9))
    net = params["net"]

    # side view: y-z
    for s in specs:
        t, P, V = s.ref_traj
        ax_side.plot(P[:, 1], P[:, 2], lw=0.8, alpha=0.7)
        ax_side.plot(*s.p_star[[1, 2]], "r.", ms=5)
    step = max(1, len(w.points) // 800)
    ax_side.plot(w.points[::step, 1], w.points[::step, 2], ".",
                 c="tab:green", ms=2, alpha=0.25, label="W cloud")
    ax_side.plot([0, 0], [net["height_bottom"], net["height_top"]],
                 "k-", lw=3, label="net")
    ax_side.axhline(0, c="gray", lw=0.5)
    ax_side.set_xlabel("y [m] (arm at y=-2, launch band y>3)")
    ax_side.set_ylabel("z [m]")
    ax_side.set_title(f"fan of {args.n} launcher episodes, side view "
                      "(red dots: p*)")
    ax_side.legend(loc="upper right")
    ax_side.set_aspect("equal")

    # top view: y-x
    for s in specs:
        t, P, V = s.ref_traj
        ax_top.plot(P[:, 1], P[:, 0], lw=0.8, alpha=0.7)
        ax_top.plot(*s.p_star[[1, 0]], "r.", ms=5)
    ax_top.plot(w.points[::step, 1], w.points[::step, 0], ".",
                c="tab:green", ms=2, alpha=0.25)
    cw = params["court"]["width"] / 2
    cl = params["court"]["length"] / 2
    ax_top.plot([0, 0], [-cw, cw], "k-", lw=3)
    ax_top.plot([-cl, cl, cl, -cl, -cl], [-cw, -cw, cw, cw, -cw], "gray", lw=0.7)
    ax_top.set_xlabel("y [m]")
    ax_top.set_ylabel("x [m]")
    ax_top.set_title("top view")
    ax_top.set_aspect("equal")

    fig.tight_layout()
    fig.savefig(args.o, dpi=130)
    print(f"wrote {args.o}")


if __name__ == "__main__":
    main()
