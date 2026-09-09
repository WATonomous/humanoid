"""Strike-zone and racket-face maps from scripts/eval_rl.py output.

  uv run scripts/plot_hits.py [runs/eval_pstar_hits.npy] [runs/hits.png]

Left: front-on view relative to the body (like a pitch chart) — where each
episode's intercept point sat, hits vs misses. Right: where the shuttle
passed the racket, in the face frame: its position at closest approach to
the face centre (the contact point for hits); the outline is the collision
face. Misses that came within NEAR_M of the face centre are placed on the
map; misses farther than that are whiffs, counted in the title.
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Ellipse, Rectangle

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import aero

GOOD, CRITICAL = "#0ca30c", "#d03b3b"          # status palette (fixed)
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
NEAR_M = 0.30   # a miss closer than this to the face centre is a near miss


def main() -> None:
    src = sys.argv[1] if len(sys.argv) > 1 else "runs/eval_pstar_hits.npy"
    out = sys.argv[2] if len(sys.argv) > 2 else "runs/hits.png"
    r = np.load(src)
    p = aero.load_params()
    base_x, base_y = p["arm"]["base_x"], p["arm"]["base_y"]
    mount = p["arm"]["mount_height"]
    fw, fh = p["racket"]["face_size"][:2]
    front, z, x, hit = r[:, 0], r[:, 1], r[:, 2], r[:, 3].astype(bool)
    x_rel = x - base_x                     # body center 0; + = robot's right
    uv = r[:, 4:6]
    dmin = r[:, 6] if r.shape[1] >= 7 else np.hypot(uv[:, 0], uv[:, 1])
    near = np.isfinite(uv[:, 0]) & (dmin < NEAR_M)
    n, nh = len(r), int(hit.sum())
    miss = ~hit
    clip = miss & near
    whiff = miss & ~near

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(13, 6.2), facecolor=SURFACE)
    fig.suptitle(f"run-11 teacher, {n} episodes: {nh} hits ({nh / n:.1%}), "
                 f"{int(clip.sum())} near misses, {int(whiff.sum())} whiffs (> {NEAR_M * 100:.0f} cm off)",
                 color=INK, fontsize=12, x=0.5, y=0.98)

    # --- left: front-on strike zone (x lateral vs z height) ---------------
    ax.set_facecolor(SURFACE)
    # body silhouette: the stand up to shoulder height, a head above it;
    # outlines drawn over the points so the body stays visible
    for zo, fill in ((0, True), (2.5, False)):
        ax.add_patch(Rectangle((-0.15, 0.0), 0.30, mount, fill=fill, fc=GRID,
                               ec=INK2, lw=0.8, zorder=zo))
        ax.add_patch(Circle((0.0, mount + 0.28), 0.12, fill=fill, fc=GRID,
                            ec=INK2, lw=0.8, zorder=zo))
    ax.axhline(mount, color=INK2, lw=0.8, ls=(0, (3, 3)), zorder=1)
    ax.text(1.02, mount + 0.02, "shoulder height", color=INK2, fontsize=8)
    ax.scatter(x_rel[hit], z[hit], s=14, c=GOOD, alpha=0.35, linewidths=0,
               zorder=2, label=f"hit ({nh})")
    ax.scatter(x_rel[miss], z[miss], s=64, c=CRITICAL, marker="X",
               edgecolors=SURFACE, linewidths=1.2, zorder=3,
               label=f"miss ({int(miss.sum())})")
    ax.set_xlim(-1.2, 1.2); ax.set_ylim(0.0, 2.3); ax.set_aspect("equal")
    ax.set_xlabel("lateral offset from body center (m)  ->  robot's right", color=INK2)
    ax.set_ylabel("height (m)", color=INK2)
    ax.set_title("intercept point p*, front-on as seen from behind the robot", color=INK, fontsize=10)
    ax.legend(loc="upper left", frameon=False, fontsize=9)

    # --- right: racket face map (face frame u, v) --------------------------
    bx.set_facecolor(SURFACE)
    bx.add_patch(Rectangle((-fw * 50, -fh * 50), fw * 100, fh * 100, fill=False, ec=INK, lw=1.6, zorder=4))
    bx.add_patch(Ellipse((0, 0), fw * 100, fh * 100, fill=False, ec=INK2, lw=1.0, ls=(0, (3, 3)), zorder=4))
    bx.axhline(0, color=GRID, lw=0.8, zorder=0); bx.axvline(0, color=GRID, lw=0.8, zorder=0)
    hc, mc = hit & near, clip
    bx.scatter(uv[hc, 0] * 100, uv[hc, 1] * 100, s=12, c=GOOD, alpha=0.30,
               linewidths=0, zorder=2, label=f"hit ({int(hc.sum())})")
    bx.scatter(uv[mc, 0] * 100, uv[mc, 1] * 100, s=64, c=CRITICAL, marker="X",
               edgecolors=SURFACE, linewidths=1.2, zorder=3,
               label=f"near miss ({int(mc.sum())})")
    lim = 20.0
    bx.set_xlim(-lim, lim); bx.set_ylim(-lim, lim); bx.set_aspect("equal")
    bx.set_xlabel("face x (cm)", color=INK2); bx.set_ylabel("face y (cm)", color=INK2)
    bx.set_title(f"shuttle at closest approach to the face (outline = {fw*100:.0f}x{fh*100:.0f} cm face)",
                 color=INK, fontsize=10)
    bx.legend(loc="upper left", frameon=False, fontsize=9)

    for a in (ax, bx):
        for s in a.spines.values():
            s.set_color(GRID)
        a.tick_params(colors=INK2, labelsize=8)
        a.grid(True, color=GRID, lw=0.6, zorder=0)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print("wrote", out)


if __name__ == "__main__":
    main()
