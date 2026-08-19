"""Interactive aero tuning lab (phase 1 companion).

Sliders for k, launch speed, launch angle; redraws the RK4 trajectory live.
Presets run the gate-1 checks visually. If a film CSV exists it is overlaid so
tuning k means dragging the slider until the curve sits on the data.

Usage:
    uv run python scripts/aero_lab.py [--film path/to/track.csv]

Film CSV format: columns t, y, z (meters, seconds); x is ignored for 2D fits.
Keys: 1 = drop preset, 2 = range preset, 3 = shape fan preset, 0 = free play.
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import aero


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--film", help="CSV with columns t,y,z of a tracked flight")
    args = ap.parse_args()

    params = aero.load_params()
    k0 = params["shuttle"]["k"]

    film = None
    if args.film:
        film = np.genfromtxt(args.film, delimiter=",", names=True)

    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(bottom=0.28)
    ax.set_xlabel("y [m]")
    ax.set_ylabel("z [m]")
    ax.grid(True, alpha=0.3)

    ax_k = plt.axes([0.15, 0.16, 0.7, 0.03])
    ax_v = plt.axes([0.15, 0.11, 0.7, 0.03])
    ax_a = plt.axes([0.15, 0.06, 0.7, 0.03])
    s_k = Slider(ax_k, "k [1/m]", 0.10, 0.35, valinit=k0)
    s_v = Slider(ax_v, "speed [m/s]", 2.0, 25.0, valinit=8.0)
    s_a = Slider(ax_a, "angle [deg]", -60.0, 85.0, valinit=30.0)

    state = {"preset": 0}

    def draw(_=None):
        ax.clear()
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("y [m]")
        ax.set_ylabel("z [m]")
        k = s_k.val
        v_t = np.sqrt(aero.GRAVITY / k)
        preset = state["preset"]

        if preset == 1:  # drop test
            t, P, V = aero.simulate([0, 0, 20.0], [0, 0, 0], k, t_max=4.0,
                                    stop_at_ground=False)
            sp = np.linalg.norm(V, axis=1)
            ax.plot(t, sp, label="|v|(t)")
            ax.axhline(v_t, ls="--", c="gray", label=f"v_t = {v_t:.2f}")
            ax.axhline(0.99 * v_t, ls=":", c="gray")
            reached = sp >= 0.99 * v_t
            if reached.any():
                ax.axvline(t[np.argmax(reached)], ls=":", c="tab:green",
                           label=f"0.99 v_t at {t[np.argmax(reached)]:.2f} s")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("speed [m/s]")
            ax.set_title("drop test: reach 0.99 v_t < 2.5 s")
        elif preset == 3:  # shape fan
            for speed in np.linspace(4, 20, 10):
                v0 = speed * np.array([0, np.cos(np.radians(45)),
                                       np.sin(np.radians(45))])
                t, P, V = aero.simulate([0, 0, 2.0], v0, k, t_max=10.0)
                ax.plot(P[:, 1], P[:, 2], lw=1)
            ax.set_title("shape fan 4-20 m/s at 45 deg: every arc parachute-shaped")
            ax.set_aspect("equal")
        else:  # free play / range preset
            ang = np.radians(s_a.val)
            v0 = s_v.val * np.array([0, np.cos(ang), np.sin(ang)])
            z0 = 0.0 if preset == 2 else 2.0
            t, P, V = aero.simulate([0, 0, z0], v0, k, t_max=10.0)
            ax.plot(P[:, 1], P[:, 2], label=f"k={k:.3f} (v_t={v_t:.2f})")
            # vacuum comparison
            tv = np.linspace(0, 2 * v0[2] / aero.GRAVITY + np.sqrt(2 * z0 / aero.GRAVITY) + 1, 200)
            yv = v0[1] * tv
            zv = z0 + v0[2] * tv - 0.5 * aero.GRAVITY * tv**2
            m = zv >= 0
            ax.plot(yv[m], zv[m], ls="--", c="gray", alpha=0.5, label="vacuum")
            if P[-1, 2] < 0:
                _, p_land = aero.landing_point(t, P)
                desc = np.degrees(np.arctan2(-V[-1, 2], V[-1, 1]))
                ax.set_title(f"range {p_land[1]:.2f} m, descent {desc:.1f} deg"
                             + ("  [range preset: expect ~7-12 m, >60 deg]" if preset == 2 else ""))
            ax.set_aspect("equal")
            if film is not None:
                ax.plot(film["y"], film["z"], ".", c="tab:red", ms=4, label="film")
                ax.legend()
            ax.legend()
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key in "0123":
            state["preset"] = int(event.key)
            if event.key == "2":
                s_v.set_val(20.0)
                s_a.set_val(45.0)
            draw()

    s_k.on_changed(draw)
    s_v.on_changed(draw)
    s_a.on_changed(draw)
    fig.canvas.mpl_connect("key_press_event", on_key)

    if film is not None:
        pts = np.column_stack([np.zeros(len(film["t"])), film["y"], film["z"]])
        k_fit, v_t_fit, res = aero.fit_k(film["t"], pts)
        print(f"least-squares fit: k={k_fit:.4f} (v_t={v_t_fit:.2f}), "
              f"residual rms={np.sqrt(np.mean(res.fun**2))*1000:.1f} mm")
        print("drag the k slider to sanity-check the fit against the points")
        s_k.set_val(k_fit)

    draw()
    plt.show()


if __name__ == "__main__":
    main()
