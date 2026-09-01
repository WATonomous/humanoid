"""Gate 5 eval: run launcher episodes under the scripted baseline and report
the gate table. Logs per-episode metrics + trajectories for replay.py.

Usage:
    uv run python scripts/run_gate5_eval.py [-n 500] [--seed 0] [-o runs/g5]
    uv run python scripts/run_gate5_eval.py -n 50 --no-record   # quick look
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from baseline import evaluate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-o", default="runs/gate5")
    ap.add_argument("--no-record", action="store_true",
                    help="skip qpos/qvel logging (no replay)")
    args = ap.parse_args()

    summary, results = evaluate.run_eval(
        args.n, seed=args.seed, record=not args.no_record, out_dir=args.o)

    print(f"\n=== gate 5 ({args.n} episodes) ===")
    rows = [
        ("contact rate", f"{summary['contact_rate']:.1%}", ">= 90%",
         summary["contact_rate"] >= 0.90),
        ("net clearance (of contacts)",
         f"{summary['net_clearance_of_contacts']:.1%}", ">= 60%",
         summary["net_clearance_of_contacts"] >= 0.60),
        ("median face pos err at t̂*",
         f"{summary['median_face_pos_err_at_t_star']*1000:.0f} mm", "< 30 mm",
         summary["median_face_pos_err_at_t_star"] < 0.03),
        ("median face timing err",
         f"{summary['median_face_timing_err']*1000:.0f} ms", "< 15 ms",
         summary["median_face_timing_err"] < 0.015),
        ("joint-limit-hit episodes", str(summary["limit_hit_episodes"]), "0",
         summary["limit_hit_episodes"] == 0),
        ("self-collision episodes", str(summary["self_collision_episodes"]),
         "0", summary["self_collision_episodes"] == 0),
    ]
    for name, val, target, ok in rows:
        print(f"{'PASS' if ok else 'FAIL'}  {name:32s} {val:>10s}  (gate {target})")
    print(f"\nextra: median min_dist {summary['median_min_dist']*1000:.0f} mm, "
          f"median cross-track {summary['median_cross_track']*1000:.0f} mm, "
          f"median contact rel speed "
          f"{summary['median_contact_rel_speed']:.1f} m/s")
    print(f"logs: {args.o}/  (replay: uv run python scripts/replay.py {args.o} --rank 0)")


if __name__ == "__main__":
    main()
