"""Binned wandb run summary without numpy, so it runs on the login node in
the wandb-only venv:  ~/.venv-login/bin/python scripts/wandb_summary.py <run_id> [bins]
(the training venv cannot import numpy there: the login VM exposes a
generic KVM CPU model without x86-64-v2 flags)."""
import sys
from statistics import mean

import wandb

run_id = sys.argv[1]
nb = int(sys.argv[2]) if len(sys.argv) > 2 else 8
cols = [("Train/mean_reward", "mean_rew", "{:8.3f}"),
        ("Episode_Reward/face_contact", "contact", "{:8.4f}"),
        ("Episode_Reward/return_flight", "retflt", "{:7.4f}"),
        ("Policy/mean_std", "std", "{:6.3f}"),
        ("Metrics/perception/face_pstar_dist_at_tstar", "d@t*", "{:6.3f}"),
        ("Metrics/feasibility/tau_duty_j6", "duty_j6", "{:7.3f}"),
        ("Metrics/feasibility/tau_duty_j4", "duty_j4", "{:7.3f}"),
        ("Metrics/feasibility/tau_duty_j1", "duty_j1", "{:7.3f}"),
        ("Metrics/feasibility/qvel_peak_j1", "qv_j1", "{:6.2f}")]
api = wandb.Api()
run = api.run(f"badminton-rl/mjlab/{run_id}")
rows = [r for r in run.scan_history(keys=["_step"] + [c[0] for c in cols])]
n = len(rows)
print(f"run {run_id}  state={run.state}  iters={n}")
hdr = f"{'steps':>10s} " + " ".join(f"{c[1]:>{len(c[2].format(0))}s}" for c in cols) + "   hit%"
print(hdr)
w = max(1, n // nb)
for lo in range(0, n, w):
    chunk = rows[lo:lo + w]
    vals = [mean(r[c[0]] for r in chunk if r.get(c[0]) is not None) for c in cols]
    hit = vals[1] * 3 / 2 * 100  # contact logged = 2.0*hit/3s
    print(f"{chunk[0]['_step']:4.0f}-{chunk[-1]['_step']:4.0f} "
          + " ".join(c[2].format(v) for c, v in zip(cols, vals)) + f"  {hit:5.1f}")
