# RL training log (teacher phase)

Running record of runs, findings, and next levers. Metric values are
episode-mean rewards as logged by rsl_rl (mjlab multiplies each reward term
by step_dt = 0.02, so a sparse 1-tick reward of weight 10 logs as 0.2).

## run 1 — a1vagllq (SLURM 593617), 2026-08-19, cut at ~943 iters

Config: as committed at e8a2b5f (approach std 0.4, entropy 0.005,
init_std 0.5, 1024 envs).

Result: plateaued from ~iter 100. approach 0.025, face_contact <= 0.0003
(contact in ~1/10k episodes), return_flight 0, mean_std collapsed
0.5 -> 0.056, every episode ends grounded at ~63 ticks.

Diagnosis (verified, not guessed): at the READY pose the face is a median
0.69 m from p* (p10 0.30, p90 0.99; measured over 200 launcher episodes).
exp(-d^2/0.4^2) at 0.69 m = 0.05 — the measured plateau is exactly the
policy sitting still at ready and collecting that value for the pending
window. Gradient too weak vs the action penalties; exploration std then
collapses and locks it in.

## run 2 — probe, approach std 0.4 -> 0.8, 1000 iters

Hypothesis: shaping gradient (not entropy, not contact weight) is the
bottleneck. Single-variable probe. sigma = 0.8 ~= optimal-gradient sigma at
d = 0.7 (max of d/dd exp(-d^2/s^2) over s is s = d).

Success criteria: approach >= 0.3 by iter 300 and rising; mean_std >= 0.15
at iter 500; face_contact > 0.005 by iter 1000.

## known next levers (in order, do not stack blindly)

1. face_contact effective bonus is only 0.2 per hit after dt scaling —
   once contacts exist, raise weight (10 -> 50+) so hitting dominates
   hovering near p*.
2. entropy_coef 0.005 let std collapse in run 1; if std collapses again
   despite a live gradient, raise to 0.01-0.02.
3. return_flight never fired; only meaningful after contacts exist.
