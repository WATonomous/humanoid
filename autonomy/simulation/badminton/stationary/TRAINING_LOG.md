# RL training log (teacher phase)

Running record of runs, findings, and next levers.

**Metric scale (corrected 2026-08-20).** Logged Episode_Reward/* values are
the per-episode sum (each term x weight x step_dt 0.02) DIVIDED BY
max_episode_length_s = 3.0 (mjlab reward_manager.py). So a 1-tick contact
at weight 100 logs as 2.0/3 = 0.667 per hit episode: face_contact 0.657
means a ~98.5% hit rate, not 33%. Every hit-rate figure below written
before this note ("31%", "33%") is 3x too low; the raw logged values
are still correct.

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

### result (cut at iter 260)

approach 0.025 -> 0.108 immediately (gradient hypothesis confirmed) but
flat from iter ~64; std collapsed again 0.44 -> 0.084; contact up 5x to
~0.00015 (0.08% of episodes) but flat. New finding: the concave exp kernel
itself rewards shrinking the action noise at a fixed pose (Jensen), so the
approach term finances the std collapse and entropy 0.005 cannot outbid it.

## run 3 — probe, entropy_coef 0.005 -> 0.02 (keeps std 0.8), 1000 iters

Hypothesis: exploration collapse, not shaping, is now the binding
constraint. Success: mean_std holds >= 0.2 through iter 500 while approach
stays >= 0.1, and face_contact climbs past 0.001.

### result (cut at iter ~549)

std holds at ~0.25 (criterion met; entropy 0.02 is right). contact doubled
to ~0.15% of episodes and return_flight fires occasionally — exploration
now finds hits — but approach is pinned at the same 0.108 as run 2 and
contact growth is negligible. A hit pays only 0.2 effective vs 0.108 for
an episode of hovering: discovery without reinforcement.

## run 4 — probe, face_contact weight 10 -> 100, 1000 iters

Hypothesis: with exploration alive, the binding constraint is the sparse
bonus scale. Effective 2.0/hit should make discovered hits dominate the
gradient. Success: face_contact episode reward > 0.02 (1% hit rate) and
visibly compounding by iter 1000.

### result (cut at iter ~503)

Hypothesis refuted: hit rate unchanged (~0.1%) at 10x the bonus — the
sparse scale was never the constraint. Side effect: with approach
saturated and no task gradient, entropy 0.02 wins the tug-of-war — std
climbed 0.49 -> 0.60, action_rate penalty grew, mean reward drifted down.
Three probes now end at the same "hover near p*" behavior: the lever
family (reward magnitudes) is not the bottleneck.

Actual constraint: gradient gap over the last 30 cm. exp(-d^2/0.8^2) is
flat near d=0, so nothing differentiates hovering 30 cm off from parking
on the flight path; only luck crosses the gap. p* lies ON the trajectory,
so precise parking is itself sufficient to produce contact.

## run 5 — probe: fine approach kernel + entropy 0.01, 1000 iters

Changes: (1) approach_fine term, same approach_intercept func, sigma 0.15,
weight 5 — steep slope over the last 30 cm; (2) entropy 0.02 -> 0.01
(0.005 collapsed, 0.02 overshoots; fine kernel now supplies an opposing
gradient). Success: approach_fine climbing by iter 300, face_contact
> 0.02 by iter 1000, std in 0.15-0.35.

### result (ran the full 1000)

Best probe yet but under the bar: contact 0.0006 -> ~0.003 (0.13% hits,
5x run 4) with growth stalling after ~iter 500; approach_fine touched
(0.010 vs ~5.3 if parked at p*); coarse approach pinned at the familiar
0.106; std healthy ~0.41. Reading: still lucky-swing hits, not parking.
Five reward-side probes now hit versions of the same wall — stop
inferring geometry from reward values and measure it.

## run 6 — probe: same config as run 5 + ground-truth metrics, 2000 iters

No reward changes. PerceptionCommand now logs
Metrics/perception/face_pstar_min_dist (per-episode min face->p*
distance) and face_pstar_dist_at_tstar. Two questions: (1) what is the
policy's actual closest approach — 10 cm (parking, contact geometry is
the blocker) or 40+ cm (not parking, shaping still the blocker)? (2)
does run 5's contact growth compound given 2x runway?
Note: test_mjlab_task::test_cork_orientation_tracks_velocity failed once
in 3 full-suite runs (passes alone and on retry) — flaky threshold under
CPU Warp nondeterminism, unrelated to this change.

### result (cut at ~345): ROOT CAUSE FOUND — arm was mounted 1.2 m in the air

The new metrics: min face->p* distance ~0.90 m flat, d@t* ~0.92 — worse
than the static ready-pose distance (0.69 m), impossible for a moving arm.
Zero-action rollouts in the Warp env showed the face at reset at
(1.15, -1.41, 2.42) instead of (0.59, -1.44, 1.22): the scene XML places
arm_base_link AND env_cfg restated the same pose via InitialStateCfg,
which mjlab COMPOSES with the XML placement (the assets.arm_base_pose
comment claiming it overwrites was wrong). Every prior run trained with
the racket hanging ~1.2 m above the court; the shuttle still passed
through p* (verified <= 5 cm), so hits were only possible on the rare
high-trajectory tails. This explains all five reward-probe plateaus.

Fix: InitialStateCfg carries only joint_pos; the XML placement is the
single source. Verified: face at reset matches the CPU env to 3 mm; gate
tests pass.

## run 7 — corrected world, run-5 reward config, 2000 iters

Same rewards as runs 5-6 (sigma 0.8 + fine 0.15/5.0, entropy 0.01,
contact 100). All prior reward conclusions were fitted to the broken
world, but the sigma-0.8 analysis was derived from the CPU geometry,
which is now the true geometry. Success: min_dist visibly dropping,
face_contact > 0.02 within the first 1000 iters.

### result (ran the full 2000): first working policy

Contact 0.627 (~31% hit rate) stable from iter ~300; min face->p* dist
2 cm (parked on the flight path); return_flight 0.15 and saturating.
Entropy 0.01 drifted std 0.42 -> 0.85 after saturation — the classic
constant-entropy-pressure vs saturated-concave-reward equilibrium —
tripling action_rate cost and holding d@t* at 0.14 (best was 0.066 at
std 0.42). Reward structure works on the corrected world.

## run 8 — first full run: entropy 0.005, 3000 iters, from scratch

Fresh start rather than resume: run 7's checkpoint is anchored to
high-noise behavior, and the recipe needs a from-scratch convergence
proof. Rationale for 0.005: the old collapses at 0.005 were the broken
world's fault, not the coefficient's. Success: contact > 1.0 (50%+ hit
rate) approaching the scripted baseline's 71%, d@t* back under 0.08,
return_flight climbing past 0.2.

## known next levers (in order, do not stack blindly)

1. face_contact effective bonus is only 0.2 per hit after dt scaling —
   once contacts exist, raise weight (10 -> 50+) so hitting dominates
   hovering near p*.
2. entropy_coef 0.005 let std collapse in run 1; if std collapses again
   despite a live gradient, raise to 0.01-0.02.
3. return_flight never fired; only meaningful after contacts exist.

## run 8 — bank eval (scripts/eval_rl.py, deterministic policy, 4096 eps)

Overall hit rate 0.993. By p* distance in front of the chest plane:
0.15-0.25 m 0.944 (n=162), 0.25-0.35 m 0.983, >= 0.35 m 0.995-0.999.
By height: flat 0.986-0.999. By lateral x: x < -0.4 m 0.954, else
0.991-1.000. Reading: contact is solved; the residual ~1% of misses sits on
body-line (chest-plane) and far-left intercepts, consistent with the
viewer review ("close clips when the shuttle comes at chest/head") and
with the scripted baseline's own residual failure class. The teacher's
remaining objective is return quality (return_flight 0.246 logged = ~0.74
per-episode sum), not contact.

### run 8 feasibility eval (scripts/eval_rl.py, 4096 eps)

Peak joint speeds 6-9 rad/s on every joint vs rated 24.6 (AK10-9) /
40.8 (AK80-9) / 45 (GL40) rad/s: speed is not a constraint, no
speed-torque derating needed. Torque: every joint reaches its peak clamp
in 95-100% of episodes (bang-bang PD swings). Duty above RATED torque:
j1/j2 16%, j3 19%, j4 35%, j5 13%, j6 (GL40 wrist) 95% — the wrist lives
at 0.73 Nm peak nearly always. Gravity torque of the 90 g racket at its
0.45 m offset is ~0.25-0.4 Nm about the wrist, i.e. at/over the 0.25 Nm
rated value just holding the racket horizontal: most of the 95% is
posture, not stroke.

## run 9 — thermal penalty (torque_thermal w -0.5), 1500 iters

New reward mdp.torque_over_rated = sum_j max(|tau_j|/tau_rated_j - 1, 0)
(qfrc_actuator, post-clamp; normalized so a 0.5 Nm wrist overload weighs
like 35 Nm at the shoulder), weight -0.5: a full-episode wrist overload
costs ~1.2 vs the 2.0 contact bonus. Physics-grounded (no heat model in
the sim), not a posture opinion: the only way to cut wrist duty is
wrist-sparing racket orientations, which should emerge. Success: hit rate
stays >= 0.97 (eval) while Metrics/feasibility/tau_duty_j6 falls from
0.95 toward the 0.2-0.35 the other joints show. If hit rate collapses
instead, the GL40 cannot do this job and the wrist upgrade in the ledger
becomes mandatory.

### result (cut at iter 573)

Hit rate 88% (run 8 at the same stage: ~97%), return_flight 0.011 (6x
lower), shoulder/elbow rated-torque duty collapsed 16%/35% -> 2%/0.2%
and shoulder peak speed 7.6 -> 3.0 rad/s: the instantaneous per-tick
penalty suppressed the transient swing peaks the strong joints are rated
for. Wrist duty only fell 0.95 -> 0.67. Root cause of the wrist duty is
NOT gravity (wrist gravity moment at READY = 0.000 Nm from the model;
j5 carries the racket's 0.57 Nm): it is the servo. kp 60 / kv 3 on a
0.73 Nm motor saturates at any tracking error > 0.7 deg, so the wrist is
bang-bang by construction. Penalty formulation lesson: heat is an
integral; a per-tick excess penalty cannot separate 20 ms peaks from
sustained load (an I2t accumulator would).

## run 10 — realistic servo gains, thermal penalty off, 1500 iters

Repo evidence: hardware has no MIT gains configured (safety_limits.yaml
mit_kp 0.0, "start kp < 10"); the CAN MIT protocol caps kp <= 500,
kd <= 5 for AK motors (mit_profiles.yaml); the GL40 has no profile. The
sim's kp [600,600,300,300,100,60] / kv [30,30,15,15,5,3] exceeded what
the drives accept, i.e. the servo was the "fake motor". New gains
kp [400,400,300,300,100,8], kv [5,5,5,5,2,0.3] (wrist = the start-low
placeholder until the drive is tuned). scene/badminton.xml rebuilt; the
gate-5 scripted-baseline numbers in README are now stale (same actuators)
and need a re-run. torque_thermal removed from the reward set (function
kept). Success: hit rate on the bank eval, wrist duty, and return quality
under an honest wrist — whatever they are, they are the real numbers.

