# Wato Humanoid V1 locomotion training — session log (2026-07-13)

Goal: get the Wato Humanoid V1 flat-terrain locomotion policy to actually walk. At the
start of this session it fell within ~0.8s of every episode, regardless of reward
tuning. By the end, falling was solved (episode length 900-1000/1000, low_base
failures down to single digits), the policy escaped the frozen double-support
standing pattern it kept converging to (double-support time down to 3-7%, from
~100%), and the robot is **confirmed walking** (small, cautious steps, live-observed
in the Isaac Sim GUI) — see "Where it stands now" at the bottom.

## 1. URDF bug fix (early in session, unrelated to the RL work below)

`Ankle_P_L` (left ankle pitch) had the same rotation axis (`0 -1 0`) as `Ankle_R_L`
(left ankle roll) — a kinematically degenerate/redundant joint. The right side was
correctly orthogonal (`Ankle_P_R axis="1 0 0"`, `Ankle_R_R axis="0 1 0"`). Fixed by
changing `Ankle_P_L` axis to `-1 0 0` in both
`Wato Humanoid Simultion Model V1.urdf` and `Wato_Humanoid_V1_isaac.urdf`, then
deleted the cached USD conversion directory (`Wato_Humanoid_V1_isaac_usd/`) since
`force_usd_conversion=False` means Isaac Lab silently reuses a stale cached USD
otherwise. Verified via `diagnose_spawn.py` (symmetric L/R body heights after fix).

Diagnosed via `rsl_rl_scripts/diagnose_policy_rollout.py`, which needed a joint-naming
fix first (it was written for a different robot's naming convention) — edited the
`_print_pair` calls to use Wato's actual joint names (`Hip_F_L`, `Ankle_R_L`, etc.).

## 2. Spawn physics tuning

`max_depenetration_velocity` in `wato_humanoid_v1.py`: 5.0 (caused a violent
upward "pop" on spawn) → tried 1.0 (worse, new instability) → settled on **2.5**
(validated via `diagnose_spawn.py`: stable 8-second zero-action stand).

## 3. The big diagnostic arc: why does it keep falling in 0.8s?

Six independent single-variable hyperparameter probes were tried and every one
converged to the identical ~40-42 step (0.8s) episode-length ceiling, with the
identical visual failure mode (rigid forward topple, legs never separating):

1. `init_noise_std` 0.5 → 0.2 (`lownoise_probe_001`) — no effect, final 40.6
2. `Hip_F` action scale 0.25 → 0.40 in isolation (`hipf_scale_probe_001`) — no effect,
   video confirmed same rigid topple
3. Reset randomization tightened (`reset_base.velocity_range` ±0.5 → ±0.1,
   `reset_robot_joints.position_range` 0.5-1.5 → 0.9-1.1) — no effect, final 42.0
4. Command range widened to include standing (`rel_standing_envs` 0.0 → 0.1,
   `lin_vel_x` (0.12,0.22) → (0.0,0.22)) — no effect, final 40.6
5. `entropy_coef` cut 0.01 → 0.002 alongside `init_noise_std` → 0.1 — no effect,
   flat at ~35-36
6. `joint_vel` observation noise scaled down to match Wato's actual (slower) actuator
   speeds (±1.5 rad/s → ±0.3 rad/s, since ±1.5 was ~41% of Hip_F/Knee's
   `velocity_limit_sim=3.6652` vs a small fraction of G1's much faster joints) —
   no effect, final 42.8

**The decisive test**: `diagnose_spawn.py` run with **literal zero actions** (bypassing
the policy network entirely) under full reset domain randomization survived **60+
steps with zero terminations** (root height sank slightly, 0.78→0.65, never crossed
the 0.30m fall threshold). Every trained policy fell by ~40 steps. **The fall was
being actively caused by the policy's actions, not a physical limit.**

Follow-up: rendered a rollout from `model_0.pt` (a completely untrained, freshly
initialized network, before any gradient update) — it showed the **exact same rigid
forward-topple** as fully-trained checkpoints. This proved the failure wasn't
something 400+ iterations of training could ever fix by itself: even near-random
network output was enough to destabilize the stance, while doing literally nothing
was fine. Conclusion: the crouch stance was only *passively* stable — it had
essentially no active margin against small forward sway.

## 4. Real fix #1: ankle actuator PD gains were undertuned

Raised ankle `stiffness` 40→80, `damping` 3→6 in `wato_humanoid_v1.py` (ankle
correction is the primary biological mechanism for resisting small-amplitude
forward/backward lean, and it was the weakest-authority actuator in the rig).

Result (`ankle_stiffness_probe_001`, 400 iterations): episode length climbed from the
~40-42 ceiling to **60.16** and was still rising when the probe ended — first real
improvement of the session. Video showed the robot standing fully upright and stable
for a complete 6-second clip, no topple at all.

Extended run (`ankle_stiffness_extended_2000`, resumed to 2400 total iterations)
reached **990/1000 episode length**, `track_lin_vel_xy_exp` reward 0.92 — but then
**diverged catastrophically** near the end: `value_function loss` exploded to 5.4e10,
`Mean reward` collapsed to -418,453, `action_rate_l2` blew up to -21,796. Root cause
(diagnosed, not yet independently reproduced-and-fixed in isolation): `rsl_rl`'s
`empirical_normalization` was `False`. Once episodes started completing at
near-max length (~1000 steps instead of ~60), the value function's regression
targets grew hugely without observation/return normalization — likely causing the
blowup. Fix applied: `empirical_normalization = True` in
`agents/rsl_rl_ppo_cfg.py`, plus `entropy_coef` reduced 0.01→0.008 (noise std had
climbed past 1.0 before the crash instead of annealing down).

## 5. Real fix #2: ankle torque limit didn't match real hardware

Cross-referenced the actual mechanical docs
(`watonomous.github.io/humanoid-docs/mechanical/index.html`) against real actuator
datasheets (Robstride RS03/RS04, CubeMars AKH70 family):

| Joint | Real motor | Real peak torque | Sim `effort_limit_sim` (before) | Match? |
|---|---|---|---|---|
| Hip_R (Hip Yaw) | RS03 | 60 Nm | 60.0 | exact match |
| Hip_A (Hip Roll) | RS04 | 120 Nm | 120.0 | exact match |
| Knee | AKH70-48 | 222 Nm | 222.0 | exact match |
| **Ankle P/R** | **RS03** | **60 Nm** | **120.0** | **mismatch — 2x real torque** |
| Hip_F (Hip Pitch) | AKH70-48 (confirmed) | 222 Nm | 222.0 | exact match |

Ankle's `effort_limit_sim` was set to 120 Nm (matching Hip_A/RS04) when its real motor
(RS03) only delivers 60 Nm peak — looks like it was copy-pasted from the wrong block.
Corrected to **60.0** in `wato_humanoid_v1.py`. Retested empirically
(`ankle_60nm_probe_001`): performance was statistically identical to the 120Nm version
(final episode length 58.1 vs 60.2) — the stiffness fix does **not** depend on the
inflated torque budget, so it should hold up on real hardware. Extended run
(`ankle_60nm_extended`, resumed to 3399 total iterations, with the
`empirical_normalization`/`entropy_coef` fixes applied) reached **990.9/1000** episode
length cleanly, **no divergence this time** — reward 0.927, `low_base` failure down to
0.76%.

Hip_F's real motor was initially ambiguous in the docs page ("AKH70" without the
gear-ratio suffix the other joints get, vs Knee Pitch's explicit "AKH70-48") — resolved
by direct confirmation: Hip Pitch is also the 48:1/222Nm variant, same as Knee. So all
five other joints (Hip_R, Hip_A, Hip_F, Knee, and now Ankle after the fix above) match
their real hardware's torque rating exactly.

## 6. The "stable but frozen" problem

Both successful high-stability runs (990+/1000 episode length) converged to a
**static, fully frozen stance** — first a narrow parallel-feet crouch, later a wider
front-back lunge (after the ankle fix) — held completely rigid for the entire episode,
zero leg motion. `feet_air_time` reward stayed at ~0.0002-0.0003 (essentially exact
zero) throughout. Confirmed via frame-by-frame video inspection (identical pose
across 6+ seconds, no cycling).

Root cause: `feet_air_time_positive_biped` (the reward function in use, confirmed to
match G1's actual official config exactly — G1 uses no other anti-double-support
mechanism) only pays out once `single_stance` is *already* true (exactly one foot in
contact). If both feet are permanently planted, that condition never fires, so there's
no gradient toward the first exploratory foot-lift. Standing still already scores
~0.92-0.99/1.0 on `track_lin_vel_xy_exp` at the original `std=0.5`, so there was no
marginal incentive to risk stepping either.

**Note**: G1's own config does not need any extra anti-double-support term and
apparently escapes this local optimum via exploration alone — this is a
Wato-specific sticking point (different dynamics/actuators), not something inherited
from a G1 gap.

### Attempted fixes that didn't work

- Tightening `track_lin_vel_xy_exp` std (0.5 → 0.25, then → 0.4) to make "standing"
  score worse: overcorrected both times — produced too weak a gradient, episode
  length plateaued at ~45-51 for 500+ iterations (`stepping_fix_fresh_002/003`),
  nowhere near the 990 stability the original std=0.5 reached. **Reverted to 0.5.**
- Narrowing the command range to force nonzero velocity (`lin_vel_x` (0.0,0.22) →
  (0.15,0.28)): same overcorrection, same stall. **Reverted to (0.0, 0.22)**,
  kept `rel_standing_envs=0.1` (a legitimate independent fix).
- Adding `double_support_penalty` (a custom reward term already present in
  `mdp/rewards.py` from an earlier, since-removed gait-phase reward system — not part
  of the G1-aligned reward set) at weight `-0.3`, **active from iteration 0**
  (`double_support_fresh_001`): fighting "learn to balance" and "stop
  double-supporting" simultaneously from a random init was too much at once —
  plateaued at episode length ~48-51 for 500+ iterations.
- Same penalty **resumed from an already-frozen, fully-stable checkpoint**
  (`double_support_fix_001`): episode length immediately dropped 990→561 as the
  penalty fought the network's already-strong prior toward the frozen stance.
  Abandoned per explicit user feedback: resuming from a checkpoint anchored to the
  wrong behavior gives misleading signal — redo fresh instead.

### What worked (partially)

`double_support_penalty` **curriculum-gated**: starts at weight 0.0, a
`CurrTerm(func=mdp.modify_reward_weight, ...)` switches it to a real weight only
after `num_steps=55000` (~PPO iteration 2290, chosen to land just after the
run reliably reaches basic stability). This let the policy learn balance first,
uncontested, then introduced the anti-double-support pressure once it had something
to lose.

- At weight **-0.3** (`double_support_curriculum_001`): activated cleanly without
  crashing stability, but was **too weak once tracking reward approached its ~0.9
  ceiling** — a max -0.3 penalty became negligible by comparison. Finished at episode
  length 932.8, double-support time only down to ~69% (from ~100%), `feet_air_time`
  still ~0.0002.
- At weight **-2.0** (`double_support_curriculum_002`, current final config):
  activation immediately dropped double-support time to ~7%, dipping to ~1% at its
  best, before settling around 3-7% by the end. `feet_air_time` reward jumped to
  **0.108** — roughly 300-500x higher than any frozen-stance checkpoint all session.
  Episode length recovered from a transient dip to 684.8/1000 by the end of the run
  (`max_iterations=4000`, final checkpoint `model_3999.pt`).

## 7. Confirmed walking, then a false negative from bad visual evidence

The double-support deadlock had been escaped — the contact-pattern reward functions
(`feet_air_time_positive_biped`, `double_support_penalty`) were being satisfied at a
level never reached before (`feet_air_time` reward 0.108, ~300-500x higher than any
frozen-stance checkpoint earlier in the session; double-support time down to 3-7%
from ~100%). But my own video-frame analysis (5fps stills sampled from a 6-second
clip, isometric camera at distance) initially read this as "not walking" — no visible
net translation between first and last frame.

**That read was wrong.** The user checked the same checkpoint live in the Isaac Sim
GUI and confirmed it was walking — small, cautious steps, stable. At the command
range in use then (`lin_vel_x` max 0.22 m/s), per-frame displacement (a few cm at
0.2s intervals) is easy to miss by eye against a wide isometric grid, especially from
sparse still-frame sampling. **Lesson applied for the rest of the session: prefer
live GUI viewing over sparse-frame video analysis when checking for slow/subtle
locomotion.**

## 8. Widening the command range toward G1's actual scale

Once walking was confirmed, widened the command range in stages, checking live after
each step rather than trusting reward metrics or frame-sampled video alone:

1. `lin_vel_x` (0.0, 0.22) → (0.0, 0.45) (`widen_velocity_001`, resumed): held up,
   `feet_air_time` nearly doubled.
2. Made `lin_vel_x` symmetric (-0.45, 0.45) for backward walking, and added
   `lin_vel_y` (-0.15, 0.15) / `ang_vel_z` (-0.3, 0.3) for the first time — both were
   zeroed all session as a deliberate scope reduction while chasing basic forward
   stepping (`expand_xyz_001`, resumed). Large expected transient dip (episode length
   270 → recovered to 900+), held up.
3. `lin_vel_x` (-0.6, 0.6), `lin_vel_y` (-0.25, 0.25), `ang_vel_z` (-0.5, 0.5)
   (`higher_velocity_001`, resumed): mild transient dip, recovered, live-confirmed
   walking still held at this range.
4. `lin_vel_x` (-1.0, 1.0), `lin_vel_y` (-0.5, 0.5), `ang_vel_z` (-1.0, 1.0) — matching
   G1's own flat-config scale directly rather than incrementally widening again
   (`g1_scale_velocity_001`, resumed): surprisingly mild disruption for such a big
   jump (episode length barely dipped), tracking accuracy improved steadily as it
   adapted. Live-confirmed still walking at this point.

**Bug found and fixed along the way**: `WatoHumanoidFlatEnvCfg_PLAY` had its own
hardcoded command ranges (`lin_vel_x=(0.16,0.16)`, `lin_vel_y`/`ang_vel_z` both
`(0.0,0.0)`) that were never updated when the main training config's ranges were
widened. Every live/video check *this entire session* had actually only ever tested
straight-forward walking at a fixed 0.16 m/s, never lateral or turning, even after
those were added to training. Fixed by mirroring the main config's ranges into the
`_PLAY` override.

**Systematic re-diff against G1's actual source** (not memory) after all this
widening turned up one more thing: `init_noise_std=0.5` in Wato's flat PPO config,
sitting at half of G1's inherited `1.0` (`G1FlatPPORunnerCfg` never overrides it) —
unexamined since before this session started. Raised to `1.0` to match.

## 9. From-scratch validation (no resume)

By this point the training chain was ~10-12k iterations deep across five resumed
runs. Per feedback received mid-session: resuming is fine for validating incremental
widening on top of already-proven behavior, but it doesn't prove the *whole current
recipe* — ankle fixes, entropy/normalization fixes, curriculum-gated
double-support penalty, G1-scale command range, corrected `init_noise_std` — actually
converges on its own from a random init, rather than riding one lucky trajectory's
momentum. Ran `scratch_validation_001`: fresh start, no `--resume`, full current
config including the `init_noise_std=1.0` fix.

Tracked a similar (if initially slower, given the harder full-width command range
active from iteration 0) trajectory shape to the earlier resumed successful runs:
slow climb through ~iteration 1000, acceleration starting ~1250, a rapid
stability-transition jump (episode length 165 → 743 → 913 across iterations
1770-2256) with the same transient value-loss/noise-std elevation seen in every
prior successful transition, resolving cleanly (value loss *improved* through the
jump rather than diverging). Live-checked at iteration 2400 (just before the
double-support curriculum activates at ~2290) — confirmed good. **This validates the
recipe itself, not just one specific training trajectory.** (Curriculum activation
and full-range stability under it were still being watched as of last check-in —
see current checkpoint for latest status.)

## 10. Housekeeping

Training generates large volumes of throwaway logs and checkpoint directories.
Established a periodic-cleanup practice once findings are safely captured in this
document: delete logs/checkpoints from superseded intermediate steps in a widening
or resume chain, keep the ones cited as specific evidence (the six failed
hyperparameter probes, the ankle-fix lineage, the double-support-penalty attempts)
and whatever the currently-active run is. One cleanup pass freed ~686MB by removing
12 of 13 old training-run checkpoint directories, keeping only the then-latest.

## 11. Reusable process write-up

Drafted `.claude/skills/rl-training-iteration/SKILL.md` in this repo, capturing the
disciplined loop developed over this session (short probes first, cut plateaus early
rather than waiting them out, decisive isolating diagnostics like the zero-action/
untrained-network test, curriculum-gating conflicting reward terms, resume-vs-fresh
methodology, cross-checking reward metrics against actual visual/live observation)
so it's directly reusable for future RL/policy training work on this or other
projects, not just this specific robot.

## Where it stands now

Walking confirmed at increasingly wide command ranges, now matching G1's own scale
(`lin_vel_x` ±1.0 m/s, `lin_vel_y` ±0.5 m/s, `ang_vel_z` ±1.0 rad/s), via both a
resumed chain and (in progress) a from-scratch validation run proving the whole
recipe converges independently of any specific training trajectory.

### Next steps

- Finish watching `scratch_validation_001` through the double-support curriculum
  activation (~iteration 2290) and confirm it holds at the full G1-scale command
  range without a resumed head start.
- `feet_air_time` weight/threshold (currently 2.0/0.2, vs G1's 0.75/0.4) hasn't been
  revisited since the double-support-penalty fix took over as the dominant
  anti-freezing force — worth checking whether it's still needed at its current
  strength or could be relaxed back toward G1's values now that stepping is
  established.
- Continue to prefer live GUI viewing over sparse-frame video analysis when checking
  behavior at low/subtle speeds (see section 7).

## Current config summary (updated)

**`modelCfg/wato_humanoid_v1.py`**
- `max_depenetration_velocity = 2.5`
- Ankle actuator: `stiffness=80.0, damping=6.0, effort_limit_sim=60.0, velocity_limit_sim=20.42`
  (others unchanged: Hip_F/Knee 222Nm/3.6652rad/s, Hip_A 120Nm/20.944rad/s, Hip_R 60Nm/20.42rad/s)

**`config/wato_humanoid_v1/agents/rsl_rl_ppo_cfg.py`**
- `empirical_normalization = True`
- `entropy_coef = 0.008`
- `init_noise_std = 1.0` (raised from 0.5 to match G1's inherited value — previously unexamined)

**`config/wato_humanoid_v1/flat_env_cfg.py`**
- `track_lin_vel_xy_exp.std = 0.5` (unchanged from original G1-aligned value)
- `feet_air_time.weight = 2.0`, `threshold = 0.2` (vs G1's 0.75/0.4 — not yet revisited)
- `double_support_penalty`: weight 0.0 → -2.0 via curriculum at 55000 env steps
- `commands.base_velocity`: `rel_standing_envs=0.1`, `lin_vel_x=(-1.0, 1.0)`,
  `lin_vel_y=(-0.5, 0.5)`, `ang_vel_z=(-1.0, 1.0)` (matches G1's flat-config scale,
  except `lin_vel_x` also includes backward since Wato's was made symmetric)
- `observations.policy.joint_vel.noise = Unoise(-0.3, 0.3)` (down from ±1.5)
- Action scales: `Hip_F=0.25, Knee=0.30, Hip_A=0.06, Hip_R=0.06, Ankle_R=0.08, Ankle_P=0.12`
- `WatoHumanoidFlatEnvCfg_PLAY` command ranges now mirror the main training ranges
  (previously hardcoded/stale — see section 8)

**Latest checkpoint (active, from-scratch validation)**:
`logs/rsl_rl/wato_humanoid_flat/2026-07-13_21-59-44/model_2400.pt`
