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

## 12. G1 diff completion, reduced randomization, and double-support-penalty removal

Finished a full systematic diff against G1's actual official source (not memory):
matched `lin_vel_x` to forward-only `(0.0, 1.0)` (reverted from symmetric — G1 doesn't
walk backward either, since turning already covers repositioning), matched
`reset_base.pose_range.yaw` to full random heading `(-3.14, 3.14)`, confirmed
`self_collision=False` is not a G1 divergence (`G1_MINIMAL_CFG` also uses
`enabled_self_collisions=False`).

Scaled domain randomization down toward G1's own gentler reset distribution
(`reset_base.velocity_range` all axes ±0.5 → ±0.2, `reset_robot_joints.position_range`
(0.5,1.5) → (0.8,1.2)) — real stepping emerged naturally without the curriculum-gated
`double_support_penalty` (`feet_air_time` rose 0.0131 → 0.1364 well before the
penalty's curriculum was even scheduled to activate). **Removed
`double_support_penalty` entirely** (reward term, curriculum activation block, and the
now-dead function in `mdp/rewards.py`) once this was confirmed live — no longer
needed once the reset distribution was gentler.

Also tested raising `feet_air_time.threshold` 0.2 → 0.4: the resulting checkpoint hit
a session-high `feet_air_time` reward (0.32) but looked *weaker* live (episode length
~630 and `low_base` ~44%, vs 900+/~20% on earlier checkpoints). **Reverted to 0.2** —
the higher threshold traded stability for fuller swings and wasn't a net win.

## 13. Self-collision test (failed) and a "crab-like" gait diagnosis

Tested enabling real self-collision (`self_collision=True`,
`enabled_self_collisions=True`) to see if it would change gait/turning behavior:
**failed catastrophically** — every episode terminated after exactly 1 step via
`base_contact`, 100% of episodes. Root cause: the default crouch-pose geometry has the
base overlapping/very close to an adjacent link, previously masked entirely by
self-collision being off. Reverted both flags back to `False`; enabling this properly
would need actual pose/geometry fixes, not just a config flag — not attempted.

Separately, `error_vel_yaw` plateaued around 0.30-0.36 for 800+ iterations while
`error_vel_xy` kept improving, and checkpoints visibly showed a "crab-like" gait (body
not facing its direction of travel). Tried raising `Hip_R` (hip yaw) action scale
0.06 → 0.15, since 0.06 was set back when `ang_vel_z` was zeroed out entirely (no
turning at all) and never revisited once turning commands were added. Result was
inconclusive via resume (fighting the network's already-learned "small correction"
prior, same failure pattern seen earlier with `double_support_penalty`); a fresh
no-resume test at the new scale settled at a similar plateau. **Kept `Hip_R=0.15`**
(more actuator authority is directionally correct even if not yet proven to fix yaw
tracking on its own) but the crab-like symptom itself turned out to have a different,
much larger root cause — see section 14.

## 14. Root cause found: URDF forward-axis bug

The user noticed live in the GUI that commanding what should be "forward" actually
produced sideways/crab motion, and that a *different* command axis produced clean
forward walking. Investigated the URDF directly rather than assuming:

- `base_to_base_link` (the root fixup joint) applies a **-90° rotation about X only**
  (`rpy="-1.5708 0 0"`), with a comment: *"Isaac Lab: Z-up root. CAD base_link is
  Y-long; fixed joint stands the robot."* This was written purely to convert the
  CAD's Y-up convention into Isaac Sim's Z-up convention.
- Comparing `Hip_F_L` (origin x=-0.101) and `Hip_F_R` (origin x=+0.066) — they differ
  almost entirely in **X**, with Y and Z nearly identical between the two hips. **X is
  the left/right leg-separation axis**, not forward.
- A pure X-axis rotation leaves the X-component of every vector unchanged, so after
  the existing fixup joint, **X is still lateral** in Isaac Lab's root frame too — and
  the CAD's true forward axis (Z) landed on **Y** instead.
- Isaac Lab's velocity command/reward convention (and G1's own URDF) assumes local
  **+X = forward**. Wato's root frame had forward on **+Y**. Every `lin_vel_x`
  "forward" command all session had actually been asking the robot to move along its
  own *lateral* axis — this is the actual explanation for the crab-like gait, not
  (primarily) insufficient `Hip_R` authority.

**Fix**: added an extra -90° rotation about Z on top of the existing X-axis fix, on
the same `base_to_base_link` joint (`rpy`: `-1.5708 0 0` → `-1.5708 0 -1.5708`). This
rotates only the horizontal plane — the already-correct up/down mapping is untouched
— remapping CAD-forward onto +X and CAD-lateral onto Y (left = +Y). Verified by
computing the resulting rotation matrices by hand before editing (not just guessing a
sign).

This invalidates every prior checkpoint (the whole notion of "forward" changed at the
root frame), so a **fresh** run was required: `urdf_forward_axis_fix_fresh_001`.
Converged cleanly — episode length climbed 35 → 981 and tracking rewards
`track_lin_vel_xy_exp`/`track_ang_vel_z_exp` reached ~0.85/0.87 by iteration ~5300,
genuine stepping confirmed (`feet_air_time` 0.0 → 0.25+), no `base_contact` issues.
Live play confirmed the gait no longer crabs.

## 15. Leg-crossing (scissoring) gait and the `feet_crossing_penalty` term

With `self_collision=False`, nothing physically stops the legs from passing through
each other — live play on the axis-fixed checkpoint showed a scissoring gait (legs
crossing the midline each stride). G1 apparently doesn't need an equivalent term for
its own morphology, but Wato's does. Added `feet_crossing_l2` to `mdp/rewards.py`:
computes lateral (body-yaw-frame Y) separation between the two feet
(`left_y - right_y`) and penalizes it going below a margin (i.e. the left foot ending
up right of the right foot, or getting too close).

Important implementation detail: `SceneEntityCfg` defaults to the asset's own internal
body index order for `body_names`, **not** the order written in the list — a
signed/asymmetric term like this needs `preserve_order=True` explicitly, or left/right
could be silently swapped.

Iterated the weight/shape twice based on live metrics + visual checks:
- **v1: linear clamp, weight -2.0, margin 0.0.** Metric oscillated in a
  -0.011 to -0.022 band across ~2000+ iterations without trending toward zero — too
  weak relative to the dominant tracking rewards (0.85+).
- **v2: linear clamp, weight -6.0, margin 0.05.** Genuine improvement (metric dropped
  to ~-0.009 to -0.011), but live play still showed feet drifting close, plus a new
  **visible stutter** — a hard linear clamp has a discontinuous gradient right at the
  margin boundary, causing sharp corrective "flinches" every time a foot approached
  it.
- **v3 (current): squared penalty, weight -40.0, margin 0.08.** Squaring the clamped
  term gives a gentler gradient for small violations and a much steeper one for large
  ones, removing the flinch; margin widened for more standing separation; weight
  raised to compensate for squaring shrinking typical small-violation magnitudes.
  Confirmed live: crossing/stutter visibly improved, tracking metrics unaffected
  (still 0.85+/0.87+).

## 16. From-scratch validation of the full final recipe

By this point the axis-fix lineage was several resumes deep (crossing penalty added
mid-training, then re-tuned twice more, each via resume on proven behavior). Per the
same resume-vs-fresh discipline as section 9: ran `full_recipe_validation_fresh_001`
— fresh start, no resume, full current recipe (URDF fix + squared crossing penalty at
final weight/margin) active from iteration 0.

Converged cleanly and comparably fast to the original axis-fix run: episode length
36 → 991 and `track_lin_vel_xy_exp`/`track_ang_vel_z_exp` → 0.87/0.90 within about 50
minutes of wall-clock training, `feet_crossing_penalty` small and stable throughout,
no `base_contact` failures. **This validates the full recipe converges on its own**,
not just riding the prior run's momentum through several staged reward additions —
important for reproducibility if this config is handed to teammates.

## 17. Operational notes learned this session (see also the shared skill)

- **GPU can't run train + play simultaneously** on this single-GPU dev box, even with
  VRAM headroom — symptom is `Exception: Failed to get DOF velocities from backend`
  during environment init, not a memory error. Workflow: pause training (kill the
  process; the last `save_interval`-based checkpoint is safe), view/play, then resume.
- **Resuming creates a brand-new timestamped run directory** each time, even with the
  same `--run_name` repeated — always find the latest checkpoint by directory mtime,
  not by assuming it continues in the same folder.
- **Launch invocation**: must use `isaaclab.sh -p script.py` (bare `python`/`python3`
  are missing the framework's dependencies), run from the directory `logs/rsl_rl/...`
  resolves relative to (not the script's own location), and set `PYTHONPATH`
  explicitly for the project's own package.
- These lessons (plus the URDF-forward-axis-convention check, the squared-penalty
  stutter fix, and the `preserve_order=True` gotcha) are now written into
  `.claude/skills/rl-training-iteration/SKILL.md` for reuse beyond this project.

## Where it stands now

Walking confirmed clean (no crab gait, crossing/stutter substantially improved) at
G1-matching command ranges, validated via **two independent training lineages**: the
resumed axis-fix chain, and a from-scratch run proving the full final recipe
converges on its own. Root causes for the two biggest remaining symptoms from the
prior session state (crab-like gait, leg-crossing) were both found and fixed at the
source (URDF frame convention; a new anti-crossing reward term), not papered over
with more hyperparameter search.

### Next steps

- Continue watching the from-scratch validation run (`full_recipe_validation_fresh_001`)
  for longer-horizon stability now that it's matched the resumed lineage's quality.
- The Knee joint's URDF limit (-60°/5°) is off by 5° from the provided hardware spec
  (-65°/0°) — flagged, not yet fixed.
- Self-collision remains disabled; enabling it properly would need an actual
  pose/geometry fix for the crouch stance overlap found in section 13, not attempted.
- Housekeeping: prune superseded checkpoint directories from the axis-fix resume
  chain and the crossing-penalty iteration once findings above are confirmed stable,
  keeping only the runs cited as evidence plus the current best.

## Current config summary (updated)

**`Wato_Humanoid_V1_isaac.urdf`**
- `base_to_base_link` fixed joint `rpy`: `-1.5708 0 0` → `-1.5708 0 -1.5708` (adds the
  forward-axis correction on top of the pre-existing up-axis fix — see section 14)

**`modelCfg/wato_humanoid_v1.py`**
- `max_depenetration_velocity = 2.5`
- `self_collision = False`, `enabled_self_collisions = False` (tested `True`, failed
  catastrophically — see section 13)
- Ankle actuator: `stiffness=80.0, damping=6.0, effort_limit_sim=60.0, velocity_limit_sim=20.42`
  (others unchanged: Hip_F/Knee 222Nm/3.6652rad/s, Hip_A 120Nm/20.944rad/s, Hip_R 60Nm/20.42rad/s)

**`config/wato_humanoid_v1/agents/rsl_rl_ppo_cfg.py`**
- `empirical_normalization = True`
- `entropy_coef = 0.008`
- `init_noise_std = 1.0` (raised from 0.5 to match G1's inherited value)

**`config/wato_humanoid_v1/flat_env_cfg.py`**
- `track_lin_vel_xy_exp.std = 0.5` (unchanged from original G1-aligned value)
- `feet_air_time.weight = 2.0`, `threshold = 0.2` (raising threshold to 0.4 tested and
  reverted — see section 12)
- `double_support_penalty`: **removed entirely** (no longer needed — see section 12)
- `feet_crossing_penalty` (new — see section 15): `mdp.feet_crossing_l2`, squared
  penalty, `weight=-40.0`, `margin=0.08`, `preserve_order=True` on the foot
  `SceneEntityCfg`
- `commands.base_velocity`: `rel_standing_envs=0.1`, `lin_vel_x=(0.0, 1.0)` (forward
  only, reverted from symmetric), `lin_vel_y=(-0.5, 0.5)`, `ang_vel_z=(-1.0, 1.0)`
  (matches G1's flat-config scale)
- `reset_base.pose_range.yaw=(-3.14, 3.14)` (matches G1, full random heading)
- `reset_base.velocity_range` all axes ±0.2 (down from ±0.5), `reset_robot_joints.
  position_range=(0.8, 1.2)` (down from (0.5,1.5)) — reduced toward G1's gentler
  distribution
- `observations.policy.joint_vel.noise = Unoise(-0.3, 0.3)` (down from ±1.5)
- Action scales: `Hip_F=0.25, Knee=0.30, Hip_A=0.06, Hip_R=0.15` (raised from 0.06 —
  see section 13), `Ankle_R=0.08, Ankle_P=0.12`
- `WatoHumanoidFlatEnvCfg_PLAY` command ranges mirror the main training ranges

**`mdp/rewards.py`**
- `double_support_penalty` function removed (dead code once the reward term using it
  was deleted)
- `feet_crossing_l2` added (see section 15)

**Latest checkpoints (two active validation lineages)**:
- Resumed axis-fix chain: `logs/rsl_rl/wato_humanoid_flat/2026-07-14_17-46-44_urdf_forward_axis_fix_fresh_001/model_9900.pt`
- From-scratch full-recipe validation: `logs/rsl_rl/wato_humanoid_flat/2026-07-14_18-01-12_full_recipe_validation_fresh_001/model_3900.pt` (still training, paused for viewing)
