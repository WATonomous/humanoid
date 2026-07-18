---
name: rl-training-iteration
description: Use whenever the user wants autonomous or semi-autonomous iterative RL/policy training — launching training runs, watching metrics, diagnosing plateaus or divergence, tweaking reward/env/hyperparameter config, and repeating until a target behavior emerges. Applies to robot locomotion, manipulation, game agents, or any Isaac Lab / Gym / RSL-RL / rl_games style training loop where the user says things like "keep iterating," "train until it works," "watch this run and adjust," or asks for autonomous tuning of a training pipeline. Not just for humanoid walking — the loop generalizes to any sparse-reward, exploration-hard RL task.
---

# RL training iteration loop

A disciplined loop for autonomously running, watching, diagnosing, and fixing an RL
training pipeline over many probes without wasting the user's time or compute.
Developed and validated over a full session getting a custom bipedal robot from
"falls in 0.8s no matter what" to "walking in three command directions."

For Isaac Lab / RSL-RL launch gotchas, GPU train+play conflicts, frame-convention
bugs, and reward-shaping specifics, also read [reference.md](reference.md).

## The loop, in order

1. **Launch a short probe first.** Never launch a 3000+ iteration run on an untested
   config change. Default to ~300-1000 iterations (or whatever the fastest meaningful
   checkpoint interval is for the framework) so a bad direction costs minutes, not
   hours.
2. **Watch on a schedule, not constantly.** Use a monitor/wakeup mechanism to check in
   every 100-250 iterations (scale to run length). Don't poll continuously — that
   wastes turns without adding signal.
3. **Track more than the headline metric.** At minimum: the primary reward/objective,
   episode length or equivalent survival metric, the specific sub-reward term you're
   trying to move, and training-health signals (value function loss, policy/action
   noise std, KL divergence). A healthy-looking primary metric can mask an unhealthy
   training process about to diverge.
4. **Cut early on a real plateau — don't wait it out.** If 2-3 consecutive checks
   (spaced by 100+ iterations) show the metric you care about flat or oscillating with
   no net movement, stop the run and change something. Letting a stalled run continue
   "just in case" is the single biggest time-waster in this loop. The user will tell
   you this directly if you get it wrong — treat it as a standing rule, not something
   to relearn each time.
5. **Diagnose with a decisive, isolating test — don't guess-and-check reward weights.**
   When several plausible causes exist, don't fix them one at a time by vibes. Design
   one test that cleanly separates hypotheses:
   - "Is the failure caused by the policy or by physics/task difficulty?" → run the
     exact same environment with a null/zero action, or with a freshly-initialized
     (untrained) policy. If the null case is fine but the trained-in-progress case
     fails identically to a random network, the problem is structural (env/physics/
     reward setup), not something more training will fix.
   - "Which of these N config differences from a known-good reference actually
     matters?" → change exactly one variable per probe, keep everything else fixed,
     and compare against the reference's own trajectory at the same checkpoint.
   - Six single-variable changes that all land on the identical plateau is itself a
     strong signal: it means the lever you're pulling isn't the bottleneck at all.
6. **Cross-check reward metrics against an actual video/live view before declaring
   victory or defeat.** Reward functions can be satisfied by degenerate behavior
   (contact-pattern fidgeting instead of real locomotion; a stepping *pattern* instead
   of net displacement). Conversely, sparse-frame video sampling can *miss* real,
   slow, subtle progress that's obvious in a live viewer. Trust a live GUI view over a
   handful of low-fps still frames — if they disagree, get better visual evidence
   before trusting either the numbers or the stills alone.
7. **When two objectives fight, curriculum-gate the harder one.** If a new reward term
   needs to compete with an already-strong existing incentive (e.g. penalizing an easy
   local-optimum behavior once the policy already scores near-ceiling on the main
   objective), don't add it from iteration 0 — the policy will spend its whole budget
   fighting two objectives before it's mastered either one. Start the new term's
   weight at 0 and use a step-count-gated curriculum (e.g. `modify_reward_weight` in
   Isaac Lab / RSL-RL) to activate it only after the easier objective is established.
   Also: if the first weight you try for the new term gets overwhelmed once the main
   reward approaches its ceiling, the fix is usually "make the new term's weight
   competitive at the ceiling," not "make the main reward's tolerance stricter" (the
   latter tends to kill the gradient entirely — see next point).
8. **Overcorrecting a reward's tolerance kills the gradient; revert to the known-good
   value rather than split the difference blindly.** Tightening a reward's forgiveness
   (e.g. shrinking an exp-kernel's std) to stop a policy from cheaply satisficing can
   overshoot into "the reward is near-zero for everyone, gradient is dead." If a
   tightened value regresses performance below the original, don't average the two —
   go back to the original and solve the satisficing problem a different way (e.g. the
   curriculum approach above, or a term that specifically targets the exploited
   pattern).
9. **Resume vs. restart from scratch — pick deliberately, not out of convenience.**
   - **Resume** when validating an *incremental* extension of an already-proven
     config (wider parameter range, longer training) where the point is to build on
     momentum you've already earned.
   - **Restart fresh** when: (a) the checkpoint you'd resume from is already anchored
     to the specific bad behavior you're trying to fix (resuming just fights the
     network's prior instead of cleanly exploring), or (b) you need to prove the
     *whole current recipe* — not just this one lucky trajectory — actually converges
     from scratch. Relying only on resumes forever means you never verify the reward/
     config design is sound on its own; you might just be riding one run's inertia.
   - Don't default to resuming purely to save wall-clock time once real behavioral
     progress is on the line — say explicitly which mode you're using and why.
10. **Clean up as you go.** Long iterative sessions generate dozens of throwaway probe
    logs and checkpoints. Periodically prune the ones that led nowhere (safe once
    findings are captured in a written summary) and keep only the lineage that
    produced the current best result plus the specific probes cited as evidence for
    documented findings.
11. **Write down what you tried and why, continuously — not just at the end.** Keep a
    running log (in a markdown file, comments in the config itself, or both) of: what
    was tried, the exact numeric result, and the conclusion. This is what lets step 5's
    "six things all hit the same plateau" pattern become visible at all, and it's what
    a future session (yours or the user's) needs to avoid re-treading the same dead
    ends.

## Communicating while iterating

- State the hypothesis before launching a probe ("testing whether X is the
  bottleneck"), not just the action.
- Report health metrics, not just the headline number, at each check-in — a terse
  "episode length 884, feet_air_time climbing, value loss stable" beats either silence
  or a wall of every logged field.
- When a run is cut early, say why in one sentence (what plateaued, for how long).
- When something is genuinely uncertain (e.g., "reward says X but I can't visually
  confirm it"), say so plainly rather than rounding up to a confident claim either way.
