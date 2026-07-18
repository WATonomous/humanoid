# Isaac Lab / sim-stack notes

Framework-specific gotchas that sit beside the general iteration loop. Read these when
working in Isaac Lab, RSL-RL, Isaac Sim, or a custom robot URDF/asset.

## Verify the coordinate/frame convention against the actual asset geometry

Don't assume a robot's root/body frame matches the framework's convention (e.g. Isaac
Lab assumes local +X = forward). If a policy converges to a persistent, otherwise
inexplicable gait pathology (a "crab walk," a chronic tracking-error plateau on one
axis that never improves no matter what else is tuned), check the URDF/asset directly:
compare the origins of a bilaterally-symmetric pair of joints (e.g. left/right hip).
Whichever axis they differ on is the true lateral axis — if the framework's tracking
reward assumes forward is a *different* axis, every velocity command has been silently
asking the robot to strafe along its own side-to-side axis instead of walking forward.
This class of bug will not show up in reward curves (tracking rewards can still be
high — the policy is correctly satisfying the command, just along the wrong physical
axis) and will not be fixed by more training, different rewards, or hyperparameter
tuning. It requires a geometry-level fix (e.g. adding a corrective rotation to the
URDF's root-to-body fixed joint) and invalidates every existing checkpoint, since the
whole notion of "forward" changes at the root frame — this is one of the few
unambiguous cases that demands a fresh run, not a resume.

## Hardware/GPU constraints on running train + play simultaneously

On a single, memory-constrained GPU, a second Isaac Sim process (e.g. `play.py`) can
fail to initialize while a training run is already using the PhysX GPU context, even
with plenty of VRAM headroom (symptom: `Exception: Failed to get DOF velocities from
backend` during `ArticulationData.__init__`, not a memory error). If this happens,
don't loop retrying — pause training (kill the process; the last saved checkpoint per
`save_interval` is still there), view/play the checkpoint, then resume training
afterward. Resuming this way *does* create a brand-new timestamped run directory each
time (not a continuation in the same folder) even when `--run_name` is repeated — find
the latest checkpoint by directory mtime, not by name, when looking for what to view
or resume from next.

## Launch invocation gotchas (Isaac Lab specifically)

- Use the framework's own launcher wrapper (e.g. `isaaclab.sh -p script.py`), not a
  bare `python`/`python3` — the bare interpreter is missing the framework's own
  dependencies even if it's technically the "right" Python for the simulator.
- Run from the working directory the script's own relative paths expect (e.g. Isaac
  Lab's `play.py`/`train.py` resolve `logs/rsl_rl/<experiment>/...` relative to `cwd`,
  not relative to the script's own location) — check this before assuming a
  `FileNotFoundError` is a real missing-checkpoint problem.
- Set `PYTHONPATH` explicitly if the project's own package isn't installed/on the
  default path.

## Reward shaping: hard clamps cause stutter; asymmetric terms need explicit ordering

- A linear `clamp(threshold - x, min=0)` penalty has a discontinuous gradient right at
  the threshold boundary. If the trained behavior shows a visible stutter or repeated
  small "flinch" corrections right around some physical boundary (a spacing margin, a
  joint limit, a contact threshold), suspect the reward shape itself before tuning
  weights further — squaring the clamped term (or another smooth falloff) usually
  removes the flinch while keeping large violations penalized hard. Note squaring
  shrinks the typical numeric magnitude for small violations, so the weight usually
  needs to go up to compensate — expect to retune it empirically rather than guess
  once and trust it.
- Any reward term that depends on which of two symmetric bodies is which (e.g. a
  signed left-vs-right computation, not just a symmetric sum) must not rely on
  `SceneEntityCfg`'s default body ordering — it defaults to the asset's own internal
  body index order, which is not guaranteed to match the order you wrote in a
  `body_names` list. Pass `preserve_order=True` explicitly whenever left/right (or any
  other order-dependent) semantics matter.
