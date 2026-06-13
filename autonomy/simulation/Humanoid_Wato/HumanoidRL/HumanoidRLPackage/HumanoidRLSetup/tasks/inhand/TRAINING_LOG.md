# In-Hand Cube Reorientation — Training Log

> Task setup and MDP code (commands, terminations, rewards, etc.) are adapted from
> [Isaac Lab](https://github.com/isaac-sim/IsaacLab) in-hand manipulation examples.

## Task
20-DOF Wato hand, in-hand cube reorientation (Isaac-Repose-Cube-WatoHand-v0).
Palm-up orientation, cube spawned at `(-0.01, 0.09, 0.5)` in the palm.

---

## What Was Tried

### 1. MCP_A abduction range expansion (WORKS — kept)
**Problem:** USD bakes ±8.6° abduction limits; fingers could barely splay.
**Fix:** Expanded `_MCP_A_LIMIT` to ±27° (AllegroHand level). `expand_abduction_limits` startup event calls `apply_wato_hand_joint_limits()` to override USD-baked values in PhysX at runtime.
**Result:** Fingers now have meaningful splay range.

---

### 2. Reward shaping — object holding (WORKS — kept)
**Problem:** Cube dropped in ~30 steps; no gradient to learn to grip.
**Fixes:**
- `track_pos_l2 weight=-3.0` (continuous drift penalty)
- `object_away_penalty weight=-5.0` (terminal drop penalty)
- `object_held_bonus weight=0.5, threshold=0.10m` — dense +1/step when cube within 10 cm. **Key fix.**

**Result:** Episode length doubled ~60 → 105-119 steps. Drop rate halved.

---

### 3. Stagnation termination fix (WORKS — kept)
**Problem:** `orientation_error_threshold=0.12` in stagnation but success threshold=0.4 — good episodes ended prematurely.
**Fix:** Stagnation threshold raised to 0.5 rad, `stagnant_steps` 90 → 150.

---

### 4. PPO config tuning (WORKS — kept)
- `entropy_coef: 0.002 → 0.0001` — stopped rewarding randomness.
- `num_steps_per_env: 24 → 48` — better return estimates.
- `success_bonus weight: 250 → 50` — reduced VF loss spikes.

---

### 5. Angular velocity toward goal reward (WEAK — kept at low weight)
**Idea:** Reward angular velocity component aligned with goal direction.
**Problem:** First attempt gave negative reward (random spin anti-aligned on average) — suppressed all rotation. Clamped to 0, re-enabled at weight=0.2 once holding stabilized.
**Outcome:** Stays flat at ~0.016 regardless of policy quality. Goal resampling on success resets the angular velocity alignment, pinning the average near zero. Kept as a weak directional signal only.

---

### 6. EMA alpha reduction — 0.95 → 0.8 (WORKS)
**alpha=0.5:** Policy didn't use extra bandwidth. Reverted.
**alpha=0.8:** Bandwidth ~1 Hz (vs ~0.25 Hz at 0.95). `action_rate_l2` rose -0.13 → -0.30. **Key unlock for rotation.** Orientation error broke below 1.5 consistently.

---

### 7. Z-axis curriculum (WORKS — converged)
**Problem:** Full 3D goals too hard to explore. Policy never discovered rotation despite holding.
**Fix:** `rotation_axes = ["z"]` — goals restricted to palm-normal spin only.
**Result (5000 iters, 245M steps, 1024 envs):**

| Metric | Start | End |
|---|---|---|
| `orientation_error` | 2.2+ | 0.98-1.15 (best: 0.87) |
| `episode_length` | 85 | 350-432 |
| `action_rate_l2` (logged) | -0.08 | -0.30 to -0.34 |
| `consecutive_success` | ~0 | 0.20-0.32 |

---

### 8. Instanceable USD + scaling to 1024 envs (WORKS)
`replicate_physics = True` was already set. The existing `hand_urdf.usd` has no companion `_meshes.usd` but runs at 1024 envs without OOM — effectively sufficient.

**To make fully instanceable (not yet done):** Isaac Sim GUI → URDF Importer → "Create Instanceable Asset" → produces `hand_urdf.usd` + `instanceable_meshes.usd`. Update `_HAND_USD_PATH` in `wato_hand_cfg.py`.

**Throughput:**
- 256 envs: ~11k steps/s
- 512 envs: ~21k steps/s
- 1024 envs: ~36k steps/s

---

### 9. Reward rebalancing to unblock rotation (WORKS)
**Problem:** Policy held well (~43%) but didn't rotate — `object_held_bonus` dominated.
**Changes:**
- `track_orientation_inv_l2`: weight 5.0 → **10.0**
- `object_held_bonus`: weight 2.0 → **0.5**
- `object_ang_vel_toward_goal`: re-enabled at weight **0.2**

**Caution:** Doubling orientation weight caused VF loss 36-57 → 131-221. Stabilized after ~160 iters.

---

### 10. Full 3D expansion attempt (FAILED — geometric limitation)
**Setup:** Resumed from z-axis checkpoint (iter 5000) with `rotation_axes = ["x", "y", "z"]`.
**Observed:** orientation_error jumped 1.0 → 2.18. `action_rate_l2` dropped -0.30 → -0.08. After 500+ iters, no improvement.

**Root cause:** Palm-up hand cannot achieve x/y tilt because:
1. No back-face contact — the cube face away from fingertips has nothing to push against.
2. Thumb doesn't reach the back face (circumduction swings side-to-side, not to the opposite face).
3. Gravity holds the cube flat — tilting requires lifting a face with no contact point.

Shadow/Allegro hands mount the thumb on the *opposite side* of the palm, giving true opposition. Fixing this on Wato requires hardware repositioning.

---

### 11. Smoothness fixes — action_rate ×5, joint_vel ×4, alpha 0.85 (WORKS)
**Problem:** After goal match, fingers still jerked. Jerking resets `consecutive_success` counter, pinning `max_consecutive_success` at 0.
**Changes:**
- `action_rate_l2` weight: -0.01 → **-0.05** (5×)
- `joint_vel_l2` weight: -2.5e-5 → **-1e-4** (4×)
- `alpha`: 0.8 → **0.85**

**Result:** Best orientation_error hit **0.72 rad** at iter 5138 (new record). Stagnation rate dropped significantly. Jerking reduced but `max_consecutive_success` still 0 — policy still can't hold for 50 consecutive steps.

---

### 12. Scale to 2048 envs (MARGINAL IMPROVEMENT — stagnation)
**Throughput:** 1024 envs ~36k steps/s → 2048 envs ~53k steps/s (1.5× not 2× — GPU near saturation).
**VF loss:** Improved from 150-250 range down to 82-157 — bigger batch gives better return estimates.
**Orientation error:** Oscillates 0.94-1.19, best mean batch 0.946. No sustained improvement beyond the ~1.0 floor.

**Stagnation diagnosis:**
- `consecutive_success` ~0.22-0.28 — policy reaches the goal (~25% of envs within 0.4 rad) but jerking immediately resets the counter.
- `action_noise_std` declining 0.96 → 0.91, entropy declining 27.4 → 26.3 — policy converging to a deterministic local attractor.
- `max_consecutive_success` stuck at 0 throughout — the hold skill was never acquired.

**Root cause of floor:** The spin skill and hold skill are in tension. The action_rate penalty pushes smoothness but the policy learned to spin-and-jerk rather than spin-and-stabilize. Once at goal, fingers keep issuing corrective actions that overshoot, knocking the cube above the 0.4 rad threshold.

---

## Final State

| Metric | Start | Current Best |
|---|---|---|
| `orientation_error` (mean) | 2.2+ rad | ~0.94 rad (mean batch); best single: 0.72 |
| `episode_length` | 85 steps | 400-470 steps |
| `object_out_of_reach` | 5-7/iter | 1.0-1.5/iter |
| `orientation_stagnation` | 2.4/iter | 0.27-0.44/iter |
| `consecutive_success` | ~0 | 0.22-0.28 |
| `max_consecutive_success` | 0 | 0 (never achieved) |

---

## Key Config (current)
- `alpha = 0.85`, `num_envs = 2048`, `replicate_physics = True`
- `rotation_axes = ["z"]`
- `track_orientation_inv_l2 weight = 10.0`
- `object_held_bonus weight = 0.5`, `object_ang_vel_toward_goal weight = 0.2`
- `success_bonus weight = 50`, `action_rate_l2 weight = -0.05`, `joint_vel_l2 weight = -1e-4`
- `entropy_coef = 0.0001`, `num_steps_per_env = 48`, `gamma = 0.998`

---

## Next Steps / Open Problems

1. **Break the stagnation floor (~1.0 rad)** — options:
   - Much stronger smoothness: `action_rate_l2` weight -0.05 → -0.15 to finally kill jerking. Risk: may suppress rotation bandwidth.
   - Fresh training run with all current hyperparams — policy may have converged to a poor attractor that a new random init escapes.

2. **Full 3D reorientation** requires hardware change: reposition thumb to opposite side of palm (true opposition), or mount hand palm-sideways so z-axis becomes a tilt direction. More compute will not overcome the geometric limitation.

3. **Fully instanceable USD** — create proper `instanceable_meshes.usd` via Isaac Sim GUI for cleaner multi-env physics sharing.

4. **Scale compute** — 4096+ envs (multi-GPU) is the next throughput step if hardware is available.
