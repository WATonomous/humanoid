# In-Hand Cube Reorientation — Training Log

## Task
20-DOF Wato hand, in-hand cube reorientation (Isaac-Repose-Cube-WatoHand-v0).
Palm-up orientation, cube spawned at `(-0.01, 0.09, 0.5)` in the palm.

---

## What Was Tried

### 1. MCP_A abduction range expansion (WORKS — kept)
**Problem:** USD bakes ±8.6° abduction limits; fingers could barely splay, making reorientation geometrically impossible.  
**Fix:** Expanded `_MCP_A_LIMIT` in `wato_hand_cfg.py` from ±8.6° → ±27° (AllegroHand level). Added `expand_abduction_limits` startup event in `wato_env_cfg.py` that calls `apply_wato_hand_joint_limits()` to push new limits into PhysX at runtime, overriding the USD-baked values.  
**Result:** Fingers now have meaningful splay range. Self-collision was already disabled so no issues.

---

### 2. Reward shaping — object holding (WORKS — kept)
**Problem:** Cube dropped in ~30 steps (1 sec); no gradient for the policy to learn to grip. Episode too short for any useful signal.  
**Fixes applied (cumulative):**
- `track_pos_l2 weight=-3.0` (continuous penalty for cube drifting from goal position)
- `object_away_penalty weight=-5.0` (terminal penalty on drop)
- `object_held_bonus weight=2.0, threshold=0.10m` — dense binary +1/step when cube is within 10 cm of goal. **This was the key fix.**

**Result:** Episode length doubled from ~60 → 105-119 steps. `object_out_of_reach` rate halved from 1.5-2.1 → 0.6-0.8 per batch.

---

### 3. Stagnation termination fix (WORKS — kept)
**Problem:** `orientation_error_threshold=0.12` in stagnation but `orientation_success_threshold=0.4`. Episodes achieving error 0.12-0.4 rad (better than needed!) were simultaneously getting success credit AND counting toward stagnation — ending good episodes at 90 steps.  
**Fix:** Raised stagnation threshold to 0.5 rad (above success threshold) and extended `stagnant_steps` 90 → 150.  
**Result:** Longer episodes, less premature termination.

---

### 4. PPO config tuning (PARTIAL — kept)
- `entropy_coef: 0.002 → 0.0001` — stopped the optimizer from rewarding randomness. `action_noise_std` stopped climbing from 1.02 → 1.05.
- `num_steps_per_env: 24 → 48` — each rollout now covers ~44% of a typical episode; better return estimates for the critic.
- `success_bonus weight: 250 → 50` — original weight caused VF loss to spike to 74-88 as policy occasionally scored success. Reducing to 50 kept VF loss in 26-55 range.

---

### 5. Angular velocity toward goal reward (FAILED — disabled)
**Idea:** Reward the component of object angular velocity aligned with the goal direction. `object_ang_vel_toward_goal` with weight=0.1.  
**Problem:** Reward was negative (-0.003) because cube's random spin was anti-aligned with the goal on average. This taught the policy to *suppress* all rotation (hold rigidly) to avoid the penalty.  
**Fix attempt:** Clamped to 0, increased weight to 0.5. Value became +0.05 but this was purely mechanical (5x weight + clamping removed negatives), not actual policy improvement. Orientation error unchanged.  
**Outcome:** Disabled. The cube barely moves and there is insufficient angular velocity signal to reward.

---

### 6. EMA alpha reduction 0.95 → 0.5 (FAILED — reverted)
**Idea:** alpha=0.95 gives ~1.5 Hz effective finger bandwidth. Reducing to 0.5 gives ~15 Hz, potentially allowing faster rotation torques.  
**Result:** `action_rate_l2` did NOT increase (policy didn't use the extra bandwidth). `orientation_error` unchanged. `object_out_of_reach` slightly worse. Reverted to 0.95.

---

### 7. Z-axis curriculum (WORKS — current)
**Problem:** Full 3D random orientation goal is too hard to explore from scratch. Policy never discovered how to rotate the cube despite holding it.  
**Fix:** `rotation_axes = ["z"]` in `wato_env_cfg.py` — goals are now only rotations about the palm normal (z-axis). The cube just needs to spin in the plane of the palm.  
**Result:** Immediate improvement:
- `track_orientation_inv_l2`: 0.24 → 0.88-1.05 (near 4x)
- `orientation_error`: 2.2-2.4 → 1.78-2.07 (trending down)
- `consecutive_success`: 0.03 → 0.20-0.27
- `time_out` episodes appearing for first time

**Note on oscillation:** Orientation error oscillates (~1.8-2.1) rather than trending smoothly because `update_goal_on_success=True` resamples a new random z-goal when error < 0.4, immediately resetting error to ~1.5. The mean oscillates around `0.25 × ~0.2 + 0.75 × ~2.3 ≈ 1.85`.

---

## Current State (iter ~600, ~7.4M steps)

| Metric | Start | Current |
|---|---|---|
| Episode length | ~30 steps | ~120-137 steps |
| `object_out_of_reach` rate | 5-7 | 0.5-0.9 |
| `orientation_error` | 2.2-2.4 rad (random) | 1.82-1.95 rad |
| `consecutive_success` | ~0 | 0.20-0.27 |
| `track_orientation_inv_l2` | ~0.25 | ~0.75-1.05 |

**Bottleneck:** Plateaued at iter ~400. Policy learns z-rotation intermittently (occasional good episodes) but hasn't converged to a consistent strategy. Root cause is sample efficiency: 256 envs × 48 steps = 12,288 samples/gradient step. Standard IsaacLab in-hand setups use 8,192+ envs = 393k samples/step.

---

## Next Steps

1. **Make USD instanceable** — convert `wato_hand/urdf/hand_urdf.urdf` to instanceable format using:
   ```bash
   /home/hy/IsaacLab/isaaclab.sh -p /home/hy/IsaacLab/scripts/tools/convert_instanceable.py \
     /home/hy/Desktop/humanoid/autonomy/simulation/Humanoid_Wato/wato_hand/urdf/ \
     /home/hy/Desktop/humanoid/autonomy/simulation/Humanoid_Wato/wato_hand/urdf_instanceable/ \
     --make-instanceable --conversion-type urdf
   ```
   Then update `_HAND_USD_PATH` in `wato_hand_cfg.py` to point to the new USD and change `replicate_physics = False` → `True` in `wato_env_cfg.py`. This allows scaling beyond 256 envs without OOM.
2. **Increase num_envs** — after instanceable conversion, try `--num_envs 1024+`. Standard IsaacLab in-hand setups use 8192 envs; even 1024 gives 4x better gradient quality.
3. **Longer training** — z-axis skill may require 50k+ iterations (~15 hours overnight) to converge at 256 envs.
4. **Curriculum expansion** — once `orientation_error` consistently < 1.5 and `consecutive_success` > 0.4 on z-axis, expand back to `rotation_axes = ["x", "y"]` for full 3D reorientation.
5. **Re-enable ang_vel reward** (with clamp, weight ~0.3) once holding is fully stable — provides rotation direction signal once the policy has enough bandwidth to act on it.
