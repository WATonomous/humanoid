# Badminton: timed intercept (Wato arm)

Timed EE intercept for the Wato hand-arm (`ModelAssets/arm.usd`) in Isaac Lab. The policy receives **privileged** swing targets (no vision): EE position, orientation, linear velocity at impact, seconds until arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings use the commanded orientation and flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link pose + velocity vs the commanded swing state at the intercept instant.

**Environments**

| Task ID | Scene | Mode |
| :--- | :--- | :--- |
| `Isaac-Badminton-Intercept-Humanoid-Arm-v0` | Ground + arm | Train |
| `Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0` | Ground + arm | Play |

## Train & play

Run from `HumanoidRL/` (the directory that contains `HumanoidRLPackage/`):

```bash
# Train
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-v0 --headless

# Play — omit --headless to see intercept rings
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/badminton_intercept_humanoid_arm/<run>/model_<iter>.pt
```

Checkpoints: `logs/rsl_rl/badminton_intercept_humanoid_arm/`. PPO defaults: `max_iterations=300`, `experiment_name=badminton_intercept_humanoid_arm` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).

## Scene & command

| Item | Value |
| :--- | :--- |
| Robot | `ARM_CFG` — 6-DOF arm actuated; fingers locked in racket-grip pose on reset |
| Racket proxy (rewards) | `forearm_v8_.*`, `DIP_INDEX_v1_.*` until a racket link exists in `arm.usd` |
| Intercept resample | Every **5 s** |
| Intercept position (base frame) | `x ∈ [-0.55, -0.15]`, `y ∈ [-0.45, 0.45]`, `z ∈ [0.15, 0.75]` m |
| Lead time (shuttle arrival) | Uniform **1.5–3.5 s** after each resample |
| Hit moment (pulse in command) | **`hit_moment_duration_s=0.13`** (~2 env steps) when lead time reaches 0 |
| Privileged command (12-D) | `[pos_xyz, quat_wxyz, vel_xyz, hit_pulse, time_to_hit]` — layout in `mdp/intercept_layout.py` |
| Impact orientation (base) | `roll ∈ [-0.15, 0.15]`, `pitch ∈ [0.45, 0.65]`, `yaw ∈ [-0.35, 0.35]` rad |
| Impact speed | Uniform **0.4–1.5 m/s** along base → intercept (arm-reachable) |

### Debug visualization (rings)

| Phase | Ring behavior |
| :--- | :--- |
| Countdown | Scale **1.0 → 0.35** linearly with remaining lead time |
| Contact | One step at **min scale** |
| After contact | Hidden until next resample (`post_hit_ring_hidden=True`) |

Ring colors (center → outer): red, yellow, green, blue + white center dot. Config: `mdp/ring_marker_utils.py`.

## EE end-state tracking (reward design)

Paper-style target: commanded **EE position, orientation, and velocity at impact time** (not game outcome, no shuttle in scene yet). Implementation:

- **Command** samples that 4D swing target + `time_to_hit` + hit pulse (`UniformInterceptCommand`).
- **Tracking errors** (world frame, best racket proxy link): `mdp/ee_tracking.py`.
- **Impact rewards** only on the hit pulse (and in-zone for ori/vel/swing-through).

**Intended behavior:** stay near a **ready** configuration while $t_{hit} > 0.55$ s (weak aim cue only), **launch** in the last **0.55 s**, then match the commanded **end state** on the hit pulse — not “reach the intercept early and hold.”

| Term | Weight | Description |
| :--- | :--- | :--- |
| `early_at_target_penalty` | −0.5 | Penalty for **camping at the intercept** while $t_{hit} > 0.25$ s. |
| `coarse_aim_toward_intercept_tanh` | 0.6 | Weak aim while $t_{hit} > 0.55$ s (`std=0.45`). |
| `timed_swing_approach_exp` | 4.0 | Launch window: strike-speed toward target × $\exp(-(d/0.4)^2)$, `hit_radius=0.13` m. |
| `ee_impact_position_hit_exp` | 10.0 | $\exp(-\|e_{pos}\|^2/\sigma^2)$ on hit pulse (`pos_std=0.10`). |
| `ee_impact_swing_through_hit_exp` | 0→12 | On hit + in zone: position × speed along **commanded strike axis** (`speed_std=0.6`). |
| `ee_impact_orientation_hit_exp` | 0→4 | On hit + in zone (optional; fingertip vs racket tilt). |
| `racket_speed_penalty_outside_swing_window` | −0.08 | Jitter when far and outside launch window. |
| Action rate / joint vel | −0.05 / −0.01 | Smoothness (curriculum ramps to −0.08 / −0.02). |

**Removed** (taught reach-and-hold): always-on `intercept_proximity`, urgency-gated position approach, product EE tracking with `prep_floor`.

### Curriculum (`mdp/curriculum.py`, `ramp_reward_weight`)

`common_step_counter` += 1 per sim step (~**24** per PPO iteration at `num_steps_per_env=24`).

| Term | Sim steps | ~Iter @ 300 max |
| :--- | :--- | :--- |
| `ee_impact_swing_through` 0→12 | 800–4500 | 33–188 |
| `ee_impact_orientation` 0→4 | 2500–6000 | 104–250 |

### Logged metrics (`UniformInterceptCommand._update_metrics`)

| Metric | Meaning |
| :--- | :--- |
| `Metrics/intercept/position_error` | Closest proxy link ↔ commanded intercept [m] |
| `Metrics/intercept/orientation_error` | Quaternion error [rad] |
| `Metrics/intercept/velocity_error` | $\|v - v_{cmd}\|$ [m/s] |
| `Metrics/intercept/hit_in_moment` | In **13 cm** zone on hit pulse |

## Terminations

| Termination | Condition |
| :--- | :--- |
| Time out | Episode length exceeds **12 s**. |

No success/failure termination on hit or miss.

## Sim settings

| Setting | Value |
| :--- | :--- |
| `decimation` | 4 |
| `sim.dt` | 1/60 s |
| `episode_length_s` | 12.0 |
| Default `num_envs` | 4096 (play: 50) |

## Training notes

- Watch **`ee_impact_position`** / **`ee_impact_swing_through`** and **`hit_in_moment`** — prep terms alone can look good while timing stays poor.
- Typical end of **300** iters: `position_error` ~0.30 m, `hit_in_moment` ~1–3%, impact rewards ~0.06 each; play to verify **late launch** vs early hold.
- If `position_error` stalls **> 0.25 m**, try fixed `lead_time=(2.0, 2.5)` early or slightly higher `coarse_aim` weight.
- Last intercept in an episode can be **cut off** if `10 s + 3.5 s lead > 12 s` episode length.
- Obs dim **60** (12-D command + joints + last action). **Retrain** after reward/command changes.

## Future (phase 3+)

- Shuttlecock rigid body + trajectory-derived commands
- Perception / estimated intercept in obs (drop privileged pose)
- Contact-force reward (see `tasks/force/`)
- Replace racket proxy body names in `mdp/rewards.py`
