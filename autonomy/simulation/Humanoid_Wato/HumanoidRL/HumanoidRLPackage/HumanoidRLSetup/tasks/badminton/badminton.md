# Badminton: timed intercept (Wato arm)

<<<<<<< HEAD
<<<<<<< HEAD
Timed EE intercept for the Wato hand-arm (`UsdModelAssets/right_arm/right_arm_assembly/right_arm_assembly.usd`) in Isaac Lab. The policy receives **privileged** swing targets (no vision): EE position, orientation, linear velocity at impact, seconds until arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings use the commanded orientation and flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link pose + velocity vs the commanded swing state at the intercept instant.
=======
Timed 3D intercept for the Wato hand-arm (`ModelAssets/arm.usd`) in Isaac Lab. The policy receives **privileged** intercept timing (no vision): where to hit, seconds until shuttle arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link position + body velocity at the intercept instant. No extra sensors required for swing rewards (`body_lin_vel_w` from articulation state).
>>>>>>> bf63d8b3 (rl-badminton)
=======
Timed EE intercept for the Wato hand-arm (`ModelAssets/arm.usd`) in Isaac Lab. The policy receives **privileged** swing targets (no vision): EE position, orientation, linear velocity at impact, seconds until arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings use the commanded orientation and flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link pose + velocity vs the commanded swing state at the intercept instant.
>>>>>>> bfee0731 (improve-badminton-rl)

**Environments**

| Task ID | Scene | Mode |
| :--- | :--- | :--- |
| `Isaac-Badminton-Intercept-Humanoid-Arm-v0` | Ground + arm | Train |
| `Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0` | Ground + arm | Play |

## Train & play

<<<<<<< HEAD
Run inside the **`simulation_isaac`** container (Isaac Lab 2.3.2 / Sim 5.1). Host setup: [`docker/simulation/isaac_lab/QUICKSTART.md`](../../../../../../../../docker/simulation/isaac_lab/QUICKSTART.md) §0–2.

```bash
# Host: start container
cd ~/Desktop/humanoid && ./watod up -d && ./watod -t simulation_isaac_dev

# Inside container — run from $RL_ROOT (HumanoidRL/)
cd $RL_ROOT

# Train
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-v0 --headless

# Play — omit --headless to see intercept rings
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
=======
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
>>>>>>> bf63d8b3 (rl-badminton)
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/badminton_intercept_humanoid_arm/<run>/model_<iter>.pt
```

<<<<<<< HEAD
<<<<<<< HEAD
Shorthand aliases (after image rebuild): `rl-train --task=...` / `rl-play --task=... --num_envs=1`.

Checkpoints: `logs/rsl_rl/badminton_intercept_humanoid_arm/`. PPO defaults: `max_iterations=500`, `experiment_name=badminton_intercept_humanoid_arm` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).
=======
Checkpoints: `logs/rsl_rl/badminton_intercept_humanoid_arm/`. PPO defaults: `max_iterations=300`, `experiment_name=badminton_intercept_humanoid_arm` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).
>>>>>>> bf63d8b3 (rl-badminton)
=======
Checkpoints: `logs/rsl_rl/badminton_intercept_humanoid_arm/`. PPO defaults: `max_iterations=500`, `experiment_name=badminton_intercept_humanoid_arm` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)

## Scene & command

| Item | Value |
| :--- | :--- |
| Robot | `ARM_CFG` — 6-DOF arm actuated; fingers locked in racket-grip pose on reset |
<<<<<<< HEAD
<<<<<<< HEAD
| Racket proxy (rewards) | `forearm_v8_.*`, `DIP_INDEX_v1_.*` until a racket link exists in `right_arm_assembly.usd` |
=======
| Racket proxy (rewards) | `forearm_v8_.*`, `DIP_INDEX_v1_.*` until a racket link exists in `arm.usd` |
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)
| Intercept resample | After each cycle (**lead_time + hit window**); skipped if episode time is too short |
| Intercept position (base frame) | `x ∈ [-0.55, -0.15]`, `y ∈ [-0.45, 0.45]`, `z ∈ [0.15, 0.75]` m |
| Lead time (shuttle arrival) | Uniform **1.5–3.5 s** after each resample |
| Hit moment (pulse in command) | **`hit_moment_duration_s=0.13`** (~2 env steps) when lead time reaches 0 |
| Privileged command (12-D) | `[pos_xyz, quat_wxyz, vel_xyz, hit_pulse, time_to_hit]` — layout in `mdp/intercept_layout.py` |
| Impact orientation (base) | `roll ∈ [-0.15, 0.15]`, `pitch ∈ [0.45, 0.65]`, `yaw ∈ [-0.35, 0.35]` rad |
| Impact speed | Uniform **0.4–1.5 m/s** along base → intercept (arm-reachable) |
=======
| Racket proxy (rewards) | `forearm_v8_.*`, `DIP_INDEX_v1_.*` until a racket link exists in `arm.usd` |
| Intercept resample | Every **5 s** |
| Intercept position (base frame) | `x ∈ [-0.55, -0.15]`, `y ∈ [-0.45, 0.45]`, `z ∈ [0.15, 0.75]` m |
| Lead time (shuttle arrival) | Uniform **1.5–3.5 s** after each resample |
<<<<<<< HEAD
<<<<<<< HEAD
| Hit moment (reward pulse) | **One env step** (~67 ms) when lead time reaches 0 |
| Privileged command (5-D) | `[target_xyz, hit_moment_pulse, time_to_hit]` |
>>>>>>> bf63d8b3 (rl-badminton)
=======
| Hit moment (pulse in command) | **One env step** (~67 ms) when lead time reaches 0 |
| Privileged command (12-D) | `[pos_xyz, quat_wxyz, vel_xyz, hit_pulse, time_to_hit]` |
=======
| Hit moment (pulse in command) | **`hit_moment_duration_s=0.13`** (~2 env steps) when lead time reaches 0 |
| Privileged command (12-D) | `[pos_xyz, quat_wxyz, vel_xyz, hit_pulse, time_to_hit]` — layout in `mdp/intercept_layout.py` |
>>>>>>> 00aee69e (improve-badminton-rl)
| Impact orientation (base) | `roll ∈ [-0.15, 0.15]`, `pitch ∈ [0.45, 0.65]`, `yaw ∈ [-0.35, 0.35]` rad |
| Impact speed | Uniform **0.4–1.5 m/s** along base → intercept (arm-reachable) |
>>>>>>> bfee0731 (improve-badminton-rl)

### Debug visualization (rings)

| Phase | Ring behavior |
| :--- | :--- |
| Countdown | Scale **1.0 → 0.35** linearly with remaining lead time |
| Contact | One step at **min scale** |
| After contact | Hidden until next resample (`post_hit_ring_hidden=True`) |

Ring colors (center → outer): red, yellow, green, blue + white center dot. Config: `mdp/ring_marker_utils.py`.

<<<<<<< HEAD
<<<<<<< HEAD
## EE end-state tracking (reward design)

Paper-style target: commanded **EE position + orientation + full 3D velocity** at impact time.
<<<<<<< HEAD

- **Command** (`UniformInterceptCommand`): intercept pose, quat, `vel_xyz` + `time_to_hit` + hit pulse.
- **Reward** (`ee_state_tracking_timed_exp`):

  $r = \exp(-\|e_{pos}\|^2)\,\exp(-\|e_{vel}\|^2)\,\exp(-e_{ori}^2)\,\bigl(e^{-t_{hit}/\tau} + \text{hit\_bonus}\,\mathbb{1}_{hit}\bigr)$

  Full **velocity vector** must match (not just speed along one axis). Orientation in the main product.

**Intended behavior:** full swing from ready; timing from `time_to_hit` in obs.

| Term | Weight | Description |
| :--- | :--- | :--- |
| `ee_state_tracking_timed_exp` | 12.0 | `pos_std=0.10`, `vel_std=0.6`, `ori_std=0.8`, `timing_std=0.45`, `hit_bonus=2`. |
| `early_at_target_penalty` | −0.3 | Penalty for waiting at intercept early. |
| Action rate / joint vel | −0.05 / −0.01 | Smoothness (curriculum ramps penalties). |

Hit window: `hit_moment_duration_s=0.20` s.

### Curriculum

| Term | Sim steps |
| :--- | :--- |
| `action_rate` → −0.08 | 25000 |
| `joint_vel` → −0.02 | 25000 |

### Logged metrics (`UniformInterceptCommand._update_metrics`)

| Metric | Meaning |
| :--- | :--- |
| `Metrics/intercept/position_error` | Closest proxy link ↔ commanded intercept [m] |
| `Metrics/intercept/orientation_error` | Quaternion error [rad] |
| `Metrics/intercept/velocity_error` | $\|v - v_{cmd}\|$ [m/s] |
| `Metrics/intercept/hit_in_moment` | In **13 cm** zone on hit pulse |
=======
## Reward
=======
## EE end-state tracking (reward design)
>>>>>>> 00aee69e (improve-badminton-rl)

Paper-style target: commanded **EE position, orientation, and velocity at impact time** (not game outcome, no shuttle in scene yet). Implementation:
=======
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)

- **Command** (`UniformInterceptCommand`): intercept pose, quat, `vel_xyz` + `time_to_hit` + hit pulse.
- **Reward** (`ee_state_tracking_timed_exp`):

  $r = \exp(-\|e_{pos}\|^2)\,\exp(-\|e_{vel}\|^2)\,\exp(-e_{ori}^2)\,\bigl(e^{-t_{hit}/\tau} + \text{hit\_bonus}\,\mathbb{1}_{hit}\bigr)$

  Full **velocity vector** must match (not just speed along one axis). Orientation in the main product.

**Intended behavior:** full swing from ready; timing from `time_to_hit` in obs.

<<<<<<< HEAD
| Category | Reward Function | Weight | Description |
| :--- | :--- | :--- | :--- |
| **Prep** | Intercept proximity (`intercept_proximity_tanh`) | 0.25 | Tanh reward for moving toward intercept ($\sigma = 0.15$), always on. |
| **Contact** | Timed intercept (`timed_intercept_proximity`) | 2.0 | Proximity gated on hit-moment pulse ($\sigma = 0.10$). |
| | Timed hit (`timed_hit_bonus`) | 6.0 | Binary bonus inside **8 cm** sweet spot on hit pulse. |
| **Swing** | Timed swing speed (`timed_swing_speed`) | 3.0 | Racket speed at contact: $\tanh(\|v\| / 1.5\,\text{m/s})$ (no contact sensor). |
| | Timed swing through (`timed_swing_through_target`) | 4.0 | Forward speed along base→intercept axis, gated on sweet spot + hit pulse. |
| **Penalties** | Action rate L2 | −0.03 | Ramps to **−0.05** over 15k steps (curriculum). |
| | Joint velocity L2 (arm only) | −0.003 | Ramps to **−0.15** over 15k steps (curriculum). |
>>>>>>> bf63d8b3 (rl-badminton)
=======
| Term | Weight | Description |
| :--- | :--- | :--- |
| `ee_state_tracking_timed_exp` | 12.0 | `pos_std=0.10`, `vel_std=0.6`, `ori_std=0.8`, `timing_std=0.45`, `hit_bonus=2`. |
| `early_at_target_penalty` | −0.3 | Penalty for waiting at intercept early. |
| Action rate / joint vel | −0.05 / −0.01 | Smoothness (curriculum ramps penalties). |

<<<<<<< HEAD
<<<<<<< HEAD
Hit window: `hit_moment_duration_s=0.13` (~2 env steps). Watch `hit_in_moment` and `ee_impact_position` in logs.
>>>>>>> bfee0731 (improve-badminton-rl)
=======
**Removed** (taught reach-and-hold): always-on `intercept_proximity`, urgency-gated position approach, product EE tracking with `prep_floor`.
=======
Hit window: `hit_moment_duration_s=0.20` s.
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)

### Curriculum

| Term | Sim steps |
| :--- | :--- |
| `action_rate` → −0.08 | 25000 |
| `joint_vel` → −0.02 | 25000 |

### Logged metrics (`UniformInterceptCommand._update_metrics`)

| Metric | Meaning |
| :--- | :--- |
| `Metrics/intercept/position_error` | Closest proxy link ↔ commanded intercept [m] |
| `Metrics/intercept/orientation_error` | Quaternion error [rad] |
| `Metrics/intercept/velocity_error` | $\|v - v_{cmd}\|$ [m/s] |
| `Metrics/intercept/hit_in_moment` | In **13 cm** zone on hit pulse |
>>>>>>> 00aee69e (improve-badminton-rl)

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)
- Watch **`ee_state_tracking`**, **`velocity_error`**, **`orientation_error`**, and **`hit_in_moment`** together — position alone can look OK while swing vector/ori stay wrong.
- Product reward is strict: all three must be reasonable near impact for high return.
- If learning stalls, try looser `ori_std` (1.0) or fixed `lead_time=(2.0, 2.5)` for early training.
- Lead time is **clamped to remaining episode time** so rings always finish shrinking before reset.
<<<<<<< HEAD
=======
- Watch **`ee_impact_position`** / **`ee_impact_swing_through`** and **`hit_in_moment`** — prep terms alone can look good while timing stays poor.
- Typical end of **300** iters: `position_error` ~0.30 m, `hit_in_moment` ~1–3%, impact rewards ~0.06 each; play to verify **late launch** vs early hold.
- If `position_error` stalls **> 0.25 m**, try fixed `lead_time=(2.0, 2.5)` early or slightly higher `coarse_aim` weight.
- Last intercept in an episode can be **cut off** if `10 s + 3.5 s lead > 12 s` episode length.
>>>>>>> 00aee69e (improve-badminton-rl)
=======
>>>>>>> cd302498 (add-dextrahrgb-and-modify-badminton-rl)
- Obs dim **60** (12-D command + joints + last action). **Retrain** after reward/command changes.

## Future (phase 3+)

- Shuttlecock rigid body + trajectory-derived commands
- Perception / estimated intercept in obs (drop privileged pose)
=======
- Timed rewards are **sparse** (one step per intercept). Expect `intercept_proximity` to learn first; timing/swing may stay at zero without curriculum (e.g. fixed `lead_time`, wider hit pulse early).
=======
- Urgency weighting makes timing part of EE tracking (strongest signal as `time_to_hit → 0`). Early in each countdown the reward is small — consider lowering `urgency_time_constant` or a short curriculum with fixed `lead_time` if learning stalls.
>>>>>>> bfee0731 (improve-badminton-rl)
- Last intercept in an episode can be **cut off** if `10 s + 3.5 s lead > 12 s` episode length.
- **Retrain** after this change; obs dim is **60** (12-D intercept command + joint state + last action).

## Future (phase 3+)

<<<<<<< HEAD
- Shuttlecock rigid body + contact sensor on racket
>>>>>>> bf63d8b3 (rl-badminton)
=======
- Shuttlecock rigid body + trajectory-derived commands
- Perception / estimated intercept in obs (drop privileged pose)
>>>>>>> bfee0731 (improve-badminton-rl)
- Contact-force reward (see `tasks/force/`)
- Replace racket proxy body names in `mdp/rewards.py`
