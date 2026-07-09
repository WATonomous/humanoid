# Badminton: timed intercept (Wato arm)

Timed EE intercept for the Wato hand-arm (`UsdModelAssets/right_arm/right_arm_assembly/right_arm_assembly.usd`) in Isaac Lab. The policy receives **privileged** swing targets (no vision): EE position, orientation, linear velocity at impact, seconds until arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings use the commanded orientation and flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link pose + velocity vs the commanded swing state at the intercept instant.

**Environments**

| Task ID | Scene | Mode |
| :--- | :--- | :--- |
| `Isaac-Badminton-Intercept-Humanoid-Arm-v0` | Ground + arm | Train |
| `Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0` | Ground + arm | Play |

## Train & play

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
  --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/badminton_intercept_humanoid_arm/<run>/model_<iter>.pt
```

Shorthand aliases (after image rebuild): `rl-train --task=...` / `rl-play --task=... --num_envs=1`.

Checkpoints: `logs/rsl_rl/badminton_intercept_humanoid_arm/`. PPO defaults: `max_iterations=500`, `experiment_name=badminton_intercept_humanoid_arm` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).

## Scene & command

| Item | Value |
| :--- | :--- |
| Robot | `ARM_CFG` — 6-DOF arm actuated; fingers locked in racket-grip pose on reset |
| Racket proxy (rewards) | `forearm_v8_.*`, `DIP_INDEX_v1_.*` until a racket link exists in `right_arm_assembly.usd` |
| Intercept resample | After each cycle (**lead_time + hit window**); skipped if episode time is too short |
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

Paper-style target: commanded **EE position + orientation + full 3D velocity** at impact time.

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

- Watch **`ee_state_tracking`**, **`velocity_error`**, **`orientation_error`**, and **`hit_in_moment`** together — position alone can look OK while swing vector/ori stay wrong.
- Product reward is strict: all three must be reasonable near impact for high return.
- If learning stalls, try looser `ori_std` (1.0) or fixed `lead_time=(2.0, 2.5)` for early training.
- Lead time is **clamped to remaining episode time** so rings always finish shrinking before reset.
- Obs dim **60** (12-D command + joints + last action). **Retrain** after reward/command changes.

## Future (phase 3+)

- Shuttlecock rigid body + trajectory-derived commands
- Perception / estimated intercept in obs (drop privileged pose)
- Contact-force reward (see `tasks/force/`)
- Replace racket proxy body names in `mdp/rewards.py`
