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
| Hit moment (pulse in command) | **One env step** (~67 ms) when lead time reaches 0 |
| Privileged command (12-D) | `[pos_xyz, quat_wxyz, vel_xyz, hit_pulse, time_to_hit]` |
| Impact orientation (base) | `roll ∈ [-0.15, 0.15]`, `pitch ∈ [0.45, 0.65]`, `yaw ∈ [-0.35, 0.35]` rad |
| Impact speed | Uniform **0.4–1.5 m/s** along base → intercept (arm-reachable) |

### Debug visualization (rings)

| Phase | Ring behavior |
| :--- | :--- |
| Countdown | Scale **1.0 → 0.35** linearly with remaining lead time |
| Contact | One step at **min scale** |
| After contact | Hidden until next resample (`post_hit_ring_hidden=True`) |

Ring colors (center → outer): red, yellow, green, blue + white center dot. Config: `mdp/ring_marker_utils.py`.

## Reward

Phased prep → impact; velocity/orientation at hit are **curriculum-ramped** after position is learned. Config: `badminton_env_cfg.py`.

| Term | Weight | Description |
| :--- | :--- | :--- |
| `intercept_proximity_timed_tanh` | 2.0 | Move toward intercept; scaled by urgency (less reward if early). |
| `ee_position_approach_exp` | 1.5 | Fine position, urgency peaks at impact. |
| `early_at_target_penalty` | −0.4 | Penalty for waiting in the zone while $t_{hit} > 0.35$ s. |
| `ee_impact_position_hit_exp` | 8.0 | Position on hit pulse (~0.13 s window). |
| `ee_impact_velocity_hit_exp` | 0→10 | **Curriculum** sim steps 800–4500 (~iter 33–188 @ 300 max). |
| `ee_impact_orientation_hit_exp` | 0→4 | **Curriculum** sim steps 2500–6000 (~iter 104–250). |
| `racket_speed_penalty_far_from_target` | −0.15 | Fast motion when $>13$ cm from intercept. |
| Action rate / joint vel | −0.05 / −0.01 | Smoothness (curriculum ramps penalties). |

Hit window: `hit_moment_duration_s=0.13` (~2 env steps). Watch `hit_in_moment` and `ee_impact_position` in logs.

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

- Urgency weighting makes timing part of EE tracking (strongest signal as `time_to_hit → 0`). Early in each countdown the reward is small — consider lowering `urgency_time_constant` or a short curriculum with fixed `lead_time` if learning stalls.
- Last intercept in an episode can be **cut off** if `10 s + 3.5 s lead > 12 s` episode length.
- **Retrain** after this change; obs dim is **60** (12-D intercept command + joint state + last action).

## Future (phase 3+)

- Shuttlecock rigid body + trajectory-derived commands
- Perception / estimated intercept in obs (drop privileged pose)
- Contact-force reward (see `tasks/force/`)
- Replace racket proxy body names in `mdp/rewards.py`
