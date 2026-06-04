# Badminton: timed intercept (Wato arm)

Timed 3D intercept for the Wato hand-arm (`ModelAssets/arm.usd`) in Isaac Lab. The policy receives **privileged** intercept timing (no vision): where to hit, seconds until shuttle arrival, and a one-step contact pulse. **Play/train with `debug_vis=True`** shows shrinking concentric rings (visual only); rings flash at min size on contact, then hide until the next resample.

**No shuttlecock in scene yet** — contact is proxied by racket link position + body velocity at the intercept instant. No extra sensors required for swing rewards (`body_lin_vel_w` from articulation state).

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
| Hit moment (reward pulse) | **One env step** (~67 ms) when lead time reaches 0 |
| Privileged command (5-D) | `[target_xyz, hit_moment_pulse, time_to_hit]` |

### Debug visualization (rings)

| Phase | Ring behavior |
| :--- | :--- |
| Countdown | Scale **1.0 → 0.35** linearly with remaining lead time |
| Contact | One step at **min scale** |
| After contact | Hidden until next resample (`post_hit_ring_hidden=True`) |

Ring colors (center → outer): red, yellow, green, blue + white center dot. Config: `mdp/ring_marker_utils.py`.

## Reward

Total reward is the weighted sum of all terms below. Config: `badminton_env_cfg.py`.

| Category | Reward Function | Weight | Description |
| :--- | :--- | :--- | :--- |
| **Prep** | Intercept proximity (`intercept_proximity_tanh`) | 0.25 | Tanh reward for moving toward intercept ($\sigma = 0.15$), always on. |
| **Contact** | Timed intercept (`timed_intercept_proximity`) | 2.0 | Proximity gated on hit-moment pulse ($\sigma = 0.10$). |
| | Timed hit (`timed_hit_bonus`) | 6.0 | Binary bonus inside **8 cm** sweet spot on hit pulse. |
| **Swing** | Timed swing speed (`timed_swing_speed`) | 3.0 | Racket speed at contact: $\tanh(\|v\| / 1.5\,\text{m/s})$ (no contact sensor). |
| | Timed swing through (`timed_swing_through_target`) | 4.0 | Forward speed along base→intercept axis, gated on sweet spot + hit pulse. |
| **Penalties** | Action rate L2 | −0.03 | Ramps to **−0.05** over 15k steps (curriculum). |
| | Joint velocity L2 (arm only) | −0.003 | Ramps to **−0.15** over 15k steps (curriculum). |

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

- Timed rewards are **sparse** (one step per intercept). Expect `intercept_proximity` to learn first; timing/swing may stay at zero without curriculum (e.g. fixed `lead_time`, wider hit pulse early).
- Last intercept in an episode can be **cut off** if `10 s + 3.5 s lead > 12 s` episode length.
- Retrain after reward/command changes; obs dim is **53** (5-D intercept command + joint state + last action).

## Future (phase 3+)

- Shuttlecock rigid body + contact sensor on racket
- Contact-force reward (see `tasks/force/`)
- Replace racket proxy body names in `mdp/rewards.py`
