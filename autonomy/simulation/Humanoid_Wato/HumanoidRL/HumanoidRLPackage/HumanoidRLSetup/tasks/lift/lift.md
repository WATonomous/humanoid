# Lift: SO101 cube manipulation

Pick-and-place for the SO101 follower arm (`so101_follower_good.usd`) in Isaac Lab. Reach rewards use a **TCP** from `ee_frame` (`FrameTransformer` on link `gripper` with the URDF `gripper_frame_joint` offset). **Play** shows that TCP as RGB axes at the palm; the green cuboid is the commanded object goal (enabled via `SO101LiftEnvCfg_PLAY`).

**Environments**

| Task ID | Scene | Mode |
| :--- | :--- | :--- |
| `Isaac-Lift-Cube-SO101-v0` | Table + DexCube | Train |
| `Isaac-Lift-Cube-SO101-Play-v0` | Table + DexCube | Play |

## Train & play

Run inside the **`simulation_il`** container (Isaac Lab 2.3.2 / Sim 5.1). Host setup: [`docker/simulation/isaac_lab/QUICKSTART.md`](../../../../../../../../docker/simulation/isaac_lab/QUICKSTART.md) §0–2.

```bash
# Host: start container
cd ~/Desktop/humanoid && ./watod up -d && ./watod -t simulation_il_dev

# Inside container — run from $RL_ROOT (HumanoidRL/)
cd $RL_ROOT

# Train (default 512 envs; SO101 USD is heavy — try 256 if OOM)
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Lift-Cube-SO101-v0 --headless

# Play — omit --headless to see green goal cuboid + RGB ee_tcp frame
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Lift-Cube-SO101-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Lift-Cube-SO101-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/lift_so101/<run>/model_<iter>.pt
```

Shorthand aliases (after image rebuild): `rl-train --task=...` / `rl-play --task=... --num_envs=1`.

Checkpoints: `logs/rsl_rl/lift_so101/`. PPO defaults: `max_iterations=2000`, `experiment_name=lift_so101` (`config/HumanoidRLEnv/agents/rsl_rl_ppo_cfg.py`).

## Scene & command

| Item | Value |
| :--- | :--- |
| Robot | `SO101_FOLLOWER_CFG` (`modelCfg/so101.py`) |
| Cube | Isaac Nucleus DexCube, scale `(0.5, 0.5, 0.5)`, spawn ~`(0.3, 0, 0.035)` |
| EE for rewards | `ee_frame` → TCP on `gripper` + URDF offset (`joint_pos_env_cfg.py`) |
| Goal command | `object_pose`, resampled every **5 s** (matches `episode_length_s=5`) |
| Goal ranges (base frame) | `x ∈ [-0.4, -0.2]`, `y ∈ [-0.15, 0.15]`, `z ∈ [0.1, 0.2]` m |

`replicate_physics=False` on SO101 (non-instanceable USD).

## Reward (SO101)

Total reward is the weighted sum of all terms below. Config: `lift_env_cfg.py`.

| Category | Reward Function | Weight | Description |
| :--- | :--- | :--- | :--- |
| **Task** | Reaching object (`object_ee_distance`) | 1.0 | Tanh reward for TCP proximity to cube ($\sigma = 0.3$). |
| | Lifting object (`object_is_lifted`) | 18.0 | Binary reward when cube height **> 0.08 m**. |
| | Object goal tracking (`object_goal_distance`) | 20.0 | Tanh reward for cube → commanded goal ($\sigma = 0.3$), gated on height **> 0.03 m**. |
| | Object goal tracking (fine) | 5.0 | Same, tighter kernel ($\sigma = 0.05$). |
| **Penalties** | Action rate L2 | $-1.0 \times 10^{-5}$ | Ramps to **−0.1** over 10k steps (curriculum). |
| | Joint velocity L2 | $-1.0 \times 10^{-5}$ | Ramps to **−0.1** over 10k steps (curriculum). |

## Terminations

| Termination | Condition |
| :--- | :--- |
| Time out | Episode length exceeds **5 s**. |
| Object dropping | Cube root height falls below **−0.05 m**. |

## Sim settings

| Setting | Value |
| :--- | :--- |
| `decimation` | 2 |
| `sim.dt` | 0.01 s (100 Hz) |
| `episode_length_s` | 5.0 |
| Default `num_envs` | 512 |
