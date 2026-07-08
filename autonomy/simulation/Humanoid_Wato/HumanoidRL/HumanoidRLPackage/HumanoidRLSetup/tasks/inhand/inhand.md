# In-Hand: Wato hand cube reorientation

In-hand dexterous manipulation for the 20-DOF Wato hand (`hand_urdf.usd`) in Isaac Lab. The policy reorients a DexCube held in the palm toward a commanded goal orientation. **Play** shows a ghost goal cube marker (enabled via `WatoHandCubeEnvCfg_PLAY`).

Task setup and MDP code are adapted from [Isaac Lab](https://github.com/isaac-sim/IsaacLab) in-hand manipulation examples. Experimentation history: `TRAINING_LOG.md`.

**Environments**

| Task ID | Scene | Mode |
| :--- | :--- | :--- |
| `Isaac-Repose-Cube-WatoHand-v0` | Palm + DexCube | Train |
| `Isaac-Repose-Cube-WatoHand-Play-v0` | Palm + DexCube | Play |
| `Isaac-Repose-Cube-WatoHand-NoVelObs-v0` | Palm + DexCube | Train (no velocity obs) |
| `Isaac-Repose-Cube-WatoHand-NoVelObs-Play-v0` | Palm + DexCube | Play (no velocity obs) |

## Train & play

Run inside the **`simulation_il`** container (Isaac Lab 2.3.2 / Sim 5.1). Host setup: [`docker/simulation/isaac_il/QUICKSTART.md`](../../../../../../../../docker/simulation/isaac_il/QUICKSTART.md) §0–2.

```bash
# Host: start container
cd ~/Desktop/humanoid && ./watod up -d && ./watod -t simulation_il_dev

# Inside container — run from $RL_ROOT (HumanoidRL/)
cd $RL_ROOT

# Train (default 2048 envs; try 1024 or 512 if OOM)
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Repose-Cube-WatoHand-v0 --headless

# Play — omit --headless to see goal-orientation marker
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Repose-Cube-WatoHand-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Repose-Cube-WatoHand-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/wato_hand_cube/<run>/model_<iter>.pt
```

Shorthand aliases (after image rebuild): `rl-train --task=...` / `rl-play --task=... --num_envs=1`.

Checkpoints: `logs/rsl_rl/wato_hand_cube/`. PPO defaults: `max_iterations=5000`, `experiment_name=wato_hand_cube` (`config/wato_hand/agents/rsl_rl_ppo_cfg.py`).

## Scene & command

| Item | Value |
| :--- | :--- |
| Robot | `INHAND_WATO_HAND_CFG` (`modelCfg/wato_hand.py`) — palm-up, 20 DOF |
| Cube | Isaac Nucleus DexCube (instanceable), scale `(0.8, 0.8, 0.8)`, spawn `INHAND_CUBE_POS` |
| Goal command | `InHandReOrientationCommand` — resampled **on success** (not on a timer) |
| Goal position | Default cube spawn + `init_pos_offset = (0, 0, -0.04)` m (hold-in-palm target) |
| Goal orientation | Sampled on allowed `rotation_axes`; success when error **< 0.4 rad** |
| Curriculum (current) | `rotation_axes = ["z"]` — palm-normal spin only (`wato_hand_env_cfg.py`) |
| Startup event | `expand_abduction_limits` — overrides USD-baked MCP_A limits to ±27° |

`replicate_physics=True` on Wato hand (instanceable-friendly).

## Reward

All base reward terms are defined in `RewardsCfg` (`inhand_env_cfg.py`). Implementations live in `mdp/rewards.py` (task-specific) and Isaac Lab `mdp` (generic penalties). `WatoHandCubeEnvCfg` adds one extra term — `spread_activity` — in `config/wato_hand/wato_hand_env_cfg.py`.

| Category | Reward Function | Weight | Description |
| :--- | :--- | :--- | :--- |
| **Task** | Position tracking (`track_pos_l2`) | −3.0 | L2 distance to goal position (encourages holding cube in palm). |
| | Orientation tracking (`track_orientation_inv_l2`) | 10.0 | $1 / (\text{orientation\_error} + 0.1)$ — dense rotation signal. |
| | Success bonus (`success_bonus`) | 50.0 | Binary reward when orientation error **< 0.4 rad**. |
| | Object held bonus (`object_held_bonus`) | 0.5 | +1/step when cube within **0.10 m** of goal position. |
| | Angular velocity toward goal (`object_ang_vel_toward_goal`) | 0.2 | Positive component of cube spin aligned with goal (clamped ≥ 0). |
| | Spread activity (`mcp_a_spread_activity`) | 0.03 | Encourages MCP_A abduction velocity + deflection (Wato only). |
| **Penalties** | Object away (`object_away_penalty`) | −5.0 | Terminal penalty when `object_out_of_reach` fires. |
| | Action rate L2 | −0.05 | Penalises jerky finger commands. |
| | Joint velocity L2 | $-1.0 \times 10^{-4}$ | Penalises high joint speeds. |
| | Action L2 | $-1.0 \times 10^{-4}$ | Penalises large action magnitudes. |

## Terminations

| Termination | Condition |
| :--- | :--- |
| Time out | Episode length exceeds **20 s** (10 s in Play). |
| Max consecutive success | Goal reached **50 times** in a row within one episode. |
| Object out of reach | Cube drifts **> 0.3 m** from robot root. |
| Orientation stagnation | Orientation error stays **> 0.5 rad** for **150** consecutive steps. |

## Sim settings

| Setting | Value |
| :--- | :--- |
| `decimation` | 4 |
| `sim.dt` | 1/120 s (~120 Hz) |
| `episode_length_s` | 20.0 |
| Default `num_envs` | 2048 |
| Action smoothing | EMA joint-position targets, `alpha = 0.85` |
| PPO `num_steps_per_env` | 48 |
| PPO `gamma` | 0.998 |
| PPO `entropy_coef` | 0.0001 |
