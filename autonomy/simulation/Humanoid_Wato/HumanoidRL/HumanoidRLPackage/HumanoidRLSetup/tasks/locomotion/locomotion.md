# Locomotion: Wato Humanoid V1 velocity tracking

Velocity-commanded bipedal locomotion for **Wato Humanoid Simulation Model V1** in Isaac Lab. The agent receives base velocity commands $(v_x, v_y, \omega_z)$ and is rewarded for tracking them while staying upright.

Asset: `autonomy/simulation/Humanoid_Wato/Wato Humanoid Simultion Model V1/` (SolidWorks URDF).
Training uses `urdf/Wato_Humanoid_V1_isaac.urdf`, which adds a Z-up `base` link and a fixed joint so the CAD Y-long frame stands in Isaac without a spawn rotation.

**Environments**

| Task ID | Terrain | Mode |
| :--- | :--- | :--- |
| `Isaac-Locomotion-Flat-WatoHumanoid-v0` | Plane | Train |
| `Isaac-Locomotion-Flat-WatoHumanoid-Play-v0` | Plane | Play |
| `Isaac-Locomotion-Rough-WatoHumanoid-v0` | Procedural rough | Train |
| `Isaac-Locomotion-Rough-WatoHumanoid-Play-v0` | Procedural rough | Play |

Legacy aliases `Isaac-Velocity-*` register the same configs.

## Train & play

Run inside the **`simulation_isaac`** container (Isaac Lab 2.3.2 / Sim 5.1). Host setup: [`docker/simulation/isaac_lab/QUICKSTART.md`](../../../../../../../../docker/simulation/isaac_lab/QUICKSTART.md).

```bash
# Host
cd ~/Desktop/humanoid && ./watod up -d && ./watod -t simulation_isaac_dev

# Inside container — from $RL_ROOT
cd $RL_ROOT

# Train — flat terrain
rl-train --task=Isaac-Locomotion-Flat-WatoHumanoid-v0 --headless

# Play — loads latest under logs/rsl_rl/wato_humanoid_flat/
rl-play --task=Isaac-Locomotion-Flat-WatoHumanoid-Play-v0 --num_envs=1

# Play — specific checkpoint
rl-play --task=Isaac-Locomotion-Flat-WatoHumanoid-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/wato_humanoid_flat/<run>/model_<iter>.pt
```

Rough-terrain variants: replace `Flat` with `Rough` and use `logs/rsl_rl/wato_humanoid_rough/`.

**Spawn / joint smoke checks**

```bash
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/diagnose_spawn.py \
  --task=Isaac-Locomotion-Flat-WatoHumanoid-v0 --headless --num_envs=1 --steps=10

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/diagnose_joints.py \
  --task=Isaac-Locomotion-Flat-WatoHumanoid-v0 --headless --num_envs=1
```

## Joints & bodies

| Role | Names |
| :--- | :--- |
| Root body | `base` (merged upright root from isaac URDF) |
| Feet (contacts) | `Foot_L`, `Foot_R` |
| Hip flexion | `Hip_F_L`, `Hip_F_R` |
| Hip abduction | `Hip_A_L`, `Hip_A_R` |
| Hip rotation | `Hip_R_L`, `Hip_R_R` |
| Knee | `Knee_L`, `Knee_R` |
| Ankle pitch / roll | `Ankle_P_*`, `Ankle_R_*` |

Config: `modelCfg/wato_humanoid_v1.py`, tasks under `config/wato_humanoid_v1/`.
