# Locomotion: G1 velocity tracking

Velocity-commanded bipedal locomotion for the Unitree G1 in Isaac Lab. The agent receives random base velocity commands $(v_x, v_y, \omega_z)$ and is rewarded for tracking them while staying upright.

**Environments**

| Task ID | Terrain | Mode |
| :--- | :--- | :--- |
| `Isaac-Locomotion-Flat-G1-v0` | Plane | Train |
| `Isaac-Locomotion-Flat-G1-Play-v0` | Plane | Play |
| `Isaac-Locomotion-Rough-G1-v0` | Procedural rough | Train |
| `Isaac-Locomotion-Rough-G1-Play-v0` | Procedural rough | Play |

Legacy aliases `Isaac-Velocity-*` register the same configs for backward compatibility with existing checkpoints and scripts.

## Train & play

<<<<<<< HEAD
Run inside the **`simulation_isaac`** container (Isaac Lab 2.3.2 / Sim 5.1). Host setup: [`docker/simulation/isaac_lab/QUICKSTART.md`](../../../../../../../../docker/simulation/isaac_lab/QUICKSTART.md) §0–2.

```bash
# Host: start container
cd ~/Desktop/humanoid && ./watod up -d && ./watod -t simulation_isaac_dev

# Inside container — run from $RL_ROOT (HumanoidRL/)
cd $RL_ROOT

# Train — flat terrain
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Locomotion-Flat-G1-v0 --headless

# Play — loads latest checkpoint from logs/rsl_rl/g1_flat/
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Locomotion-Flat-G1-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
=======
Run from `HumanoidRL/` (the directory that contains `HumanoidRLPackage/`):

```bash
# Train — flat terrain
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Locomotion-Flat-G1-v0 --headless

# Play — loads latest checkpoint from logs/rsl_rl/g1_flat/
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Locomotion-Flat-G1-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
>>>>>>> f3902ba0 (modify-locomotion-rl)
  --task=Isaac-Locomotion-Flat-G1-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/g1_flat/<run>/model_<iter>.pt
```

<<<<<<< HEAD
Shorthand aliases (after image rebuild): `rl-train --task=...` / `rl-play --task=... --num_envs=1`.

Rough-terrain variants: replace `Flat` with `Rough` and use experiment dir `logs/rsl_rl/g1_rough/`.

## LEGR leg model

The local LEGR leg model lives under:

```text
autonomy/simulation/Humanoid_Wato/UsdModelAssets/LEGR/
```

The original SolidWorks URDF export is preserved as `urdf/LEGR.urdf`. The Isaac Lab import path uses
`urdf/LEGR_isaac.urdf`, which fixes reversed mirrored joint limits and changes the left ankle pitch joint
`U_P_L` from fixed to revolute to mirror `U_P_R`. It also adds an upright `base` link above the original
`Hip` link so the locomotion task has a stable root body name after fixed joints are merged during USD
conversion.

Registered LEGR task IDs:

| Task ID | Terrain | Mode |
| :--- | :--- | :--- |
| `Isaac-Locomotion-Flat-LEGR-v0` | Plane | Train |
| `Isaac-Locomotion-Flat-LEGR-Play-v0` | Plane | Play |
| `Isaac-Locomotion-Rough-LEGR-v0` | Procedural rough | Train |
| `Isaac-Locomotion-Rough-LEGR-Play-v0` | Procedural rough | Play |

PowerShell smoke train from `HumanoidRL/`:

```powershell
$env:ISAACLAB_ROOT = "<path-to-IsaacLab>"
$env:PYTHONPATH = (Get-Location).Path
$env:WARP_CACHE_PATH = "<repo-root>\.warp_cache"

& "$env:ISAACLAB_ROOT\isaaclab.bat" -p HumanoidRLPackage\rsl_rl_scripts\train.py --task Isaac-Locomotion-Flat-LEGR-v0 --headless --num_envs 1 --max_iterations 1
```

PowerShell spawn/contact diagnostic from `HumanoidRL/`:

```powershell
& "$env:ISAACLAB_ROOT\isaaclab.bat" -p HumanoidRLPackage\rsl_rl_scripts\diagnose_spawn.py --task Isaac-Locomotion-Flat-LEGR-v0 --headless --num_envs 1 --steps 10
```

PowerShell play with the D3D12 experience:

```powershell
& "$env:ISAACLAB_ROOT\isaaclab.bat" -p HumanoidRLPackage\rsl_rl_scripts\play.py --task Isaac-Locomotion-Flat-LEGR-Play-v0 --num_envs 1 --checkpoint logs\rsl_rl\legr_flat\<run>\model_<iter>.pt --experience "$env:ISAACLAB_ROOT\apps\isaaclab.python.d3d12.kit" --rendering_mode performance
```

=======
Rough-terrain variants: replace `Flat` with `Rough` and use experiment dir `logs/rsl_rl/g1_rough/`.

>>>>>>> f3902ba0 (modify-locomotion-rl)
## Reward (G1 flat)

Total reward is the weighted sum of all terms below. Config: `config/g1/flat_env_cfg.py`.

| Category | Reward Function | Symbol | Weight | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Task** | Track linear velocity (XY) | $r_{\text{track\_lin\_vel\_xy\_exp}}$ | 1.0 | Exponential reward for tracking commanded linear velocity in the XY plane. |
| | Track angular velocity (Z) | $r_{\text{track\_ang\_vel\_z\_exp}}$ | 1.0 | Exponential reward for tracking commanded yaw rate. |
| **Penalties** | Linear velocity (Z) L2 | $r_{\text{lin\_vel\_z\_l2}}$ | −0.2 | L2 penalty on base vertical velocity. |
| | Angular velocity (XY) L2 | $r_{\text{ang\_vel\_xy\_l2}}$ | −0.05 | L2 penalty on base roll/pitch rates. |
| | Joint torques L2 | $r_{\text{dof\_torques\_l2}}$ | $-2.0 \times 10^{-6}$ | L2 penalty on hip/knee torques. |
| | Joint accelerations L2 | $r_{\text{dof\_acc\_l2}}$ | $-1.0 \times 10^{-7}$ | L2 penalty on joint accelerations. |
| | Action rate L2 | $r_{\text{action\_rate\_l2}}$ | −0.005 | L2 penalty on action smoothness. |
| | Feet air time | $r_{\text{feet\_air\_time}}$ | 0.75 | Reward for foot air time above threshold (0.4 s) when moving. |
| | Undesired contacts | $r_{\text{undesired\_contacts}}$ | −1.0 | Penalty for thigh contacts above force threshold. |
| | Flat orientation L2 | $r_{\text{flat\_orientation\_l2}}$ | 0.0 | L2 penalty on base tilt (disabled on flat). |
| **Constraints** | Joint position limits | $r_{\text{dof\_pos\_limits}}$ | 0.0 | Soft joint-limit penalty (disabled on flat). |

## Reward (G1 rough)

Rough terrain adds height-scan observations, terrain curriculum, and extra terms. Config: `config/g1/rough_env_cfg.py`.

| Category | Reward Function | Symbol | Weight | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Task** | Track linear velocity (XY) | $r_{\text{track\_lin\_vel\_xy\_exp}}$ | 1.0 | Yaw-frame exponential velocity tracking. |
| | Track angular velocity (Z) | $r_{\text{track\_ang\_vel\_z\_exp}}$ | 2.0 | World-frame yaw-rate tracking. |
| **Penalties** | Termination | $r_{\text{termination\_penalty}}$ | −200.0 | Penalty on episode termination (fall). |
| | Feet air time (biped) | $r_{\text{feet\_air\_time}}$ | 0.25 | Single-stance air/contact time reward. |
| | Feet slide | $r_{\text{feet\_slide}}$ | −0.1 | Penalty for foot sliding while in contact. |
| | Flat orientation L2 | $r_{\text{flat\_orientation\_l2}}$ | −1.0 | L2 penalty on base tilt. |
| | Joint position limits | $r_{\text{dof\_pos\_limits}}$ | −1.0 | Ankle joint soft-limit penalty. |
| | Joint deviation (hip) | $r_{\text{joint\_deviation\_hip}}$ | −0.1 | L1 deviation from default hip pose. |
| | Joint deviation (arms) | $r_{\text{joint\_deviation\_arms}}$ | −0.1 | L1 deviation from default arm pose. |
| | Joint deviation (fingers) | $r_{\text{joint\_deviation\_fingers}}$ | −0.05 | L1 deviation from default finger pose. |
| | Joint deviation (torso) | $r_{\text{joint\_deviation\_torso}}$ | −0.1 | L1 deviation from default torso pose. |
| | Linear velocity (Z) L2 | $r_{\text{lin\_vel\_z\_l2}}$ | 0.0 | Disabled on rough G1. |
| | Angular velocity (XY) L2 | $r_{\text{ang\_vel\_xy\_l2}}$ | −0.05 | L2 penalty on roll/pitch rates. |
| | Joint torques L2 | $r_{\text{dof\_torques\_l2}}$ | $-1.5 \times 10^{-7}$ | L2 penalty on hip/knee/ankle torques. |
| | Joint accelerations L2 | $r_{\text{dof\_acc\_l2}}$ | $-1.25 \times 10^{-7}$ | L2 penalty on hip/knee accelerations. |
| | Action rate L2 | $r_{\text{action\_rate\_l2}}$ | −0.005 | L2 penalty on action smoothness. |
