# Lift: SO101 cube manipulation

Pick-and-place for the SO101 follower arm (`so101_follower_good.usd`) in Isaac Lab. Reach rewards use a **TCP** from `ee_frame` (`FrameTransformer` on link `gripper` with the URDF `gripper_frame_joint` offset). **Play** shows that TCP as RGB axes at the palm; the green cuboid is the commanded object goal.

**Environments**


| Task ID                         | Scene        | Mode  |
| ------------------------------- | ------------ | ----- |
| `Isaac-Lift-Cube-SO101-v0`      | Table + cube | Train |
| `Isaac-Lift-Cube-SO101-Play-v0` | Table + cube | Play  |


## Train & play

Run from `HumanoidRL/` (the directory that contains `HumanoidRLPackage/`):

```bash
# Train (default 512 envs; SO101 USD is heavy — try 256 if OOM, then --num_envs=1024 if stable)
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py \
  --task=Isaac-Lift-Cube-SO101-v0 --headless

# Play — omit --headless to see green goal cuboid + RGB ee_tcp frame (tune offset in joint_pos_env_cfg.py)
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Lift-Cube-SO101-Play-v0 --num_envs=1

# Play — specific checkpoint
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py \
  --task=Isaac-Lift-Cube-SO101-Play-v0 --num_envs=1 \
  --checkpoint logs/rsl_rl/lift_so101/<run>/model_<iter>.pt
```

## Reward (SO101)

Total reward is the weighted sum of all terms below. Config: `lift_env_cfg.py`. Robot config: `modelCfg/so101.py`.


| Category      | Reward Function             | Symbol                                     | Weight                | Description                                                                                      |
| ------------- | --------------------------- | ------------------------------------------ | --------------------- | ------------------------------------------------------------------------------------------------ |
| **Task**      | Reaching object             | $r_{\text{reachingobject}}$                | 1.0                   | Tanh-kernel reward for `gripper` proximity to the cube ($\sigma = 0.3$).                         |
|               | Lifting object              | $r_{\text{liftingobject}}$                 | 18.0                  | Binary reward when cube height exceeds 0.04 m.                                                   |
|               | Object goal tracking        | $r_{\text{objectgoaltracking}}$            | 20.0                  | Tanh-kernel reward for moving the lifted cube toward the commanded target pose ($\sigma = 0.3$). |
|               | Object goal tracking (fine) | $r_{\text{objectgoaltrackingfinegrained}}$ | 5.0                   | Fine-grained goal tracking with tighter kernel ($\sigma = 0.05$).                                |
| **Penalties** | Action rate L2              | $r_{\text{actionrate}}$                    | $-1.0 \times 10^{-5}$ | L2 penalty on action smoothness (ramps to −0.1 over 10k steps via curriculum).                   |
|               | Joint velocity L2           | $r_{\text{jointvel}}$                      | $-1.0 \times 10^{-5}$ | L2 penalty on joint velocities (ramps to −0.1 over 10k steps via curriculum).                    |


## Terminations


| Termination     | Condition                             |
| --------------- | ------------------------------------- |
| Time out        | Episode length exceeds 5 s.           |
| Object dropping | Cube root height falls below −0.05 m. |


