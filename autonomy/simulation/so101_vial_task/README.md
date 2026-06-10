# SO101 vial task (Isaac Lab Gym)

Workshop-parity stack for the SO101 **vial → rack** task: registered Gym envs, success detection, policy eval, and rich LeRobot recording with depth/segmentation MP4 export.

Assets live under `assets/lerobot/` (sync with `./assets/lerobot/sync_so101_vial_assets.sh --full`).

## Install

```bash
cd autonomy/simulation/so101_vial_task
pip install -e .
```

Requires Isaac Lab, LeRobot, and synced USD/HDRI assets.

## Registered Gym tasks

| Task id | Use |
|---------|-----|
| `Lerobot-So101-Teleop-Vials-To-Rack` | Teleop + record (no DR) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR` | Teleop + record + domain randomization |
| `Lerobot-So101-Teleop-Vials-To-Rack-Eval` | Policy eval (success termination) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval` | Policy eval + DR |

Observation groups: `policy` (joints), `visual` (RGB/depth/seg), `subtask` (grasp/placed flags). Success uses gripper contact sensor + rack slot geometry (`humanoid_so101_vial_task/mdp/terms.py`).

## 1. Leader teleop + rich recording (Gym)

Physical SO101 Leader drives sim; **S** start/stop, **R** reset world, **C** cancel episode while recording.

```bash
cd autonomy/simulation/so101_vial_task
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p scripts/lerobot_agent.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR \
  --port /dev/ttyACM0 \
  --repo_id humanoid/so101_vial_dr \
  --repo_root ../../datasets/record_so101_gym/001 \
  --task_name "vial to rack" \
  --save_mp4 --depth --instance_id_seg
```

Optional MP4 sidecars: `--save_mp4`, `--depth`, `--instance_id_seg` (via `LeRobotRecorder`).

## 2. Policy eval in sim

Local LeRobot checkpoint (ACT, etc.):

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type lerobot \
  --policy_path outputs/train/so101_act_v1/checkpoints/last/pretrained_model \
  --num_episodes 10 \
  --rename_map '{"ego":"observation.images.ego","external_D455":"observation.images.external_D455"}'
```

Remote GR00T server:

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p scripts/lerobot_eval.py \
  --policy_type groot --policy_host localhost --policy_port 5555
```

Reports success rate when `success` termination fires (vial placed in rack slot).

## 3. Lightweight teleop (InteractiveScene)

For keyboard/leader collection without Gym, use the existing path:

`autonomy/simulation/Teleop/so101-leader teleoperation/` + `autonomy/il/` (`--record --cameras --domain_rand`).

That path is RGB-only in LeRobot; use **this** package for depth/seg MP4 sidecars and automatic success scoring.

## Layout

```
so101_vial_task/
├── humanoid_so101_vial_task/
│   ├── tasks/          # Gym env cfgs + registration
│   ├── mdp/            # resets, obs, success/contact terms
│   ├── utils/          # leader interface, LeRobotRecorder, keyboard
│   └── gr00t_client/   # remote GR00T policy client
└── scripts/
    ├── lerobot_agent.py   # teleop + rich record
    └── lerobot_eval.py    # policy rollout + success rate
```
