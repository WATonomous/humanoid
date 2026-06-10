# SO101 vial task (Isaac Lab Gym)

Workshop-parity stack for the SO101 **vial → rack** task: registered Gym envs, success detection, policy eval, and rich LeRobot recording with depth/segmentation MP4 export.

<<<<<<< HEAD
## Run this (recommended)

Use the **`simulation_isaac`** watod Docker image (Isaac Lab 2.3.2 + LeRobot). Do **not** use host `env_isaaclab` for IL — wrong Python/stack version.

| Doc | Contents |
|-----|----------|
| **[`docker/simulation/isaac_lab/QUICKSTART.md`](../../../docker/simulation/isaac_lab/QUICKSTART.md)** | Exact copy-paste: build, train ACT, eval |
| **[`docker/simulation/isaac_lab/README.md`](../../../docker/simulation/isaac_lab/README.md)** | Full setup, troubleshooting, workflows |

```bash
# host
ACTIVE_MODULES="simulation_isaac"   # in watod-config.local.sh
./watod up -d && ./watod -t simulation_isaac_dev
```

Assets: `./assets/lerobot/sync_so101_vial_assets.sh --full` (see [`assets/lerobot/README.md`](../../../assets/lerobot/README.md)).
=======
Assets live under `assets/lerobot/` (sync with `./assets/lerobot/sync_so101_vial_assets.sh --full`).

## Install

```bash
cd autonomy/simulation/so101_vial_task
pip install -e .
```

Requires Isaac Lab, LeRobot, and synced USD/HDRI assets.
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

## Registered Gym tasks

| Task id | Use |
|---------|-----|
| `Lerobot-So101-Teleop-Vials-To-Rack` | Teleop + record (no DR) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR` | Teleop + record + domain randomization |
| `Lerobot-So101-Teleop-Vials-To-Rack-Eval` | Policy eval (success termination) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval` | Policy eval + DR |

Observation groups: `policy` (joints), `visual` (RGB/depth/seg), `subtask` (grasp/placed flags). Success uses gripper contact sensor + rack slot geometry (`humanoid_so101_vial_task/mdp/terms.py`).

<<<<<<< HEAD
Cameras: `ego` (gripper), `external_D455` (lightbox) — matches [CursedRock17/so101_teleop_vials_sim_and_real](https://huggingface.co/datasets/CursedRock17/so101_teleop_vials_sim_and_real).

## 1. Train ACT (inside `simulation_isaac` container)

```bash
il-train \
  --dataset.repo_id=CursedRock17/so101_teleop_vials_sim_and_real \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=/workspace/humanoid/outputs/train/so101_hf_act \
  --policy.device=cuda \
  --steps=10000 \
  --batch_size=8 \
  --save_freq=5000 \
  --job_name=so101_hf_smoke
```

See [QUICKSTART.md](../../../docker/simulation/isaac_lab/QUICKSTART.md) for full train/eval commands and pitfalls.

## 2. Policy eval in sim (inside container)

```bash
cd /workspace/humanoid/autonomy/simulation/so101_vial_task

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type lerobot \
  --policy_path /workspace/humanoid/outputs/train/so101_hf_act/checkpoints/last/pretrained_model \
  --num_episodes 5
```

- **Local ACT:** omit `--rename_map` (sim cameras already `ego` / `external_D455`).
- **GR00T:** `--policy_type groot` and `--rename_map '{"external_D455": "front", "ego": "wrist"}'`.

Reports success rate when `success` termination fires (vial placed in rack slot).

## 3. Leader teleop + rich recording (needs USB leader)
=======
## 1. Leader teleop + rich recording (Gym)
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

Physical SO101 Leader drives sim; **S** start/stop, **R** reset world, **C** cancel episode while recording.

```bash
<<<<<<< HEAD
cd $TASK_ROOT
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_agent.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR \
  --port /dev/ttyACM0 \
  --repo_root /workspace/humanoid/datasets/record_so101_gym/001 \
  --save_mp4 --depth --instance_id_seg
```

## 4. Lightweight teleop (InteractiveScene)

RGB-only path without Gym:

`autonomy/simulation/Teleop/so101_leader_teleoperation/` + `autonomy/il/` (`--record --cameras --domain_rand`).

Use **this** package for depth/seg MP4 sidecars and automatic success scoring.
=======
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
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

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
<<<<<<< HEAD

## Host install (legacy — not for IL docker workflow)

Only if developing outside Docker on a matching Isaac Lab 2.3.2 + Python 3.11 stack:

```bash
cd autonomy/simulation/so101_vial_task
pip install -e .
```

Requires Isaac Lab, LeRobot, and synced USD/HDRI assets.
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
