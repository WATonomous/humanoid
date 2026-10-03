# SO101 vial task (Isaac Lab Gym)

Workshop-parity stack for the SO101 **vial → rack** task: registered Gym envs, success detection, policy eval, and rich LeRobot recording with depth/segmentation MP4 export.

Exact workflow used for **HF dataset → ACT train → sim eval** (no physical arm).
Stack: Isaac Lab 2.3.2 / Sim 5.1 / LeRobot 0.4.3 / ACT / watod `simulation_isaac`.

## Run this (recommended)

Use the **`simulation_isaac`** watod Docker image (Isaac Lab 2.3.2 + LeRobot). Do **not** use host `env_isaaclab` for IL — wrong Python/stack version.

| Doc | Contents |
|-----|----------|
| **[`docker/simulation/isaac_lab/README.md`](../../../docker/simulation/isaac_lab/README.md)** | Container setup: host setup, build, launch, reference |

```bash
# host
ACTIVE_MODULES="simulation_isaac"   # in watod-config.local.sh
./watod up -d && ./watod -t simulation_isaac
```

Assets (once, on host): `./assets/lerobot/sync_so101_vial_assets.sh --full` (see [`assets/lerobot/README.md`](../../../assets/lerobot/README.md)).

## Registered Gym tasks

| Task id | Use |
|---------|-----|
| `Lerobot-So101-Teleop-Vials-To-Rack` | Teleop + record (no DR) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR` | Teleop + record + domain randomization |
| `Lerobot-So101-Teleop-Vials-To-Rack-Eval` | Policy eval (success termination) |
| `Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval` | Policy eval + DR |

Observation groups: `policy` (joints), `visual` (RGB/depth/seg), `subtask` (grasp/placed flags). Success uses gripper contact sensor + rack slot geometry (`humanoid_so101_vial_task/mdp/terms.py`).

Cameras: `ego` (gripper), `external_D455` (lightbox) — matches [CursedRock17/so101_teleop_vials_sim_and_real](https://huggingface.co/datasets/CursedRock17/so101_teleop_vials_sim_and_real).

## 1. Train ACT on public HF dataset (inside `simulation_isaac` container)

Dataset: [CursedRock17/so101_teleop_vials_sim_and_real](https://huggingface.co/datasets/CursedRock17/so101_teleop_vials_sim_and_real) — **140 demos**, 26,516 frames.

**Smoke test** (10k steps ≈ 3 epochs over 140 demos, ~1–2 h):

```bash
mkdir -p /workspace/humanoid/outputs/train/so101_hf_act

train-policy \
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

**Longer run** (better policy; 50k–100k steps is more typical for usable ACT):

```bash
train-policy \
  --dataset.repo_id=CursedRock17/so101_teleop_vials_sim_and_real \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=/workspace/humanoid/outputs/train/so101_hf_act_50k \
  --policy.device=cuda \
  --steps=50000 \
  --batch_size=8 \
  --save_freq=10000 \
  --job_name=so101_hf_50k
```

Checkpoint path when done:

```
/workspace/humanoid/outputs/train/so101_hf_act/checkpoints/last/pretrained_model
```

Same path on host: `~/Desktop/humanoid/outputs/train/so101_hf_act/...`

### Train pitfalls (learned the hard way)

| Wrong | Right |
|-------|-------|
| `lerobot-train` (bare CLI) | `train-policy` or `$PYTHON -m lerobot.scripts.lerobot_train` |
| `-m lerobot.scripts.train` | `-m lerobot.scripts.lerobot_train` |
| `--training.num_epochs=10` | `--steps=10000` (default is 100k) |
| omit `--policy.push_to_hub=false` (`policy.repo_id missing`) | always set `false` for local-only training |
| `pip install humanoid-robot-learning[sim]` in image | breaks Isaac torch — Dockerfile uses `--no-deps` only |

## 2. Policy eval in sim (inside container)

**Must** run from `$TASK_ROOT`. **Do not** pass `--rename_map` for local ACT (cameras already `ego` / `external_D455`).

```bash
cd /workspace/humanoid/src/simulation/so101_vial_task

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type lerobot \
  --policy_path /workspace/humanoid/outputs/train/so101_hf_act/checkpoints/last/pretrained_model \
  --num_episodes 5
```

- Isaac GUI opens (X11 to host display).
- **R** = manual reset world.
- Prints success rate when episodes finish; the env terminates on `success` (vial placed in rack slot).
- 10k-step smoke policy can still succeed sometimes (e.g. 33–50% in early tests).

**GR00T eval (remote server)** — `--rename_map` is for GR00T / mismatched camera names, not local ACT:

```bash
cd $TASK_ROOT
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type groot \
  --policy_host localhost \
  --policy_port 5555 \
  --rename_map '{"external_D455": "front", "ego": "wrist"}'
```

### Eval pitfalls

| Wrong | Right |
|-------|-------|
| Run from `/workspace/humanoid` (`can't open file .../scripts/lerobot_eval.py`) | `cd $TASK_ROOT` first |
| `--rename_map '{"ego":"observation.images.ego",...}'` | omit for ACT — causes `KeyError: 'ego'` |
| Ctrl+C when frozen after crash | `./watod down` from host |

`[WARNING] No textures found` — run asset sync on host, then re-eval.

## 3. Leader teleop + rich recording (needs USB leader)

Physical SO101 Leader drives sim; **S** start/stop, **R** reset world, **C** cancel episode while recording.

```bash
cd $TASK_ROOT
mkdir -p /workspace/humanoid/datasets/record_so101_gym/001

PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_agent.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR \
  --port /dev/ttyACM0 \
  --repo_root /workspace/humanoid/datasets/record_so101_gym/001 \
  --save_mp4 --depth --instance_id_seg
```

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

## Host install (legacy — not for IL docker workflow)

Only if developing outside Docker on a matching Isaac Lab 2.3.2 + Python 3.11 stack:

```bash
cd src/simulation/so101_vial_task
pip install -e .
```

Requires Isaac Lab, LeRobot, and synced USD/HDRI assets.
