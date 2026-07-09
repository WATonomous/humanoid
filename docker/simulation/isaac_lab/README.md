# SO101 sim IL (watod `simulation_il`)

Workshop-parity Docker stack for SO101 **imitation learning** in sim: teleop, record, ACT/GR00T eval.

**Quick copy-paste commands:** [QUICKSTART.md](QUICKSTART.md)

| Component | Version |
|-----------|---------|
| Isaac Sim | 5.1 (via `nvcr.io/nvidia/isaac-lab:2.3.2`) |
| Isaac Lab | 2.3.2 |
| Python | 3.11 |
| PyTorch | 2.7.0 |
| LeRobot | 0.4.3 @ `e670ac5daf9b76` |

Based on [NVIDIA SO-101 workshop](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop) `teleop-docker` LeRobot install pattern (`--no-deps` + pip constraints).

## vs other environments

| Environment | Use |
|-------------|-----|
| **`simulation_il`** (this) | SO101 IL + **HumanoidRL** (RSL-RL train/play) |
| **`simulation`** | Humanoid_Wato / quest teleop (Sim 4.5) |

## Files

| Path | Role |
|------|------|
| `docker/simulation/isaac_lab/isaac_lab.Dockerfile` | Image build |
| `modules/docker-compose.simulation_il.yaml` | watod compose service |
| `autonomy/simulation/so101_vial_task/` | Gym envs + `lerobot_agent.py` / `lerobot_eval.py` |
| `assets/lerobot/` | Workshop USD/HDRI assets |

## One-time host setup

```bash
cd ~/Desktop/humanoid

xhost +local:docker
./assets/lerobot/sync_so101_vial_assets.sh --full
mkdir -p ~/docker/isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data}
```

`watod-config.local.sh` (create in repo root, do not commit):

```bash
ACTIVE_MODULES="simulation_il"
MODE_OF_OPERATION="develop"
```

GUI uses **X11 to host display** (same as [workshop `teleop-docker`](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop)). Not WebRTC.

## Build and launch

```bash
./watod build simulation_il_dev          # first time: large NGC pull
./watod up -d
./watod -t simulation_il_dev             # bash inside container
```

Rebuild after Dockerfile changes or broken torch:

```bash
./watod build --no-cache simulation_il_dev
./watod down && ./watod up -d
```

## Inside container — environment

Set automatically in `.bashrc`:

```bash
export ISAACLAB=/workspace/isaaclab
export HUMANOID_ROOT=/workspace/humanoid
export TASK_ROOT=/workspace/humanoid/autonomy/simulation/so101_vial_task
export RL_ROOT=/workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
export PYTHON=/workspace/isaaclab/_isaac_sim/python.sh
```

Aliases: `il-train`, `il-record`, `il-eval`, `rl-train`, `rl-play`.

Sanity:

```bash
$PYTHON -c "import torch; print(torch.__version__)"
$PYTHON -c "import lerobot; print('ok')"
```

## Workflows

### A. HumanoidRL — RSL-RL train / play

```bash
cd $RL_ROOT
rl-train --task=Isaac-Repose-Cube-WatoHand-v0 --headless
rl-play --task=Isaac-Repose-Cube-WatoHand-Play-v0 --num_envs=1
```

Checkpoints: `logs/rsl_rl/<experiment>/` (same path on host under `~/Desktop/humanoid/...`).

### B. Lazy IL — HF dataset → ACT train → sim eval (no arm)

See [QUICKSTART.md](QUICKSTART.md) §4–5.

1. `il-train` with `--policy.type=act`, `--policy.push_to_hub=false`, `--steps=N`
2. `cd $TASK_ROOT` then `lerobot_eval.py` with `--policy_type lerobot`
3. **No `--rename_map`** for local ACT

### C. Record demos (USB leader)

```bash
cd $TASK_ROOT
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_agent.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR \
  --port /dev/ttyACM0 \
  --repo_root /workspace/humanoid/datasets/record_so101_gym/001 \
  --save_mp4 --depth --instance_id_seg
```

Keys: **S** start/stop episode, **R** reset, **C** cancel while recording.

### D. GR00T eval (remote server)

```bash
cd $TASK_ROOT
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py \
  --task Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval \
  --policy_type groot \
  --policy_host localhost \
  --policy_port 5555 \
  --rename_map '{"external_D455": "front", "ego": "wrist"}'
```

`--rename_map` is for GR00T / mismatched camera names, not local ACT.

## Training notes (LeRobot 0.4.3)

| Topic | Detail |
|-------|--------|
| Entry point | `il-train` → `$PYTHON -m lerobot.scripts.lerobot_train` |
| Not | bare `lerobot-train`, not `lerobot.scripts.train` |
| Duration flag | `--steps` (default 100k), not `--training.num_epochs` |
| Hub upload | `--policy.push_to_hub=false` for local checkpoints |
| Dataset example | `CursedRock17/so101_teleop_vials_sim_and_real` — 140 episodes |
| 10k steps | ~3 epochs over 140 demos (smoke test) |
| 50k–100k steps | more typical for usable ACT |

## Eval notes

| Topic | Detail |
|-------|--------|
| Working directory | must be `$TASK_ROOT` |
| ACT checkpoint | `--policy_type lerobot --policy_path .../pretrained_model` |
| `rename_map` | omit for ACT on matching `ego` / `external_D455` cameras |
| Success | printed at end; env terminates on rack placement |
| Hang on exit | `./watod down` from host — Ctrl+C often fails after Isaac crash |

## Stop

```bash
exit
./watod down
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No module named torch` / packaging error | rebuild image `--no-cache` (see Dockerfile humanoid-il fix) |
| `KeyError: 'ego'` on eval | remove `--rename_map` |
| `can't open file .../scripts/lerobot_eval.py` | `cd $TASK_ROOT` |
| `policy.repo_id missing` on train | add `--policy.push_to_hub=false` |
| `No textures found` | `./assets/lerobot/sync_so101_vial_assets.sh --full` on host |
| Isaac frozen | `./watod down` from host |

## Upgrade image

```bash
./watod build --no-cache simulation_il_dev
./watod down && ./watod up -d
```

Repo code is bind-mounted at `/workspace/humanoid` — Python edits on host apply without rebuild.
