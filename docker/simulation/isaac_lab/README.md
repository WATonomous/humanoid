# Isaac Lab container (watod `simulation_isaac`)

Docker stack for Isaac Lab sim: Pioneer **RL tasks** (in-hand, locomotion, push), Quest teleop, and SO101 imitation learning.

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
| **`simulation_isaac`** (this) | RL tasks + Quest teleop + SO101 IL |
| **`simulation_mj`** | MuJoCo / mjlab |

## Files

| Path | Role |
|------|------|
| `docker/simulation/isaac_lab/isaac_lab.Dockerfile` | Image build |
| `modules/docker-compose.simulation_isaac.yaml` | watod compose service |
| `src/simulation/humanoid_rl/` | RL runners: train / play (`$RL_RUNNERS`) |
| `src/simulation/humanoid_rl_tasks/` | RL tasks — in-hand, locomotion, push_block |
| `src/simulation/so101_vial_task/` | SO101 Gym envs + `lerobot_agent.py` / `lerobot_eval.py` |
| `assets/lerobot/` | SO101 workshop USD/HDRI assets |

## One-time host setup

```bash
cd ~/Desktop/humanoid

xhost +local:docker
mkdir -p ~/docker/isaac-sim/{cache/kit,cache/ov,cache/pip,cache/glcache,cache/computecache,logs,data}
```

`watod-config.local.sh` (create in repo root, do not commit):

```bash
ACTIVE_MODULES="simulation_isaac"
MODE_OF_OPERATION="develop"
```

GUI uses **X11 to host display** (same as [workshop `teleop-docker`](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop)). Not WebRTC.

## Build and launch

```bash
./watod build simulation_isaac_dev          # first time: large NGC pull
./watod up -d
./watod -t simulation_isaac_dev             # bash inside container
```

Rebuild after Dockerfile changes or broken torch:

```bash
./watod build --no-cache simulation_isaac_dev
./watod down && ./watod up -d
```

## Inside container — environment

Set automatically in `.bashrc`:

```bash
export ISAACLAB=/workspace/isaaclab
export HUMANOID_ROOT=/workspace/humanoid
export TASK_ROOT=/workspace/humanoid/src/simulation/so101_vial_task
export RL_RUNNERS=/workspace/humanoid/src/simulation/humanoid_rl/humanoid_rl/scripts
export PYTHON=/workspace/isaaclab/_isaac_sim/python.sh
```

Aliases: `rl-train`, `rl-play`, `train-policy`, `record-demos`, `eval-policy`.

Open the plain Isaac Sim GUI (no Python task). Container is root, so set:

```bash
OMNI_KIT_ALLOW_ROOT=1 $ISAACLAB/isaaclab.sh -s
```

Sanity:

```bash
$PYTHON -c "import torch; print(torch.__version__)"
$PYTHON -c "import lerobot; print('ok')"
```

## Workflows

### A. RL tasks — RSL-RL train / play

```bash
cd $HUMANOID_ROOT

# In-hand cube reorientation
rl-train --task=Isaac-Repose-Cube-PioneerHand-v0 --headless
rl-play --task=Isaac-Repose-Cube-PioneerHand-Play-v0 --num_envs=1

# Locomotion — Pioneer humanoid V1 (flat)
rl-train --task=Isaac-Locomotion-Flat-PioneerHumanoid-v0 --headless
rl-play --task=Isaac-Locomotion-Flat-PioneerHumanoid-Play-v0 --num_envs=1
```

Checkpoints: `outputs/rl/<experiment>/` (same path on host under `~/Desktop/humanoid/...`).

Tasks and runners: [`src/simulation/README.md`](../../../src/simulation/README.md).

### B. Quest teleop

See [`src/teleop/quest_teleop/README.md`](../../../src/teleop/quest_teleop/README.md).

### C. SO101 imitation learning — ACT train, sim eval, record demos, GR00T eval

Commands, pitfalls and asset sync: [`src/simulation/so101_vial_task/README.md`](../../../src/simulation/so101_vial_task/README.md).
