# WATonomous Humanoid - UWaterloo's First Humanoid Robot 

Dockerized ROS 2 stack for controlling and interfacing with the humanoid robot, plus Isaac Lab / MuJoCo simulation, teleop, and imitation learning.

Published docs: https://watonomous.github.io/humanoid-docs/index.html

## Quick start (watod)

1. Copy config and pick a module:

```bash
cp watod-config.sh watod-config.local.sh
# edit ACTIVE_MODULES and MODE_OF_OPERATION="develop"
```

2. Build and run:

```bash
./watod build
./watod up -d
./watod -t <service>           # e.g. interfacing, mjlabs
./watod -t <service>_dev        # when the module defines a _dev service
```

Most modules expose `<name>_dev` under `MODE_OF_OPERATION=develop`. **`interfacing`** and **`mjlabs`** do not — shell into `interfacing` / `mjlabs` directly.

| `ACTIVE_MODULES` | What it runs |
|------------------|--------------|
| `interfacing` | CAN / hardware interfacing |
| `perception` | Perception (cameras, GPU) |
| `behaviour` | `joint_command`, `voxel_grid` |
| `simulation_isaac` | **Isaac Lab 2.3.2** — SO101 IL, HumanoidRL, Quest teleop |
| `simulation_mj` | MuJoCo / mjlab RL (`mjlabs` service) |

**Isaac Lab sim (recommended):** see [docker/simulation/isaac_lab/QUICKSTART.md](docker/simulation/isaac_lab/QUICKSTART.md).

## Repo map

```
humanoid
├── watod                     # Compose orchestrator
├── watod-config.sh           # Defaults (copy → watod-config.local.sh)
├── watod_scripts/            # Dev-env / Docker helpers
├── modules/                  # docker-compose.<module>.yaml
├── docker/                   # Dockerfiles per stack
│   ├── base/
│   ├── interfacing/
│   ├── perception/
│   ├── behaviour/
│   └── simulation/
│       ├── isaac_lab/        # Isaac Lab + LeRobot (primary sim)
│       └── mjlabs/           # MuJoCo / mjlab
├── src/
│   ├── wato_msgs/            # Shared messages (common_msgs)
│   ├── interfacing/          # CAN, DBC
│   ├── perception/
│   ├── behaviour/            # joint_command, voxel_grid
│   ├── simulation/           # Isaac tasks, teleop, HumanoidRL
│   ├── teleop/               # Quest WebXR → ROS 2 bridge
│   ├── il/                   # Imitation learning recording
│   └── embedded/             # STM32, ESP32S3 firmware
├── assets/lerobot/           # SO101 USD / vial-task assets
├── docs/                     # Doc conventions + architecture map
└── utils/                    # Package scaffolding helpers
```

## Simulation

| Stack | Module | Docs |
|-------|--------|------|
| Isaac Lab 2.3.2 / Sim 5.1 (SO101 IL, HumanoidRL, Quest) | `simulation_isaac` | [QUICKSTART](docker/simulation/isaac_lab/QUICKSTART.md) · [full README](docker/simulation/isaac_lab/README.md) |
| MuJoCo / mjlab | `simulation_mj` | [README](docker/simulation/mjlabs/README.md) |
| SO101 vial Gym envs | (inside `simulation_isaac`) | [so101_vial_task](src/simulation/so101_vial_task/README.md) |
| Quest bimanual teleop | (inside `simulation_isaac`) | [quest_isaac_teleop](src/teleop/quest_isaac_teleop/README.md) |
| Other teleop variants | host or container | [teleop/README.md](src/teleop/README.md) |

Isaac Lab needs Linux, NVIDIA GPU, Docker GPU passthrough, and X11 (`xhost +local:docker`).

## Development areas

| Area | Start here |
|------|------------|
| Imitation learning | [src/il/README.md](src/il/README.md) · Isaac [QUICKSTART](docker/simulation/isaac_lab/QUICKSTART.md) |
| CAN / hardware | [src/interfacing/can/README.md](src/interfacing/can/README.md) |
| Messages | [src/wato_msgs/common_msgs/README.md](src/wato_msgs/common_msgs/README.md) |
| New ROS package | [utils/README.md](utils/README.md) |
| Doc conventions | [docs/README.md](docs/README.md) |

## CAN / arm bring-up

Full checklist (power, CANable udev, calibrate, smoke test):

→ [src/interfacing/can/README.md](src/interfacing/can/README.md)

For the full **calibrate → visualize → move** sequence: [ARM_BRINGUP.md](ARM_BRINGUP.md)

Open arm work (sim mirror of calibrated joints in Isaac Lab, VR teleop, etc.) is listed there under **Open arm tasks**.

```bash
./src/interfacing/can/scripts/can_udev.sh install   # once per host → /dev/canable
```

## Requirements

- Ubuntu ≥ 22.04 (WSL / macOS may work for non-GPU stacks)
- Docker + watod (`./watod`)
- NVIDIA GPU + drivers for Isaac Lab, perception, and mjlab
