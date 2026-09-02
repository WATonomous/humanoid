# Pioneer - UWaterloo's First Humanoid Robot 

Published docs: https://watonomous.github.io/humanoid-docs/index.html

## Quick start (watod)

```bash
cp watod-config.sh watod-config.local.sh   # set ACTIVE_MODULES
./watod up -d
./watod -t <service>                        # shell into a module
```

One editable container per module — code is bind-mounted from `src/<module>`, so edits are live. Full setup and dev workflow: **[DEVELOPING.md](DEVELOPING.md)**.

| `ACTIVE_MODULES` | What it runs |
|------------------|--------------|
| `interfacing` | CAN / hardware interfacing, `joint_command` |
| `perception` | Perception (cameras, GPU), `voxel_grid` |
| `simulation_isaac` | **Isaac Lab 2.3.2** — SO101 IL, RL tasks, Quest teleop |
| `simulation_mj` | MuJoCo / mjlab RL |

**Isaac Lab sim (recommended):** see [docker/simulation/isaac_lab/QUICKSTART.md](docker/simulation/isaac_lab/QUICKSTART.md).

## Repo map

Two workflows share this repo:

- **Real robot:** `embedded` firmware ⇄ `interfacing` (CAN ⇄ ROS 2) ⇄ `perception` — driven live by `teleop`, or by a policy.
- **Sim / learning:** `teleop` collects demos in `simulation` scenes → `il` records datasets → train (imitation) or `simulation/humanoid_rl` (RL) → deploy back through `interfacing`.

```
humanoid
├── watod                    # Compose orchestrator (one editable container per module)
├── watod-config.sh          # Module defaults — copy → watod-config.local.sh
├── watod_scripts/           # Dev-env / Docker helpers
├── modules/                 # docker-compose.<module>.yaml (one per ACTIVE_MODULE)
├── docker/                  # Dockerfiles per stack (interfacing, perception, simulation/{isaac_lab,mjlabs})
├── src/
│   ├── common_msgs/         # Shared ROS 2 message definitions
│   ├── interfacing/         # CAN ⇄ ROS 2 bridge, DBC, joint_command (ArmPose → per-motor CAN); real-arm bring-up
│   ├── perception/          # Perception nodes + voxel_grid (depth → occupancy grid)
│   ├── pioneer_humanoid/    # THE arm/robot definition — articulation, joint limits, IK helpers, cameras (imported everywhere)
│   ├── simulation/          # Isaac Lab: RL tasks, teleop scenes, training runners, datagen — see src/simulation/README.md
│   ├── teleop/              # Drive the arm (sim or real): keyboard, Quest WebXR, task-space IK
│   ├── il/                  # Imitation-learning dataset recording (LeRobot)
│   └── embedded/            # STM32 / ESP32S3 motor-controller firmware
├── assets/lerobot/          # SO101 USD + vial-task assets
└── docs/                    # Pointer to the humanoid-docs site
```

Deeper structure and "where does new work go" live in each area's own `README.md`
(notably [src/simulation/README.md](src/simulation/README.md) for the RL-task vs
teleop-scene split).

## Simulation

| Stack | Module | Docs |
|-------|--------|------|
| Isaac Lab 2.3.2 / Sim 5.1 (SO101 IL, RL tasks, Quest) | `simulation_isaac` | [QUICKSTART](docker/simulation/isaac_lab/QUICKSTART.md) · [full README](docker/simulation/isaac_lab/README.md) |
| MuJoCo / mjlab | `simulation_mj` | [README](docker/simulation/mjlabs/README.md) |
| SO101 vial Gym envs | (inside `simulation_isaac`) | [so101_vial_task](src/simulation/so101_vial_task/README.md) |
| Quest bimanual teleop | (inside `simulation_isaac`) | [quest_isaac_teleop](src/teleop/quest_isaac_teleop/README.md) |
| Other teleop variants | host or container | [teleop/README.md](src/teleop/README.md) |

Isaac Lab needs Linux, NVIDIA GPU, Docker GPU passthrough, and X11 (`xhost +local:docker`).

## Development

Setup, per-module dev loop, CI/linting, and per-area guides: **[DEVELOPING.md](DEVELOPING.md)**.

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
