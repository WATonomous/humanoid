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
│   ├── pioneer_humanoid/    # THE robot definition — arm/hand/whole-body articulation, joint limits, IK, cameras (imported everywhere)
│   ├── simulation/          # Isaac Lab sim & learning — see src/simulation/README.md
│   │   ├── humanoid_rl/         #   RL runners: train / play / distill / diagnose  ($RL_RUNNERS)
│   │   ├── humanoid_rl_tasks/   #   RL tasks, flat — inhand, locomotion, push_block, pick_place
│   │   ├── humanoid_scenes/     #   teleop data-collection scenes (@scene-discovered) — bare, vial_rack, push_block
│   │   └── so101_vial_task/     #   SO101 imitation-learning task
│   ├── teleop/              # Drive the arm (sim or real): keyboard, Quest WebXR, task-space IK — resolve --scene via humanoid_scenes
│   ├── il/                  # Imitation-learning dataset recording (LeRobot)
│   └── embedded/            # STM32 / ESP32S3 motor-controller firmware
├── assets/                  # robot URDF/USD/meshes + scene props (backend-neutral, not tied to Isaac)
│   ├── pioneer_bimanual_arm/  pioneer_hand/  whole_body_humanoid/
│   ├── props/                  #   block.usd, box.usd, table.usd
│   └── lerobot/                #   SO101 arm + vial-task USDs + HDRIs (external-synced)
├── outputs/                 # training runs / checkpoints (gitignored) — rl/ , train/
└── docs/                    # Pointer to the humanoid-docs site
```

**RL task vs teleop scene** — the split a contributor needs first:

| adding… | goes in | how it registers |
|---|---|---|
| an RL task (has a reward, gets PPO-trained) | `src/simulation/humanoid_rl_tasks/<task>/` | auto (`import_packages`) |
| a teleop-only scene (collect demos, no reward) | `src/simulation/humanoid_scenes/<name>/scene.py` + `@scene("<name>")` | auto (discovery) |
| a scene that is **both** (e.g. `push_block`) | task owns the geometry in its `scene.py`; add a 1-line shim in `humanoid_scenes/` | both |

Full detail — [src/simulation/README.md](src/simulation/README.md). Other areas: each has its own `README.md`.

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
