# Imitation learning: record (`humanoid-record`)

Record teleoperation into **LeRobot** and/or **HDF5** datasets. One shared `RecordSession` for multiple collection paths:

- **Real WATO arm** — ROS 2 (`humanoid-record`)
- **Isaac Sim + keyboard** — WATO bimanual left arm IK teleop
- **Isaac Sim + SO101 Leader** — physical leader drives SO101 follower (sim-to-real)

Training stays outside this repo (`lerobot-train` on the LeRobot folder). HDF5 output is a single `trajectories.h5` with `action`, `proprio`, optional `pixels`, `ep_len`, and `ep_offset`.

## Data contract

**6-DOF left arm** — same joint order as `joint_command_core.cpp`:

1. shoulder pitch, roll, yaw  
2. elbow pitch, roll  
3. wrist pitch  

| Field | Source | Units |
|-------|--------|-------|
| `observation.state` / `proprio` | measured joint positions | rad |
| `action` | commanded joint targets (IK output on sim, `/behaviour/arm_pose` on robot) | rad |
| `observation.images.*` / `pixels` | cameras in schema (optional for sim) | uint8 |
| `task` | `--task_description` | string |

## Layout

```
autonomy/il/
├── config/
│   ├── dataset_schema.yaml       # real robot (ROS + wrist camera)
│   ├── dataset_schema_sim.yaml   # WATO keyboard sim (joint-only)
│   └── dataset_schema_so101_sim.yaml  # SO101 leader sim (6-DOF + gripper)
├── humanoid_il/
│   ├── snapshot.py               # ObservationSnapshot
│   ├── frame.py                  # build_lerobot_frame()
│   ├── so101_sim.py              # SO101 leader ↔ sim joint mapping
│   ├── recorder.py               # RecordSession (episode flags + sinks)
│   ├── record_loop.py            # blocking loop for ROS CLI
│   ├── sim_session.py            # helper for Isaac sim scripts
│   ├── sinks/
│   │   ├── lerobot.py            # Parquet + MP4
│   │   └── hdf5.py               # trajectories.h5
│   ├── schema.py
│   ├── record.py                 # humanoid-record CLI
│   └── ros_buffer.py
└── README.md
```

## Install

```bash
cd autonomy/il
pip install -e ".[record]"          # real arm + sim recording
pip install -e ".[record,ros]"      # + ROS image decoding
```

## Real robot (ROS 2)

```bash
source /path/to/humanoid/install/setup.bash

humanoid-record \
  --task_description "reach forward" \
  --num_episodes 10 \
  --sink lerobot
```

Both formats:

```bash
humanoid-record --sink lerobot,hdf5 --num_episodes 10
```

**Keyboard** (needs `pynput`):

| Key | Effect |
|-----|--------|
| S | Start logging frames for this episode |
| N | Finish episode → `save_episode` |
| D | Discard buffer, re-record same episode |
| Esc | Abort and `finalize` |

**Dry run** (no robot):

```bash
humanoid-record --dry_run --sink lerobot,hdf5 --num_episodes 2 --episode_time_s 3
```

Output: `datasets/record/001/` with LeRobot tree + `trajectories.h5`.

## Isaac Sim (keyboard teleop)

From `autonomy/simulation/Teleop/keyboard_based_teleoperation/`:

```bash
pip install -e ../../../il[record]

PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p keyboard_teleop.py --record \
  --sink lerobot,hdf5 \
  --num_episodes 5 \
  --task_description "reach and grasp"
```

Uses `config/dataset_schema_sim.yaml` (joint-only, no camera). Same S/N/D/Esc keys as real-arm recording.

Output: `datasets/record_sim/001/`.

## Isaac Sim (SO101 teleop)

<<<<<<< HEAD
**Keyboard** or **physical SO101 Leader** drives the SO101 follower in sim. From `autonomy/simulation/Teleop/so101_leader_teleoperation/`:
=======
**Keyboard** or **physical SO101 Leader** drives the SO101 follower in sim. From `autonomy/simulation/Teleop/so101-leader teleoperation/`:
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

```bash
pip install -e ../../../il[record]

# Keyboard (no USB arm)
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_keyboard_teleop.py --record \
  --sink lerobot,hdf5 --num_episodes 5 --task_description "so101 keyboard demo"

# Physical leader
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_leader_teleop.py --record \
  --sink lerobot,hdf5 --num_episodes 5 --port /dev/ttyACM0
```

Uses `config/dataset_schema_so101_sim.yaml` (6 joints including gripper, leader raw units). Same S/N/D/Esc keys.

Output: `datasets/record_so101_sim/001/`.

**Vision + domain randomization** (NVIDIA workshop parity):

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_leader_teleop.py --record \
  --cameras --domain_rand \
  --schema ../../../il/config/dataset_schema_so101_sim_vision.yaml \
  --sink lerobot,hdf5 --num_episodes 10 --port /dev/ttyACM0
```

- `humanoid_il/so101_cameras.py` — read TiledCamera RGB into LeRobot frames
- `humanoid_il/so101_domain_rand.py` — vial/rack reset, mat, lighting, camera DR

Output: `datasets/record_so101_sim_vision/001/`.

<<<<<<< HEAD
## Train (LeRobot)

**SO101 vial sim IL (recommended):** inside `simulation_isaac` Docker — [`docker/simulation/isaac_lab/QUICKSTART.md`](../../docker/simulation/isaac_lab/QUICKSTART.md) (`il-train`, `--policy.push_to_hub=false`, `--steps=...`).

**Generic / host** (outside Isaac docker):
=======
## Train (LeRobot, external)
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)

```bash
lerobot-train \
  --dataset.repo_id=humanoid/local_left_arm \
  --dataset.root=datasets/record/001 \
  --policy.type=act \
  --output_dir=outputs/train/humanoid_act_v1
```

## Sinks

| `--sink` | Output | Use case |
|----------|--------|----------|
| `lerobot` | `data/`, `videos/`, `meta/` | `lerobot-train`, HuggingFace Hub |
| `hdf5` | `trajectories.h5` | custom HDF5 loaders, offline analysis |
| `lerobot,hdf5` | both under same `001/` folder | sim validation + BC training |
