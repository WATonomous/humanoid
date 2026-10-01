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
| `action` | commanded joint targets (IK output on sim, `/arm/joint_targets` on robot) | rad |
| `observation.images.*` / `pixels` | cameras in schema (optional for sim) | uint8 |
| `task` | `--task_description` | string |

## Layout

```
src/il/
├── config/
│   ├── dataset_schema_pioneer_v1.yaml  # Pioneer v1 left arm, shared by sim and real
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
cd src/il
pip install -e ".[record]"          # real arm + sim recording
pip install -e ".[record,ros]"      # + ROS image decoding
```

## Real robot (ROS 2)

Uses `config/dataset_schema_pioneer_v1.yaml`, the same contract as sim. Not runnable yet: it stops at
startup until the gripper (`left_gripper`) has a ROS source and each recorded camera has a `topic`.
Cameras capture 640×480 (D455: `rgb_camera.color_profile: 640x480x30`); frames are resized to the
schema size (center-cropped first if the aspect differs).

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

Output: `datasets/pioneer_v1_left_arm/real/001/` with LeRobot tree + `trajectories.h5`.

## Isaac Sim (keyboard teleop)

From `src/teleop/keyboard_teleop/`:

```bash
pip install -e ../../../il[record]

PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p keyboard_teleop.py --record \
  --sink lerobot,hdf5 \
  --num_episodes 5 \
  --task_description "reach and grasp"
```

Uses `config/dataset_schema_pioneer_v1.yaml`: 6 joints (rad) + gripper closure (0 open, 1 closed), 25 fps
(every 4th physics step). Cameras (320×240 RGB, 4:3, defined in `src/teleop/teleop_cameras.py`): `ego` and
`wrist_left` by default, `wrist_right` off. Override with `--cameras ego`, `--cameras ego,wrist_left,wrist_right`
or `--cameras none`. Same S/N/D/Esc keys as real-arm recording.

Output: `datasets/pioneer_v1_left_arm/sim/001/`.

## Isaac Sim (SO101 teleop)

**Keyboard** or **physical SO101 Leader** drives the SO101 follower in sim. From `src/teleop/so101_leader_teleop/`:

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

## Train (LeRobot)

**SO101 vial sim IL (recommended):** inside `simulation_isaac` Docker — [`docker/simulation/isaac_lab/QUICKSTART.md`](../../docker/simulation/isaac_lab/QUICKSTART.md) (`il-train`, `--policy.push_to_hub=false`, `--steps=...`).

**Generic / host** (outside Isaac docker):

```bash
lerobot-train \
  --dataset.repo_id=humanoid/local_left_arm \
  --dataset.root=datasets/pioneer_v1_left_arm/real/001 \
  --policy.type=act \
  --output_dir=outputs/train/humanoid_act_v1
```

## Sinks

| `--sink` | Output | Use case |
|----------|--------|----------|
| `lerobot` | `data/`, `videos/`, `meta/` | `lerobot-train`, HuggingFace Hub |
| `hdf5` | `trajectories.h5` | custom HDF5 loaders, offline analysis |
| `lerobot,hdf5` | both under same `001/` folder | sim validation + BC training |
