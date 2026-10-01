# Robot learning: record datasets, train and evaluate policies

Record teleoperation into **LeRobot** and/or **HDF5** datasets. One shared `RecordSession` for multiple collection paths:

- **Real WATO arm** — ROS 2 (`humanoid-record`)
- **Isaac Sim + keyboard** — WATO bimanual left arm IK teleop

Policies (ACT, SmolVLA, pi0.5, …) train on these datasets with LeRobot (`train-policy` in the Isaac image). RL is sim-based and lives in `src/simulation/humanoid_rl*`. HDF5 output is a single `trajectories.h5` with `action`, `proprio`, optional `pixels`, `ep_len`, and `ep_offset`.

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
src/robot_learning/
├── config/
│   └── dataset_schema_pioneer_v1.yaml  # Pioneer v1 left arm, shared by sim and real
├── humanoid_robot_learning/
│   ├── snapshot.py               # ObservationSnapshot
│   ├── frame.py                  # build_lerobot_frame()
│   ├── so101_sim.py              # SO101 leader ↔ sim joint mapping (so101_vial_task)
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
cd src/robot_learning
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
pip install -e ../../../robot_learning[record]

PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p keyboard_teleop.py --record \
  --sink lerobot,hdf5 \
  --num_episodes 5 \
  --task_description "reach and grasp"
```

Uses `config/dataset_schema_pioneer_v1.yaml`: 6 joints (rad) + gripper closure (0 open, 1 closed), 25 fps
(every 4th physics step). Cameras (640×480 RGB, defined in `src/teleop/teleop_cameras.py`): `ego` and
`wrist_left` by default, `wrist_right` off. Override with `--cameras ego`, `--cameras ego,wrist_left,wrist_right`
or `--cameras none`. Same S/N/D/Esc keys as real-arm recording.

Output: `datasets/pioneer_v1_left_arm/sim/001/`.

## Train (LeRobot)

**SO101 vial task:** inside `simulation_isaac` Docker — [`docker/simulation/isaac_lab/QUICKSTART.md`](../../docker/simulation/isaac_lab/QUICKSTART.md) (`train-policy`, `--policy.push_to_hub=false`, `--steps=...`).

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
