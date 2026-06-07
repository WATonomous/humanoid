# Imitation learning: record (`humanoid-record`)

Record teleoperation into **LeRobot** and/or **HDF5** datasets. One shared frame contract for the **real arm (ROS 2)** and **Isaac Sim keyboard teleop**.

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
│   └── dataset_schema_sim.yaml   # Isaac sim (joint-only, no camera)
├── humanoid_il/
│   ├── snapshot.py               # ObservationSnapshot
│   ├── frame.py                  # build_lerobot_frame()
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

From `autonomy/simulation/Teleop/keyboard-based teleoperation/`:

```bash
pip install -e ../../../il[record]

PYTHONPATH=$(pwd) <path>/IsaacLab/isaaclab.sh -p keyboard_teleop.py --record \
  --sink lerobot,hdf5 \
  --num_episodes 5 \
  --task_description "reach and grasp"
```

Uses `config/dataset_schema_sim.yaml` (joint-only, no camera). Same S/N/D/Esc keys as real-arm recording.

Output: `datasets/record_sim/001/`.

## Train (LeRobot, external)

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
