# Imitation learning: Phase 1 record (`humanoid-record`)

Record teleoperation on the **real arm** into a [LeRobot](https://github.com/huggingface/lerobot) dataset. Training stays **outside** this repo (`lerobot-train` on the exported folder). Pattern follows `lehome-challenge-main/scripts/utils/dataset_record.py`; execution uses ROS 2 instead of Isaac Sim.

## Data contract

**6-DOF left arm** — same joint order as `joint_command_core.cpp`:

1. shoulder pitch, roll, yaw  
2. elbow pitch, roll  
3. wrist pitch  

| Field | Source | Units in dataset |
|-------|--------|------------------|
| `observation.state` | `ros.state_topic`, or `action_topic` if state unset | rad |
| `action` | `ros.action_topic` (`/behaviour/arm_pose`) | rad |
| `observation.images.*` | camera topics in `config/dataset_schema.yaml` | uint8 video |
| `task` | `--task_description` | string |

ROS `ArmPose` uses **degrees** (see `joint_command`); the recorder converts to radians before `add_frame`.

**Defaults:** `fps=30`, `repo_id=humanoid/local_left_arm`, output `datasets/record/001`, …

## Prerequisites

1. Built workspace with `common_msgs` (`colcon build`, `source install/setup.bash`).
2. Python env with LeRobot:  
   `pip install -e "autonomy/il[lerobot,keyboard,ros]"`
3. Stack running (example):  
   `ACTIVE_MODULES="perception interfacing behaviour teleop"` → `./watod up`
4. Teleop publishing `ArmPose` on `/behaviour/arm_pose`.

Optional: `cv_bridge` from apt inside the ROS image (`ros-$ROS_DISTRO-cv-bridge`).

## Record

```bash
cd autonomy/il
pip install -e ".[lerobot,keyboard,ros]"

source /path/to/humanoid/install/setup.bash

humanoid-record \
  --task_description "reach forward" \
  --num_episodes 10
```

**Keyboard** (lehome-style, needs `pynput`):

| Key | Effect |
|-----|--------|
| S | Start logging frames for this episode |
| N | Finish episode → `save_episode` |
| D | Discard buffer, re-record same episode |
| Esc | Abort and `finalize` |

Timed episodes (no keyboard): `--episode_time_s 60`

**Test write path without robot:**

```bash
# Disable cameras in config or use dry_run (synthetic joints + blank images)
humanoid-record --dry_run --num_episodes 2 --episode_time_s 3
```

## Configuration

Edit `config/dataset_schema.yaml`:

- `images.*.topic` — match RealSense remaps when the asset is wired.
- `ros.state_topic` — set when motor feedback is published; until then `state` mirrors `action`.
- `record.root` — base directory for new runs (`001`, `002`, …).

## Train (external)

```bash
lerobot-train \
  --dataset.repo_id=humanoid/local_left_arm \
  --dataset.root=datasets/record/001 \
  --policy.type=act \
  --output_dir=outputs/train/humanoid_act_v1
```

Feature names and shapes must match `meta/info.json` from the recorded run.

## Layout

```
autonomy/il/
├── config/dataset_schema.yaml
├── humanoid_il/
│   ├── schema.py          # LeRobotDataset.create features
│   ├── arm_pose_io.py     # ArmPose → (6,) rad
│   ├── ros_buffer.py      # ROS subscribers
│   ├── record.py          # CLI
│   └── episode_keys.py    # S / N / D / Esc
└── README.md
```

## LeHome reference map

| This repo | LeHome |
|-----------|--------|
| `schema.py` | `dataset_record.py` (features + `create`) |
| `record.py` | `dataset_sim.py` + record loop |
| `episode_keys.py` | `register_teleop_callbacks` |
| `record_utils.py` | `lehome/utils/record.py` |

Phase 2 (offline inspect/augment) and Phase 3 (policy deploy) are not in this PR.

## Known limits (v1)

- No motor-feedback topic yet → `observation.state` may equal commanded `action`.
- Hand DOFs not in the dataset.
- Sim / garment record deferred to a later PR.
