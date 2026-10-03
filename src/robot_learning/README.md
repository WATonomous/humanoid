# Robot learning: record datasets, train policies

Teleoperate the Pioneer left arm, record the demos as a **LeRobot** dataset, train a policy (ACT, SmolVLA, pi0.5, …) on it. This package is the recording side: one schema and one recorder shared by every teleop script. RL is separate and lives in `src/simulation/humanoid_rl*`.

## 1. Collect demos (leader arm, sim)

Add `--record` to the leader-arm teleop, in Isaac or MuJoCo:

```bash
# simulation_isaac container
cd /workspace/humanoid/src/teleop/pioneer_leader_arm_teleop
/workspace/isaaclab/isaaclab.sh -p pioneer_leader_arm_teleop.py --scene vial_rack --record \
  --task_description "put the vial in the rack" --num_episodes 20

# simulation_mj container (CPU)
python3 pioneer_leader_arm_teleop.py --target mujoco --scene peg_insert --record
```

| Key | Effect |
|-----|--------|
| S | Start the take |
| N | Save the take, then the scene resets |
| D | Discard the take and redo it |
| R | Reset arm and scene (discards a take in progress) |

Other flags: `--cameras ego,wrist_left` / `none`, `--dataset_root`, `--schema`. Leader setup, calibration and scenes: [`src/teleop/pioneer_leader_arm_teleop/README.md`](../teleop/pioneer_leader_arm_teleop/README.md).

Output: `<repo>/datasets/pioneer_v1_left_arm/sim/`. Later sessions append to the same dataset.

Isaac keyboard teleop (`src/teleop/keyboard_teleop/keyboard_teleop.py`, task-space IK) takes the same `--record` flags and writes the same format.

## 2. Data contract

`config/dataset_schema_pioneer_v1.yaml`, shared by sim and real. **7 values**, 25 fps:

| # | Name | Unit |
|---|------|------|
| 1–3 | `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw` | rad |
| 4–5 | `left_elbow_pitch`, `left_elbow_roll` | rad |
| 6 | `left_wrist_pitch` | rad |
| 7 | `left_gripper` | 0 open … 1 closed |

| Field | Content |
|-------|---------|
| `observation.state` | measured joint positions + gripper closure |
| `action` | commanded joint targets + gripper command |
| `observation.images.<name>` | `ego` and `wrist_left` by default, 640×480 RGB (`wrist_right` off) |
| `task` | `--task_description` |

Camera poses and lenses: `pioneer_humanoid/arm_params.py`.

## 3. Train

In the `simulation_isaac` container (`train-policy` is `lerobot_train` at the pinned LeRobot commit):

```bash
train-policy \
  --dataset.repo_id=humanoid/pioneer_v1_left_arm \
  --dataset.root=/workspace/humanoid/datasets/pioneer_v1_left_arm/sim \
  --policy.type=act \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --output_dir=/workspace/humanoid/outputs/train/pioneer_act
```

Flag pitfalls (`--steps`, not epochs; always `--policy.push_to_hub=false` for local runs) are listed in the [SO101 README](../simulation/so101_vial_task/README.md), which uses the same command.

There is no sim rollout script for a trained Pioneer policy yet. `rtc_driver.py` is the Real-Time Chunking driver for flow-matching policies (pi0 / pi0.5 / SmolVLA) that such a script would use.

## 4. Real arm (planned)

Not runnable yet. The plan is the same leader arm and the same schema, saved under `datasets/pioneer_v1_left_arm/real/`:

- [#327](https://github.com/WATonomous/pioneer_humanoid/issues/327) — `pioneer_leader_arm_teleop.py --target real`: dry-run by default, `--live` to command the arm; adds the gripper command path.
- [#328](https://github.com/WATonomous/pioneer_humanoid/issues/328) — real demo recording (cameras, state from motor feedback). Still open there: a separate `humanoid-record` process, or `--target real --record`.

`humanoid-record` (ROS 2, `humanoid_robot_learning/record.py`) exists today but stops at startup until the gripper has a ROS source and each recorded camera has a `topic` in the schema. Its write path can be tested with no robot:

```bash
pip install -e "src/robot_learning[record]"
humanoid-record --dry_run --sink lerobot,hdf5 --num_episodes 2 --episode_time_s 3
```

`--sink` (`lerobot`, `hdf5`, or both) exists only on `humanoid-record` and the Quest teleop; the sim leader and keyboard teleop always write LeRobot.
