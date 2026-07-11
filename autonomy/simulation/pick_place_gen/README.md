# pick_place_gen — Scripted Pick-and-Place IL Data Generation

Autonomous demonstration generator for the **wato_bimanual_arm (left arm)**:
[NVIDIA cuRobo](https://github.com/NVlabs/curobo) plans collision-free,
minimum-jerk trajectories from privileged ground-truth object poses (no
teleop), an orchestrator executes pick→transit→place with per-step actuation
noise, and success-gated episodes are written as a **LeRobot dataset** with
two RGB cameras through `humanoid_il.SimLeRobotRecorder`.

Everything runs inside the **`simulation_il_dev`** container
(`ACTIVE_MODULES="simulation_il"`, `MODE_OF_OPERATION="develop"`; see
`docker/simulation/isaac_il/QUICKSTART.md`). Never run on the host.

```
task_params.py (YAML-configurable)  ──►  pick_place_bimanual Isaac Lab task
        │                                (Isaac-PickPlace-BimanualLeft-v0)
        ▼
generate_demos.py  ──►  orchestrator.py (phase machine, noise injection)
        │                       │
        │                       ▼
        │               curobo_expert.py (MotionPlanner: plan_grasp / plan_pose)
        ▼
humanoid_il SimLeRobotRecorder  ──►  LeRobot dataset (state 8, action 8,
                                     env_state 14, external+wrist 480×640 RGB)
```

## Quick start (inside the container)

```bash
cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL

# 1. (one-time / after URDF changes) rebuild the cuRobo robot model
cd ../../pick_place_gen && $PYTHON build_wato_robot_cfg.py && cd -

# 2. sanity: reachability grid (93.8% plan success over the default workspace)
cd ../../pick_place_gen && $PYTHON validate_curobo_plan.py \
    --box-center 0.355 -0.20 0.17 --box-extent 0.15 0.24 0.10 \
    --table-top 0.05 --num-yaws 8 && cd -

# 3. env smoke test
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p \
    HumanoidRLPackage/HumanoidRLSetup/tasks/pick_place_bimanual/scripts/smoke_hold_pose.py \
    --headless --enable_cameras

# 4. generate! (no recording: drop --record/--schema; add --save_debug_images DIR
#    to dump camera PNGs at phase transitions of episode 0)
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p ../../pick_place_gen/generate_demos.py \
    --headless --enable_cameras --num_episodes 75 --seed 42 \
    --record --schema /workspace/humanoid/autonomy/il/config/dataset_schema_pick_place_bimanual.yaml \
    --dataset_root /workspace/humanoid/datasets/record_pick_place_bimanual
```

Only ground-truth-successful episodes are saved (object within XY/Z tolerance
of the target, settled ≥ `success.settle_steps` control steps after release).
The driver prints attempted/succeeded/saved, a failure histogram, and wall
time.

## Configuring the task

Every knob lives in one dataclass tree — `task_params.py` — overridable from
YAML via `--task_params my_task.yaml`. YAML keys mirror the dataclass fields;
unknown keys raise immediately. All poses are in the **robot base frame**
(= env origin; table top at z = 0.05, validated workspace
x ∈ [0.28, 0.43], y ∈ [−0.32, −0.10] — see `wato_constants.py`).

| Group | Field | Default | Meaning |
|---|---|---|---|
| `object` | `size` | `[0.04, 0.04, 0.04]` | cuboid edge lengths [m] |
| | `mass` | `0.05` | [kg] |
| | `color` | `[0.8, 0.1, 0.1]` | RGB 0–1 |
| | `x_range` / `y_range` | workspace | spawn ranges [m] |
| | `yaw_range` | `[-π, π]` | spawn yaw |
| `place` | `mode` | `"table"` | `"table"`: random pose on table; `"stack"`: on top of `place_object` |
| | `x_range` / `y_range` | workspace | place-target sample range (table mode) |
| | `min_separation` | `0.12` | min XY distance object ↔ target at reset [m] |
| | `xy_tolerance` / `z_tolerance` | `0.03` / `0.02` | success tolerances [m] |
| | `stack_object_size/mass/color` | 5 cm blue | the base object (stack mode) |
| `motion` | `hover_offset` | `0.10` | pre-grasp hover above the grasp pose [m] |
| | `place_hover_offset` | `0.06` | hover above the place pose (kept low: near-ceiling poses plan poorly) |
| | `grasp_tip_depth` | `0.010` | fingertip-center below object top (clamped so pads clear the table) |
| | `lift_height` | `0.16` | post-grasp lift [m] |
| | `place_clearance` | `0.005` | drop gap above support [m] |
| | `yaw_candidates` | `8` | top-down grasp yaw goalset size |
| | `time_dilation` | `0.6` | slow planner trajectories (smaller = slower) |
| `noise` | `enabled` | `true` | master switch for all diversity/noise |
| | `joint_noise_std_deg` | `0.25` | per-step Gaussian noise on streamed arm targets — the BC recovery signal |
| | `via_point_prob/lateral/vertical` | `0.7 / 0.08 / 0.05` | random transit via-point (non-straight paths) |
| | `grasp_yaw_jitter_deg` | `8.0` | grasp yaw perturbation |
| | `friction_range` / `restitution_range` | `[0.6,1.2] / [0,0.1]` | per-episode object material randomization |
| `success` | `settle_steps` | `25` | consecutive steps the placed object must hold pose |
| | `max_lin_vel` | `0.02` | settle velocity threshold [m/s] |
| `cameras` | `enabled/external/wrist` | all `true` | needs `--enable_cameras` on the CLI too |
| | `width/height` | `640/480` | per camera |
| `episode` | `sim_dt/decimation` | `0.01 / 2` | physics 100 Hz, control 50 Hz |
| | `record_divisor` | `2` | record every Nth control step → **fps 25** |
| | `phase_timeout_s` | `8.0` | per-phase watchdog (streaming phases auto-extend) |
| | `*_effort_limit` | see below | actuator overrides (this task only) |

### Worked examples

Plain pick-and-place with a bigger box (`--task_params big_box.yaml`):

```yaml
object:
  size: [0.05, 0.05, 0.05]
  mass: 0.08
  color: [0.1, 0.7, 0.2]
```

Stacking (place the red cube on the blue one):

```yaml
place:
  mode: stack
```

Fast state-only generation (no cameras; also drop `--enable_cameras`):

```yaml
cameras:
  enabled: false
```

## Dataset schema

`autonomy/il/config/dataset_schema_pick_place_bimanual.yaml` — LeRobot
features per frame (fps 25):

- `action` (8): commanded joint targets `[joint1L, joint2l..joint6l, joint7l, joint8l]`
- `observation.state` (8): measured joint positions, same order
- `observation.environment_state` (14): object pose (pos+quat wxyz) + place-target pose, robot base frame — privileged state for debugging/state-based baselines
- `observation.images.external` / `observation.images.wrist`: 480×640 RGB video

Action = joint positions (cuRobo-native). The fingertip-EE pose can be
reconstructed from state via `bimanual_arm_cfg.compute_gripper_tip_pose_b`.

## Findings / flags for the team (do not skip)

1. **`GRIPPER_OPEN`/`GRIPPER_CLOSED` are inverted in
   `Teleop/keyboard-based teleoperation/bimanual_arm_cfg.py`.** Measured from
   the finger STLs: at `(0, 0)` the pads are ~9.5 cm apart (OPEN); at
   `(-0.05, +0.05)` the finger meshes cross and interpenetrate. The unused
   dicts in `quest_isaac_teleop/bimanual_pink_controller_cfg.py` had it right.
   This task uses the corrected convention (`wato_constants.py`); the shared
   teleop file was deliberately left untouched — teleop should fix + retest.
2. **The default pose parks the grippers crossed** (`joint7l=-0.05,
   joint8l=+0.05`), which with `enabled_self_collisions=True` generates
   permanent internal contact forces (measured 53 Nm at the shoulder in free
   space). This task overrides gripper defaults to `(0,0)` and disables PhysX
   self-collisions (path-level self-collision safety comes from cuRobo).
3. **Rated motor torques cannot hold the arm statically in sim.** With the
   config's rated limits, `joint2l` saturates and sags 0.21 rad (~9 cm at the
   fingertip). This task raises shoulder/elbow to the documented PEAK torques
   (53 / 22 Nm) and the wrist to 5 Nm — **the wrist exceeds the GL40 spec
   (0.73 Nm peak); the physical wrist likely cannot hold this gripper
   top-down either. Needs a mechanical review.**
4. The wrist camera is a "hand-eye" view mounted off the gripper rail; its
   framing is angled but consistent — cube + both fingers visible throughout.

## Host memory

An early 75-episode camera pilot froze a 30 GB host: RSS grew ~0.6 GB per
saved episode until the kernel hit a memory-pressure livelock. Root causes
(both fixed in `humanoid_il/sim_recorder.py`, validated flat over long runs):

1. **Per-episode GPU→CPU copies were never returned to the OS** under Isaac
   Sim's allocator. Episodes now travel through a fixed pool of two pinned
   CPU slots, allocated once and reused — constant memory, faster DMA, and
   at most two episodes in flight (`save_episode` blocks when the writer
   lags).
2. **lerobot's in-process PyAV/SVT-AV1 video encoding leaked native memory**
   inside Isaac Sim. Encoding now runs in a short-lived `ffmpeg` subprocess
   with identical codec parameters (`_install_subprocess_video_encoder`).

`generate_demos.py` prints a `[mem]` line per episode (`--debug_memory` adds
a live-buffer census). If RSS trends upward across episodes, treat it as a
regression in this fix.

If a hard crash interrupts recording, the dataset may hold one truncated
orphan parquet (the write in flight) and refuse to load (`Parquet magic
bytes not found in footer`). Repair: validate each `data/chunk-*/*.parquet`
with `pyarrow.parquet.read_metadata` and delete the corrupt one(s) —
committed episodes, videos, and metadata stay intact.

## Known limitations / next steps

- The grasped object is **removed from the cuRobo world model during
  transit** rather than attached to the gripper (v1 simplification);
  clearance comes from `lift_height`/`place_clearance`. cuRobo's
  `attachment_manager` is the upgrade path.
- Single env (`num_envs=1`); scaling path is batched cuRobo planning +
  per-env recorder buffers.
- `plan_pick_failed` episodes (~10%) are discarded up-front (no sim time
  wasted); tune `object.x_range/y_range` against `validate_curobo_plan.py`
  if the rate grows.

## Rebuilding the robot model

`build_wato_robot_cfg.py` (a) writes `bimanual_arm_curobo.urdf` with real
joint limits (the SolidWorks export has all limits `(0,0)`), (b) MorphIt-fits
collision spheres (`--protrusion-weight 100`; the default 10 over-inflates
thin links), (c) locks the right arm + grippers, adds the `attached_object`
frame, and auto-ignores sphere pairs that overlap at the default pose (the
physical robot holds that pose, so they are fit artifacts). Output:
`curobo_cfg/wato_bimanual_left.yml`.

**TODO — merge `bimanual_arm_curobo.urdf` back into `bimanual_arm.urdf`.**
The cuRobo URDF exists only because the shared `bimanual_arm.urdf` ships with
all joint limits `(0,0)`, which cuRobo reads directly and would clamp every
joint to zero. The two are otherwise the same robot and should be one file —
but `bimanual_arm.urdf` is also loaded by the quest teleop pipeline
(`run_quest_bimanual_teleop.py`), so patching its limits in place risks
changing that pipeline's IK. Merge once the real limits are confirmed safe for
teleop (or teleop is migrated to the patched file), then delete the copy.
