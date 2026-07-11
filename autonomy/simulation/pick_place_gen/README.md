# pick_place_gen — Scripted Pick-and-Place IL Data Generation

Autonomous demonstration generator for the **wato_bimanual_arm (left arm)**.
[NVIDIA cuRobo](https://github.com/NVlabs/curobo) plans collision-free,
minimum-jerk trajectories from privileged ground-truth object poses (no teleop);
an orchestrator runs pick → transit → place with per-step actuation noise; and
success-gated episodes are written as a **LeRobot dataset** (two RGB cameras)
through `humanoid_il.SimLeRobotRecorder`. Supports placing the cube at a random
table spot, on a second cube (`stack`), or **inside a tray** (`tray`).

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

---

## Executive summary — every moving part

Paths are relative to the repo root (`~/Documents/Wato/humanoid`). Everything in
`pick_place_gen/` is listed by filename.

### Container (Docker)
| Path | What it is |
|---|---|
| `docker/simulation/isaac_lab_il_datagen/isaac_lab_il_datagen.Dockerfile` | The runtime image: a thin **fork of the shared `isaac_lab` container that adds cuRobo** (CUDA headers + NVRTC). Keeps cuRobo out of the shared container. |
| `modules/docker-compose.isaac_lab_il_datagen.yaml` | watod module that builds/runs the container above (GPU, X11, repo bind-mount). |

### Core pipeline (`autonomy/simulation/pick_place_gen/`)
| Path | What it does |
|---|---|
| `generate_demos.py` | **Main driver.** Builds the env, loops episodes, reads ground-truth poses, runs the orchestrator, gates success, and (optionally) records the dataset. |
| `orchestrator.py` | **Phase machine.** Turns ground-truth state into an 8-dim joint action each step (approach → grasp → lift → transit → place → release), injects noise, models tray walls as obstacles. |
| `curobo_expert.py` | **cuRobo wrapper.** Owns the `MotionPlanner`; `plan_pick`/`plan_move` produce joint trajectories resampled to the control rate; maintains the collision world. |
| `task_params.py` | **The config surface.** One dataclass tree (`object`/`place`/`motion`/`noise`/`success`/`cameras`/`episode`) loaded from a `--task_params` YAML. |
| `wato_constants.py` | Isaac-free constants (joint limits, gripper geometry, workspace box, tray geometry, asset paths) shared by the cuRobo-side scripts. |

### Isaac Lab task (`.../HumanoidRL/HumanoidRLPackage/HumanoidRLSetup/tasks/pick_place_bimanual/`)
| Path | What it does |
|---|---|
| `pick_place_env_cfg.py` | **The env cfg.** Scene (blue arm, red cube, black table, white tray), actions, observations, reset events, actuator/effort overrides. `apply_task_params()` projects a `PickPlaceTaskParams` onto it. |
| `robot_cfg_shim.py` | Import bridge to the shared `bimanual_arm_cfg.py` (which lives outside this package). Only file that touches those paths. |
| `mdp/events.py` | `reset_objects_min_separation` — randomizes object poses with pairwise separation and a **tray keep-out** (cube spawns around the tray, never on it). |
| `mdp/observations.py` | Observation terms (object pose in robot frame). |
| `config/HumanoidRLEnv/__init__.py` | `gym.register("Isaac-PickPlace-BimanualLeft-v0", ...)` — auto-discovered. |
| `scripts/smoke_hold_pose.py` | Fast sanity check: builds the env, holds the default pose, asserts no NaNs / no drift / cameras render. |

### Robot model + validation (`pick_place_gen/`)
| Path | What it does |
|---|---|
| `build_wato_robot_cfg.py` | Regenerates the cuRobo robot config: patches URDF joint limits, MorphIt-fits collision spheres, locks the right arm. Output → `curobo_cfg/wato_bimanual_left.yml`. |
| `validate_curobo_plan.py` | Standalone reachability grid — reports cuRobo plan success rate over a workspace box (~93.8% baseline). Use it to size spawn ranges. |
| `curobo_cfg/wato_bimanual_left.yml` | **Generated** cuRobo robot config (kinematics + spheres). Runtime artifact; committed. |
| `.../wato_bimanual_arm/urdf/bimanual_arm_curobo.urdf` | **Generated** URDF with real joint limits that cuRobo reads at runtime (see the last TODO). |

### Recorder & dataset (`autonomy/il/`)
| Path | What it does |
|---|---|
| `humanoid_il/sim_recorder.py` | `SimLeRobotRecorder` — GPU-buffered, leak-free LeRobot writer (pinned CPU slot pool + ffmpeg-subprocess video encode). |
| `config/dataset_schema_pick_place_bimanual.yaml` | Dataset feature schema (joint names, image entries, env-state names) — passed via `--schema`. |

### Task configs (`pick_place_gen/params/`)
| Path | What it does |
|---|---|
| `params/tray_demo.yaml` | **Cube-into-tray demo** (blue arm, red cube, black table, white tray), cameras **off** — for watching. |
| `params/tray_pilot.yaml` | Same tray task with cameras **on** — for recording a dataset. |
| `params/stack.yaml` | Stacking mode (place the cube on a second cube). |
| `params/fast_no_cameras.yaml` | Plain table pick-and-place, cameras off (fast state-only generation). |

---

## Quick start

### 1 · Bring up the container (on the HOST, from the repo root)
```bash
# a) point watod at this container — edit watod-config.local.sh to contain:
#      ACTIVE_MODULES="isaac_lab_il_datagen"
#      MODE_OF_OPERATION="develop"

# b) build + start it (first build takes a few min). Needs the isaac_lab base
#    image; if you don't have it locally, docker will pull/build it first.
./watod up --build -d

# c) allow the container to open GUI windows on your display
xhost +local:root
```

### 2 · Get a shell inside the container
```bash
./watod exec isaac_lab_il_datagen_dev bash
# fallback if that doesn't pass through:
docker exec -it $(docker ps -qf name=isaac_lab_il_datagen_dev) bash
```

### 3 · Run it (inside the container)
```bash
cd /workspace/humanoid/autonomy/simulation/Humanoid_Wato/HumanoidRL
GEN=../../pick_place_gen

# A) WATCH IT — GUI window, 3 tray episodes, no recording
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $GEN/generate_demos.py \
    --num_episodes 3 --seed 0 --task_params $GEN/params/tray_demo.yaml

# B) HEADLESS TEST — no GUI, 15 episodes, prints the success rate (no recording)
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $GEN/generate_demos.py \
    --headless --num_episodes 15 --seed 0 --task_params $GEN/params/tray_demo.yaml

# C) RECORDING PILOT — headless + cameras, saves a LeRobot dataset
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $GEN/generate_demos.py \
    --headless --enable_cameras --num_episodes 25 --seed 0 --record \
    --schema /workspace/humanoid/autonomy/il/config/dataset_schema_pick_place_bimanual.yaml \
    --dataset_root /workspace/humanoid/datasets/record_pick_place_tray \
    --task_params $GEN/params/tray_pilot.yaml
```
Only ground-truth-successful episodes are saved; the driver prints
attempted/succeeded/saved, a failure histogram, and wall time. Same `--seed`
gives an identical run — **vary the seed** to get diverse data.

### CLI flags (dictionary)
| Flag | Default | What it does |
|---|---|---|
| `--num_episodes N` | `10` | How many episodes to attempt (successes are what get saved). |
| `--seed S` | `0` | RNG seed. Same seed ⇒ identical run; change it for different episodes. |
| `--task_params FILE.yaml` | none | Task config override (object, place mode, tray, noise, cameras…). |
| `--record` | off | Write a LeRobot dataset (requires `--schema`). |
| `--schema FILE.yaml` | none | Dataset feature schema (required with `--record`). |
| `--dataset_root DIR` | schema default | Where the dataset is written. |
| `--task_description "…"` | `"pick and place the cube"` | Natural-language task label stored in the dataset. |
| `--save_debug_images DIR` | none | Dump camera PNGs at phase transitions of episode 0 (debug framing). |
| `--debug_memory` | off | Per-episode census of large live buffers (memory-leak triage). |
| `--headless` | off | Run **without** the Isaac Sim GUI window (faster; required on servers). |
| `--enable_cameras` | off | Render the RGB cameras (**required** to record video). |

`--headless` and `--enable_cameras` are Isaac Sim launcher flags. Recording
needs **both** `--enable_cameras` on the CLI **and** `cameras.enabled: true` in
the params (see `tray_pilot.yaml`).

---

## Configuring the task

Every knob lives in `task_params.py`, overridable from YAML via `--task_params`.
YAML keys mirror the dataclass fields; unknown keys raise immediately. All poses
are in the **robot base frame** (= env origin; table top z = 0.05, validated
workspace x ∈ [0.28, 0.43], y ∈ [−0.32, −0.10] — see `wato_constants.py`).

| Group | Field | Default | Meaning |
|---|---|---|---|
| `object` | `size` | `[0.04,0.04,0.04]` | cuboid edge lengths [m] |
| | `mass` / `color` | `0.05` / red | [kg] / RGB 0–1 |
| | `x_range` / `y_range` / `yaw_range` | workspace / `[-π,π]` | spawn ranges |
| `place` | `mode` | `"table"` | `"table"` random table pose · `"stack"` on `place_object` · `"tray"` centre of a fixed tray |
| | `x_range` / `y_range` | workspace | place-target sample range (table mode) |
| | `min_separation` | `0.12` | min XY distance object ↔ target at reset [m] |
| | `xy_tolerance` / `z_tolerance` | `0.03` / `0.02` | success tolerances [m] |
| | `stack_object_size/mass/color` | 5 cm blue | base object (stack mode) |
| | `tray_scale` | `0.7` | tray footprint scale (`tray.usda` is 0.20 × 0.16 m) |
| | `tray_height_scale` | `1.0` | extra scale on wall **height** only; `<1` = lower walls so the gripper clears the rim |
| | `tray_center` | `[0.33,-0.16]` | tray interior centre, env frame [m] |
| `motion` | `hover_offset` / `place_hover_offset` | `0.10` / `0.06` | pre-grasp / pre-place hover [m] |
| | `grasp_tip_depth` / `lift_height` / `place_clearance` | `0.010` / `0.16` / `0.005` | grasp depth / lift / drop gap [m] |
| | `yaw_candidates` / `time_dilation` | `8` / `0.6` | grasp yaw goalset / trajectory slow-down |
| `noise` | `enabled` | `true` | master switch for all diversity/noise |
| | `hover_jitter` | `0.02` | SD of the off-nominal approach-hover detour (recovery signal) [m] |
| | `joint_noise_std_deg` | `0.25` | per-step Gaussian noise on streamed arm targets [deg] |
| | `via_point_prob/lateral/vertical` | `0.7 / 0.08 / 0.05` | random transit via-point |
| | `grasp_yaw_jitter_deg` | `8.0` | grasp yaw perturbation |
| | `friction_range` / `restitution_range` | `[0.6,1.2]` / `[0,0.1]` | per-episode object material randomization |
| `success` | `settle_steps` / `max_lin_vel` | `25` / `0.02` | consecutive settled steps / velocity threshold |
| `cameras` | `enabled/external/wrist` | all `true` | needs `--enable_cameras` on the CLI too |
| | `width/height` | `640/480` | per camera |
| `episode` | `sim_dt/decimation` | `0.01 / 2` | physics 100 Hz, control 50 Hz |
| | `record_divisor` | `2` | record every Nth control step → **fps 25** |
| | `phase_timeout_s` | `8.0` | per-phase watchdog |
| | `*_effort_limit` | see file | actuator overrides (this task only) |

### Worked examples

Place the cube **inside a tray** (the readable demo — see `params/tray_demo.yaml`):
```yaml
object: { color: [1.0, 0.0, 0.0], x_range: [0.27, 0.45], y_range: [-0.40, -0.09] }
place:  { mode: tray, tray_scale: 0.7, tray_height_scale: 0.4, tray_center: [0.33, -0.26] }
```

Stacking (place the cube on the blue base cube):
```yaml
place: { mode: stack }
```

Fast state-only generation (also drop `--enable_cameras`):
```yaml
cameras: { enabled: false }
```

## Dataset schema

`autonomy/il/config/dataset_schema_pick_place_bimanual.yaml` — LeRobot features
per frame (fps 25):

- `action` (8): commanded joint targets `[joint1L, joint2l..joint6l, joint7l, joint8l]`
- `observation.state` (8): measured joint positions, same order
- `observation.environment_state` (14): object pose (pos+quat wxyz) + place-target pose, robot base frame
- `observation.images.external` / `observation.images.wrist`: 480×640 RGB video

Action = joint positions (cuRobo-native). The fingertip-EE pose can be
reconstructed from state via `bimanual_arm_cfg.compute_gripper_tip_pose_b`.

## Findings / flags for the team (do not skip)

1. **`GRIPPER_OPEN`/`GRIPPER_CLOSED` are inverted** in the shared
   `keyboard_based_teleoperation/bimanual_arm_cfg.py`. Measured from the finger
   STLs: at `(0,0)` the pads are ~9.5 cm apart (OPEN); at `(-0.05,+0.05)` the
   meshes cross. This task uses the corrected convention (`wato_constants.py`);
   the shared teleop file was left untouched — teleop should fix + retest.
2. **The default pose parks the grippers crossed**, which with
   `enabled_self_collisions=True` generates permanent internal contact forces
   (~53 Nm at the shoulder). This task overrides gripper defaults to `(0,0)` and
   disables PhysX self-collisions (path-level safety comes from cuRobo).
3. **Rated motor torques cannot hold the arm statically in sim** — `joint2l`
   sags 0.21 rad. This task raises shoulder/elbow to documented PEAK torques
   (53 / 22 Nm) and the wrist to 5 Nm — **the wrist exceeds the GL40 spec
   (0.73 Nm); needs a mechanical review.**
4. The wrist camera is a hand-eye view off the gripper rail; framing is angled
   but consistent (cube + both fingers visible throughout).

## Host memory & file ownership

An early camera pilot froze a 30 GB host: RSS grew ~0.6 GB per saved episode.
Root causes (both fixed in `humanoid_il/sim_recorder.py`, validated flat):
per-episode GPU→CPU copies never returned to the OS (now a fixed pool of two
pinned CPU slots), and lerobot's in-process video encode leaked (now a
short-lived `ffmpeg` subprocess). `generate_demos.py` prints a `[mem]` line per
episode as a regression canary; `--debug_memory` adds a live-buffer census.

**File ownership:** the container runs as root, so recorded files would land
root-owned. `generate_demos.py` auto-chowns the finished dataset to the
bind-mount owner (your host user) so it's deletable without `sudo`. Datasets
live under `datasets/` (gitignored) — share them via HuggingFace Hub or team
storage, **not** git.

If a hard crash interrupts recording, the dataset may hold one truncated orphan
parquet and refuse to load. Repair: validate each `data/chunk-*/*.parquet` with
`pyarrow.parquet.read_metadata` and delete the corrupt one(s).

## Known limitations / next steps

- The grasped object is **removed from the cuRobo world model during transit**
  (v1 simplification); clearance comes from `lift_height`/`place_clearance`.
- Single env (`num_envs=1`); scaling path is batched cuRobo planning.
- `plan_pick_failed` episodes are discarded up-front (no sim time wasted); tune
  `object.x_range/y_range` against `validate_curobo_plan.py` if the rate grows.

## Rebuilding the robot model

`build_wato_robot_cfg.py` (a) writes `bimanual_arm_curobo.urdf` with real joint
limits (the SolidWorks export has all limits `(0,0)`), (b) MorphIt-fits
collision spheres (`--protrusion-weight 100`), (c) locks the right arm +
grippers and adds the `attached_object` frame. Output →
`curobo_cfg/wato_bimanual_left.yml`.

---

## TODO / future fixes

1. **Stop the arm self-clipping — tighten the cuRobo joint limits.** On some
   episodes the arm passes through itself (PhysX self-collision is disabled by
   design, and cuRobo's model currently allows the offending poses). Narrow the
   joint limits (and/or self-collision spheres) in the cuRobo robot config so
   the planner never selects self-colliding configurations.

2. **Dataset file permissions — make the fix permanent.** New runs are
   auto-chowned to the host user (`generate_demos._fix_output_ownership`), but
   the container still runs as root, so any *other* root-written files (and
   pre-existing datasets) remain undeletable without `sudo`. Permanent fix: run
   the container as the host UID (`--user $(id -u):$(id -g)` or a userns remap)
   so nothing is ever root-owned.

3. **Avoid needless 180° gripper flips.** The gripper is symmetric, so the
   end-effector has **two valid orientations** (normal and mirrored) for any
   grasp — either is fine. The planner currently sometimes flips a full 180°
   when the un-flipped orientation would have worked, adding wasted motion. Add
   the mirrored yaw as an equivalent grasp candidate and pick whichever is
   closest to the current wrist yaw, so the arm only flips when it actually has
   to. (Pure optimization — reduces path length and awkward motions.)

4. **Merge `bimanual_arm_curobo.urdf` back into `bimanual_arm.urdf`.** The
   cuRobo URDF exists only because the shared `bimanual_arm.urdf` ships with all
   joint limits `(0,0)`, which cuRobo reads directly. The two are otherwise the
   same robot — but `bimanual_arm.urdf` is also loaded by the quest teleop
   pipeline, so patching its limits in place risks changing that pipeline's IK.
   Merge once the real limits are confirmed safe for teleop, then delete the copy.
