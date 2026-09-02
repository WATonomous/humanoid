# Isaac-PickPlace-BimanualLeft-v0

Generalized pick-and-place with the wato_bimanual_arm's LEFT arm, built for
scripted IL data generation (cuRobo expert). The right arm holds its default
pose (no action term).

| | |
|---|---|
| Robot | `BIMANUAL_ARM_CFG` (via `robot_cfg_shim`) with task-local overrides: grippers default OPEN `(0,0)`, PhysX self-collisions off, peak-torque effort limits |
| Actions | `JointPositionActionCfg`, 8 left joints `[joint1L, joint2l..joint6l, joint7l, joint8l]` |
| Observations | left joint pos/vel, object pose in root frame, last action (dict obs) |
| Objects | configurable cuboid (+ optional stack base object) |
| Events | joint min-separation object reset, friction/restitution randomization |
| Terminations | time_out, object below table |
| Cameras | optional TiledCameras `external` + `wrist` (480×640 RGB) |
| Sim | dt 0.01, decimation 2 (50 Hz control) |

Everything is parameterized by `pick_place_gen/task_params.py`
(`PickPlaceTaskParams`); build the env with `make_env_cfg(params)` and pass
via `gym.make(..., cfg=env_cfg)`. Full docs + data generation:
`src/simulation/pick_place_gen/README.md`.

Smoke test (inside simulation_il container, from $HUMANOID_ROOT):

```bash
PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p \
    src/simulation/humanoid_rl_tasks/humanoid_rl_tasks/pick_place/scripts/smoke_hold_pose.py \
    --headless --enable_cameras
```
