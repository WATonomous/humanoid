# AI for Industry Challenge — sim assets

Task-board and connector sim assets from the Intrinsic "AI for Industry
Challenge" (SFP/SC/NIC connector insertion task), copied here for reuse with
our own humanoid arm.

- Source repo: https://github.com/AMMistry18/aic (fork/derivative of
  Intrinsic + Open Robotics' `intrinsic-dev/aic` toolkit)
- Source commit: `fd27bbe875dbd656e381f72ae2b44efdfb243924`
- License: Apache-2.0 (see `LICENSE` in the source repo; `aic_assets`
  declares `Apache-2.0` explicitly in its `package.xml`). This directory
  keeps that license — attribution/notice requirements apply if
  redistributed further.
- Write-up describing the original perception/insertion pipeline:
  https://satyaa27.github.io/intrinsic-ai-for-industry-challenge-team-tar-2-sol/

## What's here

- `gazebo_models/` — Gazebo/SDF models (`model.sdf` + `.glb` visual mesh per
  part): task board, NIC card, SFP module, SC plug/port, LC plug/mount,
  cables, the Robotiq Hand-E gripper, Axia80 F/T sensor, Basler camera,
  mounts, and enclosure/walls/floor.
- `mujoco_scene/mjcf/` — the same scene exported to MuJoCo MJCF
  (`scene.xml` includes `aic_world.xml` + `aic_robot.xml`), with per-part
  `.obj`/`.stl` meshes and textures. This includes the original UR5e arm +
  Robotiq gripper meshes from the challenge robot.
- `mujoco_scene/scripts/` — `view_scene.py` (quick MuJoCo viewer) and
  `add_cable_plugin.py` (the post-processing script that split/tuned the
  raw `sdf2mjcf` output into `aic_robot.xml`/`aic_world.xml`/`scene.xml`
  and added actuators, sensors, and cable physics — kept for reference if
  the scene needs regenerating from a fresh SDF export).

## Using this with our own arm

The task-relevant geometry (task board, NIC card, SFP/SC/LC connectors and
ports, cables) is arm-agnostic — it's what a gripper needs to interact with,
not the UR5e itself. To drive this with our humanoid arm instead of the
challenge's UR5e + Robotiq gripper:

1. Keep the environment bodies from `mujoco_scene/mjcf/aic_world.xml`
   (task board, ports, cables) or the equivalent Gazebo models in
   `gazebo_models/`.
2. Swap the robot body (`aic_robot.xml`, or the UR5e/Hand-E SDF models) for
   our arm's own URDF/MJCF description.
3. Re-attach sensors/sites (`gripper_tcp`, FT sensor site) to the
   corresponding link on our arm so downstream perception/control code
   (see the write-up above) can target the same frames.

Full conversion tooling (Gazebo→MJCF pipeline, ROS 2 launch files) lives
upstream in `AMMistry18/aic` if the scene needs to be regenerated from
scratch; only the generated scene + a couple of convenience scripts were
copied here.
