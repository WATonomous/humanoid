# `task_space_ik.py`

Task-space (Cartesian) IK for the Pioneer bimanual arm's **left arm**. A cube in the
viewport is the absolute gripper-tip pose target — drag/rotate it and the arm follows via
Differential IK (DLS).

```bash
# sim only (default)
./run.sh   # or: <isaaclab.sh> -p task_space_ik.py
```

## Sim-to-real (`--publish-real-left-arm`)

Adds a direct rclpy publisher on `/behaviour/arm_pose`:

```
task_space_ik.py --publish-real-left-arm
  -> /behaviour/arm_pose
  -> joint_command_node   (seed-from-feedback + velocity/delta/low-pass; autonomy/behaviour/joint_command)
  -> /interfacing/motorCMD
  -> can_node
  -> AK motors (0x0A-0x0E, POSITION_LOOP)
```

`joint_command_node` seeds its rate-limiter from live motor feedback on the first
`ArmPose`, so even the first command ramps from the real arm's actual pose — no snap. The
script still waits `_REAL_ARM_PUBLISH_START_DELAY_S` (3 s) before publishing so an operator
can get a hand on the e-stop.

**KNOWN GAP:** the published values are raw sim joint angles. `joint_command_node` treats
them as calibrated cmd-frame degrees (0 = `calibrate_arm.py`'s home), while sim `qpos=0` is
an unrelated USD reference. Nothing converts between the two, so real motion won't track
the sim/cube target until a sim↔real calibration exists.

### Bring-up

Prereqs: ROS 2 workspace built (`cd autonomy && colcon build`), CANable on `/dev/canable`,
e-stop + arm power on.

```bash
# Terminal 1 — CAN
ros2 launch can can.launch.py

# Terminal 2 — joint_command (the safety node)
ros2 launch joint_command joint_command.launch.py

# Terminal 3 — the sim, in the simulation_isaac container (has ROS 2 built for Isaac's Python)
<isaaclab.sh> -p autonomy/teleop/task_space/task_space_ik.py \
  --publish-real-left-arm
```

(No UDP bridge — the old `--udp` path via `udp_to_ros_bridge.py` is gone; the sim publishes
`/behaviour/arm_pose` directly.)

**Have a human on the e-stop for the first real run.**
