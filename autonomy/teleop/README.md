# Teleop

All teleoperation entry points live here (one home, not scattered across `autonomy/simulation/`).

## Quest 2 VR (bimanual)

| Package | Role |
|---------|------|
| [`quest_teleop/`](quest_teleop/) | WebXR page + WSS bridge → `/quest_teleop` (C++ ROS 2 pkg) |
| [`quest_isaac_teleop/`](quest_isaac_teleop/) | Isaac Sim runner: both arms, weighted-DLS fingertip IK |

- **Full setup (certs, adb, Isaac Sim):** [`quest_isaac_teleop/README.md`](quest_isaac_teleop/README.md)
- **Bridge-only (node + WebXR server):** [`quest_teleop/README.md`](quest_teleop/README.md)

## Other teleop & dataset collection

| Folder | Input device | Sim robot | IL recording |
|--------|--------------|-----------|--------------|
| [`keyboard_teleop/`](keyboard_teleop/) | Keyboard + IK | pioneer bimanual (right arm) | `--record` → `humanoid_il` |
| [`task_space_controller/`](task_space_controller/) | Viewport pose target + IK | pioneer bimanual (left arm) | `--publish-real-left-arm` → `/behaviour/arm_pose` |
| [`so101_leader_teleop/`](so101_leader_teleop/) | SO101 Leader (USB) or keyboard + IK | SO101 follower | `--record` → `humanoid_il` |
| [`camera_teleop/`](camera_teleop/) | Webcam hand landmarks | pioneer hand | not wired yet |
| `humanoid-record` (CLI) | ROS topics | real pioneer arm | `autonomy/il` |

Shared IK helpers + arm config: `autonomy/simulation/pioneer_humanoid`.
Shared recorder: `autonomy/il` — see [IL README](../il/README.md).

## Upper body control
Arm-hand (6 DOF arm + 15 DOF hand).

## Lower body control (not used currently, future reference)
4D tensor `[x, y, yaw, lower_body_height]`.

### Reference
IsaacLab: `IsaacLab-main/source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree`
