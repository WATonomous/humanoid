## Teleoperation & dataset collection

| Folder | Input device | Sim robot | IL recording |
|--------|--------------|-----------|--------------|
| `keyboard-based teleoperation/` | Keyboard + IK | WATO bimanual (left arm) | `--record` → `humanoid_il` |
| `so101-leader teleoperation/` | SO101 Leader (USB) or keyboard + IK | SO101 follower | `--record` → `humanoid_il` |
| `camera-based teleoperation/` | Webcam hand landmarks | WATO hand | not wired yet |
| `humanoid-record` (CLI) | ROS topics | real WATO arm | `autonomy/il` |

Shared recorder: `autonomy/il` — see [IL README](../../il/README.md).

## Upper body control
Control for Arm-hand (6 DOF Arm + 15 DOF Hand)

## Lower body control (Not used currently, for future reference)
- 4D Tensor [x, y, yaw, lower_body_height]

### Reference:
IsaacLab repo:
IsaacLab-main/source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree
