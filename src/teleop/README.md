# Teleop

All teleoperation entry points, one home. **Everything runs inside the
`simulation_isaac` watod container** (Isaac Lab 2.3.2) — do not use host Isaac Lab.

```bash
# host: ACTIVE_MODULES="simulation_isaac" in watod-config.local.sh
./watod up -d
./watod -t simulation_isaac          # shell into the container
```

The repo is bind-mounted at `/workspace/humanoid`. Shared arm config + IK helpers:
`src/pioneer_humanoid`. Shared recorder: `src/il`.

| Folder | Input | Sim robot | Notes |
|--------|-------|-----------|-------|
| [`quest_isaac_teleop/`](quest_isaac_teleop/) | Quest 2 hand tracking | pioneer bimanual, both arms | weighted-DLS fingertip IK; see its README (certs, adb, setup) |
| [`quest_teleop/`](quest_teleop/) | — | — | WebXR page + WSS bridge → `/quest_teleop` (C++ ROS 2 pkg); see its README |
| **`keyboard_teleop/`** | keyboard + IK | pioneer bimanual, left arm | see below |
| [`task_space_controller/`](task_space_controller/) | viewport pose gizmo + IK | pioneer bimanual, left arm | `--publish-real-left-arm` drives the real arm — **its README covers the CAN pipeline + e-stop** |
| [`so101_leader_teleop/`](so101_leader_teleop/) | SO101 Leader (USB) or keyboard | SO101 follower | see its README (vial scene, DR, cameras) |
| `humanoid-record` (CLI) | ROS topics | real pioneer arm | `src/il` |

## keyboard_teleop

Left arm follows the keyboard via differential IK (absolute-pose target, DLS).

```bash
# inside the simulation_isaac container:
cd /workspace/humanoid/src/teleop/keyboard_teleop
PYTHONPATH=$(pwd) /workspace/isaaclab/isaaclab.sh -p keyboard_teleop.py [--scene bare|push] [--record]
```

- **Move:** `W/S` x · `A/D` y · `Q/E` z · `Z/X` `T/G` `C/V` rotate · **hold `Shift`** = fine
- **`K`** toggle gripper · **`R`** reset arm
- **`--scene`:** `bare` (default) or `push` (table + ramp-box + block + lightbox)
- **`--record`** (image already has `humanoid-il`): `S` start · `N` save · `D` discard · `Esc` stop → `datasets/record_sim/`

## Notes

- Upper body = 6-DOF arm + 15-DOF hand. Lower body (unused): 4D `[x, y, yaw, height]`.
- Retargeter reference: `IsaacLab/.../devices/openxr/retargeters/humanoid/unitree`
