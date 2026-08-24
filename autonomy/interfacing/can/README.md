# CAN interfacing (`can` package)

ROS 2 bridge: `/interfacing/motorCMD` ↔ CAN ↔ `/interfacing/motorFeedback`.
[Electrical docs](https://watonomous.github.io/humanoid-docs/electrical/index.html) · [Interfacing docs](https://watonomous.github.io/humanoid-docs/interfacing/index.html)

## Arm bring-up

1. **Hardware**: battery + E-stop closed (motor power ≠ CAN power). CANable USB → host; CAN_H/CAN_L → arm (120 Ω term).
2. **Host setup (once)**: `./autonomy/interfacing/can/scripts/can_udev.sh install` → `/dev/canable`.
   `watod-config.local.sh`: `ACTIVE_MODULES="interfacing"`, `MODE_OF_OPERATION="develop"`.
3. **Bring up**:
   ```bash
   ./watod build && ./watod up -d
   ./watod -t interfacing
   source /opt/watonomous/setup.bash
   ```
   `can.launch.py` starts `can_node` + SLCAN (`/dev/canable` → `can0` @ 1 Mbps).

### Verify
```bash
ros2 node list                  # /can_node
candump can0                    # e.g. 0x290A–0x290E
ros2 topic echo /interfacing/motorFeedback common_msgs/msg/MotorFeedback --once
```

### Calibrate (`calibrate_arm.py`)
Per joint: confirm motor id → home zero → one end Enter → other end Enter → writes `zero_offset`/limits/`can_id`.
```bash
source /opt/watonomous/setup.bash
python3 /root/ament_ws/src/interfacing/can/scripts/calibrate_arm.py \
  --arm-side left --write-mapping --mapping /calibration/hardware_mapping.yaml
```
Prompt: **Enter**=yes · id=correct id · **s**=skip · **q**=quit.

---

## Open arm tasks (onboarding / assignable)

Live joint mirror, mjlab sim parity, and interactive calibration are done — see
[ARM_BRINGUP.md](../../../ARM_BRINGUP.md) for calibrate → visualize → move.

| Status | Task | Why |
|--------|------|-----|
| TODO | **VR teleop** — Quest → real motors via teleop + `joint_command` / CAN | End-to-end teleop UX |
| TODO (later) | **Isaac Lab sim-to-real** — `task_space_real.py` (IK) + `reach` RL task driving the real arm | Validate IK/policy against real hardware |

---

## Topics / config

`/interfacing/motorCMD` (`MotorCmd`, ROS→CAN) · `/interfacing/motorFeedback` (`MotorFeedback`, CAN→ROS)

`config/params.yaml` defaults: `can_interface=can0` `device_path=/dev/canable` `bustype=slcan` `bitrate=1000000`

DBC: `autonomy/interfacing/dbc/humanoid.dbc` · Debug: `candump can0`

## CAN-FD (opt-in, unvalidated on real hardware)

`enable_can_fd` / `data_bitrate` in `params.yaml` exist as scaffolding for a future move to
CAN-FD (higher data-phase bitrate, more bus headroom at high motor counts/control rates).
Defaults are `enable_can_fd: false`, which preserves today's classic-CAN/SLCAN behavior
exactly.

**Do not flip this on against the real arm without bench-testing first.** Enabling it
requires:
- A CAN-FD-capable adapter (the CANable + SLCAN setup this repo documents cannot carry FD
  frames — SLCAN is the Lawicel ASCII protocol, which has no FD framing). You need a native
  SocketCAN-FD interface (`bustype: "socketcan"`), e.g. via `gs_usb`/candleLight firmware.
- Re-validating CAN watchdog/timeout behavior on that new adapter before trusting it near
  motors — see the `real-hardware-safety` skill.

`can_node` will refuse to start (not silently fall back to classic CAN) if `enable_can_fd:
true` is set together with `bustype: "slcan"`, since that combination can't work.
