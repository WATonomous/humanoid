# CAN interfacing (`can` package)

ROS 2 bridge: `/interfacing/motorCMD` ↔ CAN ↔ `/interfacing/motorFeedback`.
[Electrical docs](https://watonomous.github.io/humanoid-docs/electrical/index.html) · [Interfacing docs](https://watonomous.github.io/humanoid-docs/interfacing/index.html)

## Arm bring-up

1. **Hardware**: battery + E-stop closed (motor power ≠ CAN power). CANable USB → host; CAN_H/CAN_L → arm (120 Ω term).
2. **Host setup (once)**: `./src/interfacing/can/scripts/can_udev.sh install` → `/dev/canable`.
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
| TODO (later) | **Isaac Lab sim-to-real** — `task_space_ik.py --publish-real-left-arm` (IK) + `reach` RL task driving the real arm | Validate IK/policy against real hardware |

---

## Topics / config

`/interfacing/motorCMD` (`MotorCmd`, ROS→CAN) · `/interfacing/motorFeedback` (`MotorFeedback`, CAN→ROS)

`config/params.yaml` defaults: `can_interface=can0` `device_path=/dev/canable` `bustype=slcan` `bitrate=1000000` `fd_enabled=false`

DBC: `src/interfacing/dbc/humanoid.dbc` · Debug: `candump can0`

### CAN-FD

Off by default (`fd_enabled: false`). To use it:

- **Adapter**: the CANable dongle running `slcand` is classic-CAN only — the Lawicel/slcan
  ASCII protocol has no FD framing or data-phase negotiation. FD requires swapping it for a
  native SocketCAN-FD-capable adapter (e.g. a gs_usb/candleLight-firmware device, or a
  PCAN-USB FD) and setting `bustype: "socketcan"` directly (not `"slcan"`).
- **Bring-up**: bring the interface up with the data-phase bitrate set *before* `can_node`
  starts, e.g.:
  ```bash
  ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
  ```
  `can_core` only enables `CAN_RAW_FD_FRAMES` on its socket (`fd_enabled: true` +
  `data_bitrate` in `params.yaml`) — it does not configure the adapter's data-phase rate
  itself.
- **Frame size stays 8 bytes**: the CubeMars AK10-9/AK80-9 MIT/servo protocol
  (`config/mit_profiles.yaml`, `humanoid.dbc`) is a fixed 8-byte-frame format defined by the
  motor firmware, which doesn't understand FD frames. Enabling FD here buys bus
  arbitration/throughput headroom (more motors polling at higher rates, lower latency) — it
  does not let individual motor messages grow past 8 bytes. If a future device on this bus
  (e.g. a custom FD-native motor controller) needs bigger payloads, that's a DBC/message
  redesign on top of this, not required by FD itself.
