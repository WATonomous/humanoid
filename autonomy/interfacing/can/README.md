# CAN interfacing (`can` package)

ROS 2 bridge: `/interfacing/motorCMD` ↔ CAN ↔ `/interfacing/motorFeedback`.

Wiring / power: [Electrical docs](https://watonomous.github.io/humanoid-docs/electrical/index.html) · Architecture: [Interfacing docs](https://watonomous.github.io/humanoid-docs/interfacing/index.html)

## Arm bring-up

### Hardware
1. Battery + E-stop (closed = powered). Motor power is separate from CAN.
2. CANable USB → host; CAN_H/CAN_L → arm (120 Ω termination).

### Host (once per machine)

```bash
./autonomy/interfacing/can/scripts/can_udev.sh install   # → /dev/canable
```

`watod-config.local.sh`:

```bash
ACTIVE_MODULES="interfacing"
MODE_OF_OPERATION="develop"
```

```bash
./watod build && ./watod up -d
./watod -t interfacing
source /opt/watonomous/setup.bash
```

`can.launch.py` brings up `can_node` + SLCAN (`/dev/canable` → `can0` @ 1 Mbps). The compose file sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` so shells can discover `can_node`.

### Verify

```bash
ros2 node list                  # /can_node
candump can0                    # e.g. 0x290A–0x290E
ros2 topic echo /interfacing/motorFeedback common_msgs/msg/MotorFeedback --once
# filter one motor:  ... | grep -A6 "motor_id: 14"
```

### Smoke test (optional)

Arm clear, hand on E-stop. Command **near** last feedback — do not jump to `0` blind.

```bash
ros2 topic pub --once /interfacing/motorCMD common_msgs/msg/MotorCmd \
  "{motor_id: 14, control_type: 4, position: 649.0}"
```

`control_type`: `4` position (deg), `5` set origin, `8` disable.

### Calibrate (`calibrate_arm.py`)

Per joint: confirm motor id → home zero → one end Enter → other end Enter. Writes `zero_offset`, limits, and remapped `can_id` with `--write-mapping`.

The script source is bind-mounted; run it from the mount so you get the latest prompts without rebuilding:

```bash
source /opt/watonomous/setup.bash
python3 /root/ament_ws/src/interfacing/can/scripts/calibrate_arm.py \
  --arm-side left --write-mapping --mapping /calibration/hardware_mapping.yaml
# or after rebuild:  ros2 run can calibrate_arm.py --arm-side left --write-mapping
```

Prompt per joint: **Is motor id N?** → **Enter** = yes · type `14` / `0x0E` = correct id · **s** = skip · **q** = quit.

---

## Open arm tasks (onboarding / assignable)

Work that should land **before** heavy IK / teleop so new people do not fight broken joint frames:

| Status | Task | Why |
|--------|------|-----|
| **TODO** | **Light joint mirror (preferred for calib verify)** — publish `sensor_msgs/JointState` from `/interfacing/motorFeedback` + `hardware_mapping.yaml`, show the arm URDF in **RViz2** (or Foxglove). Optional: overlay commanded vs measured | Fast check that zeros / signs / `can_id`s match reality; no GPU sim required. URDFs already exist under `Humanoid_Wato/arm_assembly/` |
| **TODO** (later) | **Full sim parity** — same live joint stream into Isaac Lab / mjlab when validating teleop, IK, or policies | Heavier; use after the RViz check is green |
| **TODO** | **VR teleop of the physical arm** — Quest (or similar) → real motors via teleop + `joint_command` / CAN | End-to-end teleop UX; assignable to one member |
| In progress | Interactive calibration (`calibrate_arm.py`) + correct `can_id` / `zero_offset` in YAML | Foundation for the tasks above |

Suggested acceptance (light path): move one real joint → same joint moves the same way in RViz; home pose in real ≈ home in the URDF.

---

## Topics / config

| Topic | Direction |
|-------|-----------|
| `/interfacing/motorCMD` (`MotorCmd`) | ROS → CAN |
| `/interfacing/motorFeedback` (`MotorFeedback`) | CAN → ROS |

| Param (`config/params.yaml`) | Default |
|------------------------------|---------|
| `can_interface` / `device_path` / `bustype` / `bitrate` | `can0` / `/dev/canable` / `slcan` / `1000000` |

DBC: `autonomy/interfacing/dbc/humanoid.dbc`. Debug: `candump can0`.
