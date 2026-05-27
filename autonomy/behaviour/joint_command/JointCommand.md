# Joint command: `joint_command_node` + `joint_command_core`

We convert high-level arm joint targets (`ArmPose`) into per-motor CAN commands (`MotorCmd`), with YAML-driven calibration and runtime safety moderation (clamp, rate limit, smoothing). Intended for policy / teleop outputs before hardware.

## Pipeline

**Input:** `common_msgs/ArmPose` on `/behaviour/arm_pose` (6 angles: 3 shoulder, 2 elbow, 1 wrist).

**Output:** six `common_msgs/MotorCmd` messages on `/interfacing/motorCMD` (`POSITION_LOOP` by default).

**Node behavior:**
1. On each `ArmPose`, run moderation → publish moderated commands.
2. Timer at `control_rate_hz` republishes the latest moderated commands (keeps position hold on motors).

## Per-joint processing (`armPoseToMotorCmds`)

For each joint $i$, let $q^{\mathrm{in}}_i$ be the incoming angle (degrees, same units as `hardware_mapping.yaml`).

Repeat in order (skip rate/smooth steps on the first message after startup):

1. **Position clamp** — if enabled, clip to hardware limits:
   $$
   q \leftarrow \mathrm{clip}(q,\ q_{\min},\ q_{\max}).
   $$
2. **Velocity limit** — cap change per control tick using previous moderated target $q^{\mathrm{prev}}_i$:
   $$
   \Delta q_{\max} = \frac{\texttt{velocity\_max}}{\texttt{control\_rate\_hz}}.
   $$
3. **Delta limit** — additional per-step cap `delta_max` (degrees/tick).
4. **Low-pass** — exponential smoothing with $\alpha =$ `low_pass_alpha`:
   $$
   q \leftarrow \alpha\, q^{\mathrm{prev}} + (1-\alpha)\, q.
   $$
5. **Position clamp again** — limits still hold after smoothing.
6. **Calibration** — map to motor frame before publish:
   $$
   q_{\mathrm{motor}} = \texttt{direction} \cdot (q - \texttt{zero\_offset}).
   $$

Store $q$ as $q^{\mathrm{prev}}$ for the next message.

## Config files

| File | Role |
|------|------|
| `config/joint_command.yaml` | ROS params: arm side, topics, control rate, control type |
| `config/hardware_mapping.yaml` | Per-joint `can_id`, limits, `direction`, `zero_offset` |
| `config/safety_limits.yaml` | Moderation toggles and per-joint `velocity_max`, `delta_max`, `low_pass_alpha` |

Safety YAML uses a top-level `safety:` key with `global` defaults and optional `joints` overrides (shoulder/elbow/wrist paths match hardware mapping).

## Tuning `safety_limits.yaml`

Units are **degrees** and **deg/s**. At 50 Hz, `velocity_max: 100` implies up to **2.0°/tick** from the velocity limiter.

Start conservative on hardware, then increase until motion is responsive without jitter or limit hitting. Current values are bench defaults, not policy-tuned.

| Parameter | Effect |
|-----------|--------|
| `velocity_max` | Max joint speed (converted to °/tick) |
| `delta_max` | Hard cap on ° change per tick |
| `low_pass_alpha` | Higher → smoother/slower (e.g. `0.85`) |
| `enable_*` | Toggle each stage without recompiling |

## Launch

```bash
ros2 launch joint_command joint_command.launch.py
```

**Defaults:** `arm_side=left`, `control_rate_hz=50`, `control_type=POSITION_LOOP` (4).
