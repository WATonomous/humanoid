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
2. **Velocity/ramp limit** — one of two modes, selected per joint by `enable_trapezoidal_limit`:
   - **Plain clamp** (`enable_trapezoidal_limit: false`, default) — cap change per control tick
     using previous moderated target $q^{\mathrm{prev}}_i$:
     $$
     \Delta q_{\max} = \frac{\texttt{velocity\_max}}{\texttt{control\_rate\_hz}}.
     $$
     This clamp previously fed into a low-pass smoothing step, which re-discounted it: at
     $\alpha=0.85$, steady-state speed settled to roughly $(1-\alpha)$ of the clamped value —
     `velocity_max: 40` (deg/s) produced **~6 deg/s** in practice, not 40. Confirmed
     numerically. The low-pass has since been **removed entirely** (see below) — this plain
     clamp now delivers the full configured speed, just with no acceleration ramp (an instant
     jump to the allowed per-tick step rather than a smooth ramp-up).
   - **Trapezoidal ramp** (`enable_trapezoidal_limit: true`) — accelerate at `accel_max`
     (deg/s²) toward `velocity_max`, then decelerate at `accel_max` to land exactly on target
     with no overshoot, re-planned every tick (the target itself may still be moving, e.g. IK
     or teleop chasing a live cube). This mode **replaces** the plain clamp for that joint.
     Not yet bench-tested on real hardware; keep off until validated on your own arm at a low
     `accel_max`, then raise gradually.
3. **Delta limit** — additional per-step cap `delta_max` (degrees/tick), applied in both modes.
4. **Position clamp again** — limits still hold after rate-limiting.
5. **Calibration** — map to motor frame before publish:
   $$
   q_{\mathrm{motor}} = \texttt{direction} \cdot (q - \texttt{zero\_offset}).
   $$

Store $q$ as $q^{\mathrm{prev}}$ (and, for the trapezoidal mode, its ramp velocity) for the next message.

**Why the low-pass was removed rather than just disabled:** it never bounded speed on its own
— for a large target jump its first step is `(1-alpha)` of the *entire* error, which scales
unboundedly with the jump size, unlike the clamp's fixed deg/tick ceiling. Once the
trapezoidal ramp covers the "smooth motion" use case, the low-pass had no remaining purpose
that wasn't better served by either the plain clamp or the ramp, so it was deleted rather
than left as a footgun someone could re-enable later.

### Structural limitation (both modes)

A continuously streaming target source (IK chasing a moving cube, VR teleop, a policy) feeds
the moderator a target that's only slightly ahead of the arm's current position *every tick* —
so the ramp/clamp may never reach `velocity_max` regardless of its value, since there's rarely
enough distance-to-target to justify cruising. This is structural, not a moderator bug. The
robust fix is to send velocity/accel as part of the target and let the motor's own firmware
servo loop (CAN `POSITION_VELOCITY`, see `can_node.cpp`) handle the ramp instead of inferring
speed from position deltas on the ROS side — see the CAN interfacing docs for the deg/s→ERPM
conversion needed to use that control type.

## Config files

| File | Role |
|------|------|
| `config/joint_command.yaml` | ROS params: arm side, topics, control rate, control type |
| `config/hardware_mapping.yaml` | Per-joint `can_id`, limits, `direction`, `zero_offset` |
| `config/safety_limits.yaml` | Moderation toggles and per-joint `velocity_max`, `delta_max`, `accel_max` |

Safety YAML uses a top-level `safety:` key with `global` defaults and optional `joints` overrides (shoulder/elbow/wrist paths match hardware mapping).

## Tuning `safety_limits.yaml`

Units are **degrees** and **deg/s**. At 50 Hz, `velocity_max: 100` implies up to **2.0°/tick** from the velocity limiter.

Start conservative on hardware, then increase until motion is responsive without jitter or limit hitting. Current values are bench defaults, not policy-tuned.

| Parameter | Effect |
|-----------|--------|
| `velocity_max` | Max joint speed — cruise speed in trapezoidal mode, plain clamp otherwise |
| `accel_max` | Trapezoidal mode only: acceleration/deceleration rate (deg/s²) |
| `delta_max` | Hard cap on ° change per tick |
| `enable_trapezoidal_limit` | Switch from the plain clamp to the trapezoidal ramp (per-joint) |
| `enable_*` | Toggle each stage without recompiling |

## Launch

```bash
ros2 launch joint_command joint_command.launch.py
```

**Defaults:** `arm_side=left`, `control_rate_hz=50`, `control_type=POSITION_LOOP` (4).
