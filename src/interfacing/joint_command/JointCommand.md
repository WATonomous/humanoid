# Joint command: `joint_command_node` + `joint_command_core`

We convert high-level arm joint targets (`ArmPose`) into per-motor CAN commands (`MotorCmd`), with YAML-driven calibration and runtime safety moderation (clamp, rate limit, smoothing). Intended for policy / teleop outputs before hardware.

## Pipeline

**Input:** `common_msgs/ArmPose` on `/arm/joint_targets` (6 angles: 3 shoulder, 2 elbow, 1 wrist).

**Output:** six `common_msgs/MotorCmd` messages on `/interfacing/motorCMD` (`POSITION_LOOP` by default).

**Node behavior:**
1. Each `ArmPose` is cached.
2. A timer at `control_rate_hz` runs moderation one step and publishes, so `velocity_max` is a
   true deg/s bound however fast `ArmPose` arrives.
3. Nothing is published until the rate-limiter has been seeded from real feedback.
4. If no `ArmPose` arrives within `command_timeout_sec`, commands stop and the next one re-seeds.

## Per-joint processing (`armPoseToMotorCmds`)

For each joint $i$, let $q^{\mathrm{in}}_i$ be the incoming angle (degrees, same units as `hardware_mapping.yaml`).

Repeat in order, once per control tick:

1. **Position clamp** — if enabled, clip to hardware limits:
   $$
   q \leftarrow \mathrm{clip}(q,\ q_{\min},\ q_{\max}).
   $$
2. **Low-pass** — exponential smoothing with $\alpha =$ `low_pass_alpha`:
   $$
   q \leftarrow \alpha\, q^{\mathrm{prev}} + (1-\alpha)\, q.
   $$
3. **Velocity limit** — cap change per control tick using previous moderated target $q^{\mathrm{prev}}_i$:
   $$
   \Delta q_{\max} = \frac{\texttt{velocity\_max}}{\texttt{control\_rate\_hz}}.
   $$
4. **Delta limit** — additional per-step cap `delta_max` (degrees/tick).
5. **Position clamp again** — limits still hold after smoothing. If it bites (joint outside its
   limits), the move back into range is rate-limited too, not snapped.
6. **Calibration** — map to motor frame before publish:
   $$
   q_{\mathrm{motor}} = \texttt{direction} \cdot (q - \texttt{zero\_offset}).
   $$

Store $q$ as $q^{\mathrm{prev}}$ for the next tick.

## MIT joints

`control_type` is set per joint in `safety_limits.yaml` (`-1` = node default). All six joints
currently run `MIT_CONTROL` (0): the GL40 wrist (`mit_family: gl2`) and the AKs (`ak`). See "MIT
mode" in [can/README.md](../can/README.md).

A MIT joint is a PD drive with no internal limit checking, so `joint_command` adds:

| Field | Role |
|---|---|
| `mit_kp` / `mit_kd` | stiffness / damping (physical units) |
| `mit_max_torque` | fault above this torque |
| `mit_max_track_err` | fault if the joint lags its setpoint by more (deg) |
| `mit_feedback_timeout` | fault after this long without feedback |
| `mit_family` | `gl2` or `ak`; they disagree on the status byte (AK 1 = over-temperature) |
| `mit_fault_action` | `limp` (kp = kd = 0, `MIT_EXIT`) or `damp` (kp = 0, kd = `mit_fault_kd`); default `damp` for `ak` |
| `mit_fault_kd` | damping for `damp`, in (0, 5] |

**Startup rule** (the node refuses to launch otherwise): quantised `mit_kp` × `mit_max_track_err`
(rad) ≤ `mit_max_torque`.

Lifecycle: `MIT_ENTER` at startup, zero-stiffness frames until seeded, then gains, then
zero-stiffness frames again when the stream goes stale. A fault latches: `limp` joints get
`MIT_EXIT`, `damp` joints keep getting damping frames (a silent AK trips its CAN timeout and drops
the arm). On Ctrl-C, `damp` joints are damped for `mit_shutdown_damp_sec` (default 2 s), then every
MIT joint is exited. Stop `joint_command` **before** `can_node`, with the arm supported.

## Excluded joints

At seeding, a joint with no feedback in the last 0.5 s, or physically outside its limits (stale
calibration: re-run `calibrate_arm.py`), is **excluded** until the next seed. Servo joints get no
command and MIT joints get their fault action. It is never ramped from an assumed 0 or clamped to
a limit, and the rest of the arm keeps working.

## Config files

| File | Role |
|------|------|
| `config/joint_command.yaml` | ROS params: arm side, topics, control rate, control type |
| `config/hardware_mapping.yaml` | Per-joint `can_id`, limits, `direction`, `zero_offset` |
| `config/safety_limits.yaml` | Moderation toggles, per-joint `velocity_max`, `delta_max`, `low_pass_alpha`, `control_type`, MIT gains and limits |

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
| `control_type` | Per joint; `0` = MIT_CONTROL, `4` = POSITION_LOOP, `-1` = node default |
| `mit_*` | MIT gains and fault thresholds (see above) |

## Tests

`colcon test --packages-select joint_command` runs gtests against the **shipped** config, so an
unsafe edit (clamp off, velocity past the 2 rad/s testing ceiling, broken gain rule) fails the build.

## Launch

```bash
ros2 launch joint_command joint_command.launch.py
```

**Defaults:** `arm_side=left`, `control_rate_hz=50`, `control_type=POSITION_LOOP` (4), overridden per joint in `safety_limits.yaml`.
