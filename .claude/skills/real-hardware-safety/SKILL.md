---
name: real-hardware-safety
description: >
  CRITICAL safety skill for the WATonomous humanoid robot arm project. Use this skill ANY time
  code is being written, reviewed, debugged, or modified that will run on or could run on real
  robot hardware — including motor control code, CAN bus commands, ROS2 launch files, trajectory
  planners, teleop scripts, sim-to-real transfer, zero-position calibration, or any script that
  sends commands to CubeMars AK10-9, AK80-9, or GL40 motors. Also trigger when the user mentions
  "real arm", "real hardware", "deploy", "on the robot", "test on hardware", "run on the arm",
  "sim to real", "sim2real", "zero position", "homing", "calibration", "motor test", or any
  indication that code will command physical actuators. Even if the user just asks for a "quick
  script to move the arm", trigger this skill. AI-generated code for robot arms has caused physical
  damage before (stripped screws, broken mounts, violent jerking) — this skill exists to prevent
  that from happening again.
---

# Real Hardware Safety Checker

## Why This Skill Exists

This team has experienced real hardware damage from deploying AI-generated code without sufficient
safety checks. In one incident, mismatched zero positions between simulation and the real arm
caused the arm to jerk violently at full speed, ripping threads off motor mounting screws. This
skill ensures that every piece of code destined for real hardware goes through a rigorous safety
review before it touches an actuator.

## MANDATORY: The Safety Checklist

Before generating, reviewing, or approving ANY code that will run on real hardware, Claude MUST
walk through every item in this checklist. Do not skip items. Do not assume things are fine. If
any item cannot be verified from the code/context, flag it explicitly and tell the user they must
verify it manually before running.

---

### 1. VELOCITY LIMITS — Is everything slow?

**THIS IS THE #1 KILLER. Check it first.**

- [ ] All joint velocity commands are clamped to **safe testing values**
- [ ] For initial testing / bring-up: max **2 rad/s** per joint (not the motor's full 26+ rad/s)
- [ ] For validated trajectories: max **5 rad/s** unless the user has explicitly confirmed higher
- [ ] Velocity limits are enforced in code, not just "expected" from the trajectory

**What to look for in code:**
```python
# BAD — no velocity limit, motor can go full speed
motor.velocity = target_velocity

# GOOD — explicit clamp
MAX_VEL = 2.0  # rad/s — TESTING LIMIT
motor.velocity = np.clip(target_velocity, -MAX_VEL, MAX_VEL)
```

```cpp
// BAD — raw command passthrough
velocity_cmd = msg->velocity;

// GOOD — clamped
constexpr double MAX_VEL = 2.0;  // rad/s — TESTING LIMIT
velocity_cmd = std::clamp(msg->velocity, -MAX_VEL, MAX_VEL);
```

**If velocity limiting is missing, DO NOT just mention it — inject it into the code and explain
why.** This is non-negotiable.

---

### 2. ZERO POSITION / HOMING — Are sim and real aligned?

**This is what caused the screw-ripping incident.**

- [ ] The code does NOT assume the motor's zero position matches the simulation's zero position
- [ ] There is an explicit calibration/homing step, OR the code reads the current motor position
      before commanding any movement
- [ ] The first motion command is a small, slow move relative to the CURRENT position, not an
      absolute position that could be far from where the arm physically is
- [ ] If using MIT mode: `set_zero_position()` is called only when the arm is in a known,
      verified physical configuration

**What to look for:**
```python
# EXTREMELY DANGEROUS — sends absolute position without knowing where arm is
motor.position = 0.0  # "go to zero" — but where is zero??

# SAFE — read current position first, then move relative
current_pos = motor.get_position()
target_pos = current_pos + 0.1  # small incremental move
motor.position = target_pos
```

**Critical questions to surface to the user:**
- "Have you verified that the motor zero positions match your URDF/sim zero positions?"
- "Is the arm currently in the position your code thinks it's in?"
- "Have you done a manual check by reading encoder values before sending commands?"

---

### 3. JOINT POSITION LIMITS — Can the arm reach impossible configurations?

- [ ] Every joint has min/max position limits defined
- [ ] Limits match the physical mechanical stops of the actual arm (not just the motor's ±12.5 rad range)
- [ ] Limits are enforced at the command level, not just in the URDF
- [ ] The limits account for cable routing, sensor wiring, and structural interference

**Recommended limits structure (adapt to actual arm geometry):**
```python
JOINT_LIMITS = {
    'shoulder_1': {'min': -1.57, 'max': 1.57},   # radians
    'shoulder_2': {'min': -1.57, 'max': 1.57},
    'elbow_1':    {'min': -2.09, 'max': 2.09},
    'elbow_2':    {'min': -1.57, 'max': 1.57},
    'elbow_3':    {'min': -1.57, 'max': 1.57},
    'wrist':      {'min': -3.14, 'max': 3.14},
    'gripper':    {'min': 0.0,   'max': 1.0},
}
```

---

### 4. TORQUE LIMITS — Can the motors overpower the structure?

- [ ] Torque/current commands are clamped well below motor peak ratings for testing
- [ ] For AK10-9 (peak 53Nm): limit to ≤10Nm during testing
- [ ] For AK80-9 (peak 22Nm): limit to ≤5Nm during testing
- [ ] For GL40 (peak 0.73Nm): limit to ≤0.3Nm during testing
- [ ] If using MIT mode impedance (Kp/Kd), gains are low enough that the resulting torque
      stays within safe bounds even at the position limit

**What to look for:**
```python
# BAD — full Kp, arm will slam into hard stops
Kp = 50.0
Kd = 5.0

# GOOD — compliant enough to push away by hand
Kp = 3.0   # low stiffness for testing
Kd = 0.3   # light damping
```

---

### 5. EMERGENCY STOP — Can you kill it instantly?

- [ ] The code has a software E-stop mechanism (keyboard interrupt, ROS service, etc.)
- [ ] On E-stop, ALL motors receive zero-torque commands immediately
- [ ] The user has been reminded about the HARDWARE E-stop requirement

**Remind the user every time:**
> ⚠️ **HARDWARE E-STOP REMINDER**: Before running this on the real arm, confirm you have a
> physical emergency stop button wired in series with the 48V motor power supply. Software
> E-stops are a complement, not a replacement. If your control PC freezes, only a hardware
> E-stop can save the arm.

**Software E-stop pattern:**
```python
import signal
import sys

def emergency_stop(signum, frame):
    print("E-STOP TRIGGERED — killing all motors")
    for motor in motors:
        motor.torque = 0.0
        motor.velocity = 0.0
        motor.update()
    for motor in motors:
        motor.disable()
    sys.exit(1)

signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)
```

---

### 6. CAN BUS WATCHDOG — What happens if communication drops?

- [ ] Motors have CAN timeout enabled (verify via R-Link, recommend 100-200ms)
- [ ] The control loop monitors for missed feedback messages
- [ ] If a motor stops responding, the code shuts down ALL motors, not just the unresponsive one
- [ ] The control loop runs at a consistent rate (e.g., 100Hz+) — if the loop slows down,
      the motor timeout should catch it

---

### 7. STARTUP SEQUENCE — Does the code ramp up safely?

- [ ] Motors are enabled one at a time, not all simultaneously
- [ ] After enabling, there is a brief pause to read initial state before commanding
- [ ] The first commands are zero-torque / hold-current-position, not "go to target"
- [ ] There is a gradual ramp from current position to desired position

**Safe startup pattern:**
```python
# 1. Enable motors one by one
for motor in motors:
    motor.enable()
    time.sleep(0.5)
    current_pos = motor.get_position()
    print(f"Motor {motor.id} at position {current_pos:.3f} rad")

# 2. Verify positions look reasonable
for motor in motors:
    pos = motor.get_position()
    if abs(pos) > EXPECTED_RANGE:
        print(f"WARNING: Motor {motor.id} position {pos:.3f} outside expected range!")
        print("Aborting — check zero positions")
        emergency_stop()

# 3. Hold current positions with low gains before moving
for motor in motors:
    motor.set_impedance_gains(Kp=2.0, Kd=0.2)
    motor.position = motor.get_position()  # hold where you are
    motor.update()

time.sleep(2.0)  # hold for 2 seconds, observe behavior

# 4. Slowly ramp to target
```

---

### 8. SIM-TO-REAL TRANSFER — Has the code been verified in simulation first?

- [ ] The trajectory / controller has been tested in simulation (Gazebo, MuJoCo, etc.)
- [ ] Joint positions, velocities, and torques from sim have been plotted and reviewed
- [ ] The sim uses the same URDF, joint limits, and control gains as the real deployment
- [ ] Any sim-specific parameters (gravity compensation, friction) have been adjusted for real

**Ask the user:**
- "Have you run this in simulation first? What did the joint trajectories look like?"
- "Are the URDF joint limits identical between your sim config and your real config?"

---

### 9. THERMAL MONITORING — Will the motors overheat?

- [ ] Motor temperature is being read from CAN feedback
- [ ] There is a thermal shutdown threshold (recommend 60°C for testing, check datasheet)
- [ ] Long-duration tests have duty cycle limits or rest periods

---

### 10. STRUCTURAL INTEGRITY — Has the physical arm been inspected?

**Remind the user to physically check:**
- All motor mounting screws are tight and use threadlocker where appropriate
- 3D-printed brackets are not cracked or deformed
- Cables and wires have strain relief and won't snag during motion
- The arm is securely clamped to a stable surface
- Nothing is in the arm's workspace that could be damaged

---

## How To Present This Review

When reviewing code for real hardware deployment, Claude should:

1. **Lead with the safety review** — before discussing the code's functionality, logic, or
   elegance, go through the checklist above
2. **Use a clear pass/fail format** for each checklist item
3. **Inject fixes directly** — don't just say "you should add velocity limiting"; actually
   modify the code to include it
4. **Print the hardware E-stop reminder** every single time
5. **End with a "Ready to Run?" summary** like this:

```
═══════════════════════════════════════════════════════
  REAL HARDWARE SAFETY REVIEW
═══════════════════════════════════════════════════════
  ✅ Velocity limits:      Clamped to 2.0 rad/s
  ✅ Zero position:        Reads current pos before moving
  ✅ Joint limits:          Defined and enforced
  ⚠️  Torque limits:       Using Kp=10 — consider lowering to 3-5 for first test
  ✅ Software E-stop:       SIGINT handler present
  ❌ CAN watchdog:          No timeout monitoring — ADD THIS
  ✅ Startup sequence:      Gradual ramp-up
  ✅ Sim verification:      User confirmed sim test passed
  ✅ Thermal monitoring:    Temperature read in feedback loop
  ⚠️  Physical inspection:  REMIND USER TO CHECK SCREWS AND MOUNTS
═══════════════════════════════════════════════════════
  STATUS: NOT READY — fix CAN watchdog before running
═══════════════════════════════════════════════════════
```

---

## Motor-Specific Reference

### CubeMars AK10-9 V3.0 (Shoulder joints)
- Rated torque: 18 Nm / Peak: 53 Nm
- Gear ratio: 9:1
- MIT mode position range: ±12.5 rad (output shaft)
- MIT mode velocity range: ±50 rad/s (output shaft)
- **Safe testing torque: ≤10 Nm**
- **Safe testing velocity: ≤2 rad/s**

### CubeMars AK80-9 V3.0 (Elbow joints)
- Rated torque: 9 Nm / Peak: 22 Nm
- Gear ratio: 9:1
- MIT mode position range: ±12.5 rad (output shaft)
- MIT mode velocity range: ±50 rad/s (output shaft)
- **Safe testing torque: ≤5 Nm**
- **Safe testing velocity: ≤2 rad/s**

### CubeMars GL40 KV70 (Wrist + Gripper)
- Rated torque: 0.25 Nm / Peak: 0.73 Nm
- **Safe testing torque: ≤0.3 Nm**
- **Safe testing velocity: ≤3 rad/s**

### CAN Bus Configuration
- Bus type: CAN 2.0 Classic, 500 kbps
- Interface: CANable (SLCAN) via /dev/ttyACM0
- Termination: 120Ω at both ends required
- Communication timeout: set to 100-200ms via R-Link

---

## Common AI-Generated Code Mistakes That Break Hardware

These are patterns that AI code generators (including Claude) frequently produce that are
dangerous on real hardware:

1. **Assuming zero is zero** — AI writes `go_to_position(0, 0, 0, 0, 0, 0)` as a "home"
   command without realizing the motor's zero is wherever it was when powered on

2. **No velocity limiting** — AI generates position trajectories with no max velocity, so
   the motor tries to get there as fast as possible

3. **Full-stiffness impedance** — AI uses Kp=100 from a sim example where motors are
   idealized; on real hardware this means the arm becomes a battering ram

4. **Missing error handling** — AI writes the happy path but if a CAN message is dropped
   or a motor faults, the code hangs or crashes without disabling motors

5. **Copy-pasting sim parameters** — Sim gravity is often slightly off, friction models
   don't match, and encoder zeros are different. AI copies the sim config verbatim.

6. **Absolute position commands at startup** — The very first command is "go to position X"
   without reading where the arm currently is. If the arm is at position Y far from X, it
   moves at maximum speed to get there.

7. **No ramp-up** — Trajectory starts at full speed from t=0 instead of gradually
   accelerating from rest

**When generating code, Claude must actively avoid all seven of these patterns.**

---

## Final Note

This skill is deliberately conservative. It is better to be annoyingly cautious and have a
working robot than to be permissive and have a pile of broken 3D prints and stripped screws.
Every time you think "this check is overkill," remember the arm that ripped its own mounting
screws out because sim-zero ≠ real-zero.
