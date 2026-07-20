# Real arm bring-up: calibrate → visualize → move

Three separate steps, in three different packages. This page is just the entry point —
each step's real detail lives in its own doc.

## 1. Calibrate
Per-joint zero + limits, from live motor feedback. **Re-run after every power-on** for
elbow.pitch, elbow.roll, and shoulder.yaw (AK80-9 motors) — their single-turn absolute
encoders don't reliably survive a power cycle; shoulder.pitch/roll (AK10-9) have so far.

→ [autonomy/interfacing/can/README.md](autonomy/interfacing/can/README.md) — hardware
bring-up, `can_node`, `calibrate_arm.py` usage.

## 2. Visualize
Read-only mirror of live motor feedback in the browser. Confirms calibration looks right
before commanding anything.

```bash
./watod -t mjlabs
python3 autonomy/simulation/Humanoid_Wato/wato_bimanual_arm/live_arm_mjviser.py \
  --arm-side left --urdf-side right
# open http://localhost:8080
```
→ script docstring in `live_arm_mjviser.py` for full flag reference (`--flip`, `--offset`, etc).

## 3. Move (optional, real motor control)
Only after 1–2 look right. Rate-limited, seeds from live feedback (no startup slam).

→ [autonomy/behaviour/joint_command/MOVE_ARM_RUNBOOK.md](autonomy/behaviour/joint_command/MOVE_ARM_RUNBOOK.md)
