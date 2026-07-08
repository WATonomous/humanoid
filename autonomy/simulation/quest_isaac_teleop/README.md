# Quest 2 Bimanual Arm Teleop

Hand-tracking data from a Quest 2 headset drives both arms in Isaac Sim 5.1
in real time. Everything runs inside a **single `simulation_il` container** —
no separate teleop container is needed.

```
Quest 2 headset (WebXR hand tracking)
    │  HTTPS port 8443  (adb reverse tunnel)
    ▼
webxr_server.py  ──────────────────────────────┐
                                               │ self-signed cert
    │  WSS port 9090  (adb reverse tunnel)     │  (must be trusted on Quest
    ▼                                          │   for BOTH ports 8443 + 9090)
quest_teleop_node  (ROS 2 WebSocket bridge)    │
    │  /quest_teleop  (ROS 2 topic)            │
    ▼                                          │
run_quest_bimanual_teleop.py                   │
    │  PinkIKController → joint cmds           │
    ▼                                          │
Isaac Sim 5.1  (rendered on host monitor) ◄────┘
```

All four processes run inside `simulation_il_dev`. The Quest browser connects
back to the host via `adb reverse` USB tunnels — no Wi-Fi or external network
needed.

---

## Architecture notes

### Why this is a single container now

The old architecture used two containers:
- `teleop` — ran `quest_teleop_node` + `webxr_server` + a UDP relay bridge
- `simulation_il` — received hand data over UDP sockets, had no ROS 2

The UDP bridge was only needed because `simulation_il` had no ROS 2. We
eliminated it by building ROS 2 Humble from source inside `simulation_il` and
having `run_quest_bimanual_teleop.py` subscribe to `/quest_teleop` directly.

### Why ROS 2 is built from source (not apt)

Isaac Sim 5.1 bundles its own Python 3.11. The apt ROS 2 packages (`rclpy`,
etc.) are compiled against Python 3.10 and **will not load** in Isaac Sim's
interpreter. Building from source against deadsnakes Python 3.11 produces
`cpython-311` ABI-tagged extensions that load in both the system Python 3.11
and Isaac Sim's bundled Python 3.11.

### The two Python 3.11 installs

There are intentionally **two** separate Python 3.11 installations in the
final image:

| Python | Path | Purpose |
|--------|------|---------|
| Isaac Sim bundled | `/workspace/isaaclab/_isaac_sim/kit/python/bin/python3.11` | Runs Isaac Sim, IK script, loads rclpy at runtime |
| deadsnakes (apt) | `/usr/bin/python3.11` | Used **only** by `colcon build` — never at runtime |

The deadsnakes install exists purely to satisfy cmake import paths baked into
`std_msgs`/`geometry_msgs` during the ROS 2 build stage. Its numpy version
must be unpinned (latest 2.x) to match the builder stage's numpy, since
numpy ≥ 2.0 uses `numpy/_core/include` and numpy < 2.0 uses `numpy/core/include`
— cmake export files bake this path in at build time.

**Never pip-install ROS 2 build tools into Isaac Sim's own Python (`$PYTHON`).
It has no C headers in its pip_prebundle paths and will cause packaging version
conflicts with lerobot/isaaclab-rl.**

---

## One-time setup

### 1. Build the image

The Dockerfile installs deadsnakes Python 3.11, builds ROS 2 Humble from
source, and bakes `colcon build` into `.bashrc`. Run this once after any
Dockerfile change:

```bash
# From repo root on the host
ACTIVE_MODULES="simulation_il" ./watod build
```

Takes 10–20 minutes the first time. Subsequent builds use Docker layer cache
and are much faster unless the ROS 2 build layer changes.

### 2. SSL certificates

The WebXR HTTPS server needs a self-signed cert. Generate once on the host:

```bash
mkdir -p autonomy/teleop/quest_teleop/certs
openssl req -x509 -newkey rsa:4096 -keyout autonomy/teleop/quest_teleop/certs/key.pem \
    -out autonomy/teleop/quest_teleop/certs/cert.pem -days 3650 -nodes \
    -subj "/CN=localhost"
```

The compose file bind-mounts this directory into the container at `/certs`.

### 3. Quest developer mode

The Quest must have **developer mode** enabled in the Meta mobile app so that
`adb` can connect over USB.

### 4. gnirehtet (optional — Quest internet)

gnirehtet lets the Quest use the host's internet connection over USB. Not
required for teleop to work, but useful so the Quest can load pages.

```bash
# Download to ~/gnirehtet if not already there
~/gnirehtet/gnirehtet-rust-linux64/gnirehtet install
```

---

## Running the pipeline

Start the container first, then open 5 terminals:

```bash
ACTIVE_MODULES="simulation_il" ./watod up -d
```

### Terminal 1 — Host: Internet sharing (optional)

```bash
~/gnirehtet/gnirehtet-rust-linux64/gnirehtet run
```

Wait for repeating `TcpConnection` log lines. Leave running.

### Terminal 2 — Host: USB port forwarding

Plug in the Quest over USB, then:

```bash
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
```

No output = success. Run `adb devices` first to confirm the Quest is detected.
This terminal is done.

### Terminal 3 — simulation\_il\_dev: ROS 2 hand tracking node

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

`.bashrc` automatically runs `colcon build` when you shell in. Wait for:

```
Summary: 2 packages finished
```

Then source the workspace and run the node:

```bash
source /opt/ros/humble/setup.bash
source /root/ament_ws/install/setup.bash
ros2 run quest_teleop quest_teleop_node
```

Wait for `[INFO] WSS server listening on port 9090`. Leave running.

> **Note:** If `ros2` command is not found, the image predates the ros2cli
> addition. Use the direct binary instead:
> `/root/ament_ws/install/quest_teleop/lib/quest_teleop/quest_teleop_node`
> Then rebuild the image to get `ros2` CLI support.

### Terminal 4 — simulation\_il\_dev: HTTPS WebXR page server

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

Wait for `Summary: 2 packages finished`, then:

```bash
python3 /workspace/humanoid/autonomy/teleop/quest_teleop/scripts/webxr_server.py
```

Wait for `Serving at https://0.0.0.0:8443`. Leave running.

### Terminal 5 — simulation\_il\_dev: Isaac Sim + IK script

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

Wait for `Summary: 2 packages finished`, then:

```bash
cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
./run_quest_bimanual_teleop.sh
```

Isaac Sim takes **1–3 minutes** to load. Wait for:

```
[Quest] Ready. Waiting for /quest_teleop messages.
```

before putting on the headset.

### Quest headset

The self-signed cert must be trusted separately for each port the browser
connects to. Missing either step will silently break the WebSocket connection
and `/quest_teleop` will receive no data.

1. Put on the headset
2. Open **Meta Browser** (not the Isaac Sim app)
3. Navigate to `https://localhost:9090` — click **Advanced → Proceed** to trust the WSS cert
4. Navigate to `https://localhost:8443/` — click **Advanced → Proceed** to trust the HTTPS cert
5. Press **Start**

Both arms should begin following your wrists. The Isaac Sim terminal will
print `[Quest] First wrist data` when the first message arrives.

---

## Controls

| Action | Effect |
|--------|--------|
| Move right wrist | Right arm follows |
| Move left wrist | Left arm follows |
| Pinch right thumb + index | Right gripper closes/opens |
| Pinch left thumb + index | Left gripper closes/opens |

Pass `--gain VALUE` (default `2.0`) to `run_quest_bimanual_teleop.sh` to scale
how far the arm moves per metre of real wrist motion.

---

## Debugging

Check what's publishing on the hand pose topic (requires sourced workspace):

```bash
source /opt/ros/humble/setup.bash
source /root/ament_ws/install/setup.bash
ros2 topic echo /quest_teleop
```

You should see `QuestHandPose` messages at ~30 Hz when the Quest is active.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `colcon build` fails silently on shell entry | Image wasn't rebuilt after Dockerfile changed | `ACTIVE_MODULES="simulation_il" ./watod build` |
| `Cannot run the interpreter /usr/bin/python3.11` | Same as above | Rebuild image |
| `ros2: command not found` | Image predates ros2cli addition | Rebuild image, or use direct binary path (see Terminal 3 note) |
| `quest_teleop_node: No such file or directory` | Build didn't finish yet | Wait for `Summary: 2 packages finished` |
| WebSocket closed immediately on Quest | `quest_teleop_node` not running | Start Terminal 3 first |
| Quest browser shows connection refused on port 8443 | `webxr_server.py` not running or `adb reverse` not done | Start Terminal 4 and re-run Terminal 2 |
| Quest browser shows `localhost terminated connection` | SSL issue or server not running | Confirm Terminal 4 is running; check cert files exist at `autonomy/teleop/quest_teleop/certs/` |
| Quest browser shows security warning | Self-signed cert | Expected — click Advanced → Proceed |
| `/quest_teleop` topic empty after pressing Start | WSS cert not trusted for port 9090 | In Quest browser, navigate to `https://localhost:9090` and accept the warning, then return to port 8443 and press Start again |
| Arms don't move after pressing Start | No hand data reaching IK script | Check Terminal 5 prints `[Quest] First wrist data`; check Terminal 3 is running and topic is publishing |
| Isaac Sim numpy broken after container restart | Container was manually patched (not image-built) | Run `./watod build` to make fixes permanent; restart container |
| `ModuleNotFoundError: No module named 'em'` during colcon | `empy` installed into wrong Python (Isaac Sim's, not deadsnakes) | Rebuild image — Dockerfile now uses `/usr/bin/python3.11` explicitly |
| `packaging` version conflict warnings | `catkin_pkg` was pip-installed into Isaac Sim's Python | Fixed in current Dockerfile — rebuild image |
| adb reverse shows no output | Quest not detected | Check USB cable, check developer mode is on, run `adb devices` |
| adb reverse entries disappear | Quest locked or USB disconnected | Unlock Quest, replug USB, re-run Terminal 2 |

---

## Tuning

### Translation scale

`--gain` (default `2.0`) scales how far the IK target moves per metre of real
wrist motion. Increase if the arms feel sluggish to reach positions; decrease
if they overshoot.

### IK tracking speed

Tracking aggressiveness is controlled by two parameters per end-effector task
in `quest_isaac_teleop/bimanual_pink_controller_cfg.py`:

| Parameter | Effect | Current value |
|-----------|--------|---------------|
| `gain` | How much of the pose error the solver corrects per time-step. Higher = faster convergence, but may overshoot. | `8.0` |
| `lm_damping` | Levenberg–Marquardt regularisation. Higher = smoother but slower, especially near the target. | `0.5` |

Pink IK uses a proportional controller: joint velocity ∝ `gain × error`. This
means the arm naturally slows down as it approaches the target. If it feels
too sluggish in the end-range, raise `gain` further or lower `lm_damping`.
If it becomes unstable (oscillating), lower `gain` or raise `lm_damping`.

Actuator responsiveness (how fast joints track the IK output) is set in
`autonomy/simulation/Teleop/keyboard-based\ teleoperation/bimanual_arm_cfg.py`
via `stiffness` and `damping` on each `ImplicitActuatorCfg`. Current values
are 2× the original motor datasheet figures for faster sim tracking.

### Coordinate frame mapping

`_QUEST_TO_WORLD` in `run_quest_bimanual_teleop.py` is the 3×3 rotation matrix
that maps Quest standing-space axes to simulation world-frame axes. It must
have determinant +1 (proper rotation) — a det = −1 matrix silently corrupts
orientation tracking via `quat_from_matrix`.

`_DEPTH_SIGN` is a per-axis sign vector `[sx, sy, sz]` applied to the
world-frame position delta after the matrix remap. Use it to flip individual
axes without touching the orientation-critical matrix.

If an axis feels backwards, flip the corresponding element of `_DEPTH_SIGN`
(currently `[1, 1, 1]`). Do **not** negate a row of `_QUEST_TO_WORLD` — that
changes the determinant sign and breaks orientation.

### Wrist orientation alignment

**Observed issue:** when the simulation first starts, the robot EE orientation
does not match the user's physical wrist orientation. Rotating the wrist in one
physical axis can drive the EE in a different axis in sim (~30° misalignment on
the right arm, ~90° on the left arm in testing).

**Root cause:** the Quest wrist tracking frame and the robot's link6/link6l
URDF frame have different axis conventions. After the Quest→World→Base frame
remap, the rotation axes do not 1:1 correspond between the physical wrist and
the EE.

**Current knob:** `_WRIST_ORIENT_OFFSET` (line ~135 in
`run_quest_bimanual_teleop.py`) is a `(w, x, y, z)` quaternion applied as a
conjugation on the world-frame orientation delta before converting to base
frame. Identity `[1, 0, 0, 0]` = no correction. Common trial values:

| Value | Meaning |
|-------|---------|
| `[1, 0, 0, 0]` | No correction (identity) |
| `[0, 0, 0, 1]` | 180° about world Z |
| `[0.707, 0, 0, 0.707]` | 90° about world Z |
| `[0.707, 0, 0.707, 0]` | 90° about world Y |
| `[0.707, 0.707, 0, 0]` | 90° about world X |

**Possible automatic fix (not yet implemented):** at startup, compute the
rotation from "Quest wrist in world frame" to "robot EE in world frame" using
the home poses captured on the first message, and apply that as the per-arm
correction automatically:

```python
# In the first-message block, after home poses are captured:
ee_quat_w_l  = quat_mul(root_quat_w, home_ee_quat_b_left)
ee_quat_w_r  = quat_mul(root_quat_w, home_ee_quat_b_right)
q_home_w_l   = quat_mul(quat_mul(quest_to_world_quat, quest_home_left_quat.unsqueeze(0)),
                         quat_inv(quest_to_world_quat))
q_home_w_r   = quat_mul(quat_mul(quest_to_world_quat, quest_home_right_quat.unsqueeze(0)),
                         quat_inv(quest_to_world_quat))
orient_corr_left  = quat_mul(ee_quat_w_l, quat_inv(q_home_w_l))   # (1, 4)
orient_corr_right = quat_mul(ee_quat_w_r, quat_inv(q_home_w_r))   # (1, 4)
```

Then replace `wrist_orient_offset` in the orientation loop with
`quat_mul(wrist_orient_offset, orient_corr_left/right)`. This absorbs the
fixed frame offset automatically at the cost of making the calibration
sensitive to whatever wrist orientation the user holds at the moment the first
message arrives.
