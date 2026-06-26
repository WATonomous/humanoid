# Quest Bimanual Arm Teleop

Two teleoperation modes are available.  **Mode A** (UDP bridge) is the one
that works today.  Mode B (CloudXR) is incomplete — it depends on
`isaaclab_teleop`/`isaacteleop` which are not yet in the container.

---

## Mode A — UDP bridge (works now)

Hand-tracking data flows:

```
Quest WebXR browser
    │  WSS port 9090 via adb reverse
    ▼
quest_teleop_node (teleop container)
    │  ROS 2 /quest_teleop topic
    ▼
quest_teleop_bridge.py (teleop container)
    │  JSON UDP  localhost:19090  (host-network: same on both containers)
    ▼
run_quest_bimanual_teleop.py (simulation_il container)
    │  DiffIK → joint targets
    ▼
Isaac Sim bimanual arm (rendered on PC monitor)
```

### Step 1 — Host: USB tunnel

Plug in the Quest over USB, then on the **host**:

```bash
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
```

### Step 2 — teleop container: ROS 2 node + WebXR page

```bash
# terminal A — inside teleop container
./watod -t teleop
cd /root/ament_ws
colcon build --packages-select common_msgs quest_teleop
source install/setup.bash
ros2 run quest_teleop quest_teleop_node
```

```bash
# terminal B — inside teleop container
./watod -t teleop
python3 /root/ament_ws/src/teleop/quest_teleop/scripts/webxr_server.py
```

### Step 3 — teleop container: UDP relay

```bash
# terminal C — inside teleop container
./watod -t teleop
cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
./run_relay.sh
```

### Step 4 — simulation_il container: launch Isaac Sim

```bash
# terminal D — inside simulation_il container
./watod -t simulation_il_dev
cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
./run_quest_bimanual_teleop.sh
```

Wait for `[Quest] Ready.` in the output.

### Step 5 — Quest: connect WebXR

Open `https://localhost:8443/` in the Quest browser, accept the self-signed
cert, then press **Start**.  Both arms move with your wrists.  Pinch thumb +
index to close each gripper.

### Controls

| Action | Effect |
|--------|--------|
| Move right wrist | Right arm follows |
| Move left wrist | Left arm follows |
| Pinch right thumb+index | Right gripper closes/opens |
| Pinch left thumb+index | Left gripper closes/opens |

### Tuning

Pass `--gain VALUE` (default 0.8) to `run_quest_bimanual_teleop.sh` to scale
how far the arm moves per metre of real wrist motion.

---

## Mode B — CloudXR streaming (incomplete)

`bimanual_teleop_env_cfg.py` and `run_bimanual_teleop.sh` implement a CloudXR
path where Isaac Sim streams video to the Quest headset.  This requires the
`isaaclab_teleop` / `isaacteleop` NVIDIA SDK packages which are not yet
installed in the container.  The `isaaclab_teleop` import in
`bimanual_teleop_env_cfg.py` is stubbed out so the module is importable, but
the retargeting pipeline inside it is non-functional until those packages are
added.

---

## Verify hand data is arriving

In a new simulation_il shell, before launching Isaac Sim you can test the
UDP data with:

```bash
python3 -c "
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 19090))
data, _ = s.recvfrom(65536)
msg = json.loads(data)
print('left wrist:', msg['left_wrist'])
print('right wrist:', msg['right_wrist'])
"
```
