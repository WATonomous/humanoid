# Quest Bimanual Arm Teleop

Hand-tracking data from the Quest 2 drives both arms in Isaac Sim 5.1.
Everything runs inside the `simulation_il` container — no separate teleop
container is needed.

```
Quest WebXR browser
    │  WSS port 9090 via adb reverse
    ▼
quest_teleop_node  (simulation_il container)
    │  ROS 2 /quest_teleop topic
    ▼
run_quest_bimanual_teleop.py  (simulation_il container)
    │  DifferentialIKController → joint targets
    ▼
Isaac Sim bimanual arm  (rendered on PC monitor)
```

---

## Prerequisites — build the image first

The Dockerfile installs a separate Python 3.11 (deadsnakes) used only for
building ROS 2 packages. This must be baked in before the container will work:

```bash
# From repo root on the host — only needed once per Dockerfile change
ACTIVE_MODULES="simulation_il" ./watod build
```

This takes several minutes the first time. After that, `./watod up -d` uses the
cached image.

---

## Terminal 1 — Host: Internet sharing for Quest (optional)

```bash
gnirehtet run
```

Wait until you see repeating `TcpConnection` log lines — the Quest has internet.
Leave this running for the whole session.

---

## Terminal 2 — Host: USB port forwarding

Plug in the Quest over USB, then:

```bash
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
```

No output means success. This terminal is done.

---

## Terminal 3 — simulation\_il\_dev: ROS 2 hand tracking node

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

When you shell in, `.bashrc` automatically runs `colcon build` for `common_msgs`
and `quest_teleop`. Wait for:

```
Summary: 2 packages finished
```

Then run the node:

```bash
ros2 run quest_teleop quest_teleop_node
```

You should see `[INFO] WSS server listening on port 9090`. Leave this running.

---

## Terminal 4 — simulation\_il\_dev: HTTPS WebXR page server

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

Wait for `Summary: 2 packages finished`, then:

```bash
python3 /workspace/humanoid/autonomy/teleop/quest_teleop/scripts/webxr_server.py
```

You should see `Serving at https://0.0.0.0:8443`. Leave this running.

---

## Terminal 5 — simulation\_il\_dev: Isaac Sim + IK script

```bash
cd ~/Documents/Wato/humanoid && ./watod -t simulation_il_dev
```

Wait for `Summary: 2 packages finished`, then:

```bash
cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
./run_quest_bimanual_teleop.sh
```

Isaac Sim takes 1–3 minutes to load. Wait for:

```
[Quest] Ready. Waiting for /quest_teleop messages.
```

before putting on the headset.

---

## Quest Headset

1. Put on the headset
2. Open **Meta Browser** (not the Isaac Sim app)
3. Go to `https://localhost:8443/`
4. You'll get a security warning — click **Advanced → Proceed** (self-signed cert, expected)
5. Press **Start**

The Isaac Sim terminal should print hand data. Both arms follow your wrists.
Pinch thumb + index finger to close each gripper.

---

## Controls

| Action | Effect |
|--------|--------|
| Move right wrist | Right arm follows |
| Move left wrist | Left arm follows |
| Pinch right thumb+index | Right gripper closes/opens |
| Pinch left thumb+index | Left gripper closes/opens |

Pass `--gain VALUE` (default 4.0) to `run_quest_bimanual_teleop.sh` to scale
how far the arm moves per metre of wrist motion.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `colcon build` fails on shell entry | Image wasn't rebuilt after Dockerfile changed — run `ACTIVE_MODULES="simulation_il" ./watod build` first |
| `Package 'quest_teleop' not found` | Build didn't finish — wait for `Summary: 2 packages finished` |
| `Cannot run the interpreter /usr/bin/python3.11` | Same as above — image needs rebuild with `./watod build` |
| `quest_teleop_node: No such file or directory` | `ros2 run` requires sourced workspace — `.bashrc` does this; open a fresh shell into the container |
| Quest browser says connection refused | Make sure Terminals 3 and 4 are running and `adb reverse` was done |
| Arms don't move | Check Terminal 5 prints `[Quest] First wrist data` — if not, check Terminal 3 is publishing on `/quest_teleop` |
| Isaac Sim numpy broken after container restart | Container state was manually patched — you need `./watod build` to make the fix permanent |

---

## Mode B — CloudXR streaming (incomplete)

`bimanual_teleop_env_cfg.py` and `run_bimanual_teleop.sh` implement a CloudXR
path where Isaac Sim streams video to the Quest headset. This requires the
`isaaclab_teleop` / `isaacteleop` NVIDIA SDK packages which are not yet
installed in the container and is not functional.
