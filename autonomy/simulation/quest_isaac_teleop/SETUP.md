# Quest Isaac Teleop Setup

End-to-end guide for teleoperation of the simulated humanoid arm in Isaac Lab using the Quest 2 controller.

---

## Architecture Overview

```
Quest 2 headset
    │  (WebXR over WSS)
    ▼
[Docker container]
  quest_teleop_node  →  publishes /quest_teleop (common_msgs/QuestHandPose)
    │  (ROS2 over UDP, network_mode: host)
    ▼
[Local machine]
  quest_Isaac.py  →  subscribes /quest_teleop  →  drives Isaac Lab sim
```

---

## Problems Encountered and Fixes

### 1. `ros2 topic echo /quest_teleop` was frozen locally

**Cause:** Two separate issues:

- **ROS_DOMAIN_ID mismatch** — the container and local machine were on different DDS domains so they could not discover each other's nodes.
- **FastDDS shared memory transport** — even with `network_mode: host`, FastDDS defaults to shared memory for same-host communication. The container and local processes are in different mount namespaces so `/dev/shm` is not shared between them, causing data transfer to silently fail even though topic discovery worked.

**Fix:** Set `ROS_DOMAIN_ID=0` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` on both sides to force UDP transport.

Changes made:
- Added to `modules/docker-compose.teleop.yaml`:
  ```yaml
  environment:
    - ROS_DOMAIN_ID=0
    - FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  ```
- Added to `~/.bashrc`:
  ```bash
  export ROS_DOMAIN_ID=0
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  ```

---

### 2. `common_msgs` not found locally

**Cause:** `common_msgs` is a custom ROS2 package built inside the container. It is not installed on the local machine by default, so `quest_Isaac.py` could not import `QuestHandPose`.

**Fix:** Build it locally once:
```bash
cd ~/Wato/humanoid/autonomy/wato_msgs
colcon build --packages-select common_msgs
```

---

### 3. Isaac node not subscribing (`Subscription count: 1` in `ros2 topic info`)

**Cause:** `isaaclab.sh` uses Isaac Sim's bundled Python (`_isaac_sim/kit/python/bin/python3`), not the system Python. When launched without sourcing ROS2 first, the DDS shared libraries are not available in that Python environment, so `rclpy` silently fails to connect to the ROS2 network.

**Fix:** Source ROS2 and `common_msgs` in the same terminal before launching `isaaclab.sh`.

---

## One-Time Setup

### 1. Apply `~/.bashrc` changes
```bash
source ~/.bashrc
```

### 2. Build `common_msgs` locally (only needed once, or after message changes)
```bash
cd ~/Wato/humanoid/autonomy/wato_msgs
colcon build --packages-select common_msgs
```

### 3. Build `quest_isaac_teleop` locally (only needed once, or after code changes)
```bash
cd ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop
colcon build
```

### 4. Set up CloudXR.js client locally (only needed once)

The CloudXR.js sample app is not fully installable from public npm. NVIDIA ships
the `@nvidia/cloudxr` package as a `.tgz` file through NGC.

Download the CloudXR.js SDK tarball from NVIDIA NGC and put it in `~/Downloads`,
for example:

```text
~/Downloads/nvidia-cloudxr-6.2.0.tgz
```

Then install the sample client:

```bash
cd ~/robotics
git clone https://github.com/NVIDIA/cloudxr-js-samples.git
cd ~/robotics/cloudxr-js-samples/simple

npm install ~/Downloads/nvidia-cloudxr-6.2.0.tgz
npm install
```

### 5. Set up USB reverse networking for Quest 2

Plain `adb reverse` is enough to open local TCP web pages from the Quest browser.
For CloudXR media over USB, keep `gnirehtet` available because CloudXR/WebRTC uses
UDP media traffic too.

Download and unpack `gnirehtet`:

```bash
cd ~/Downloads
wget https://github.com/Genymobile/gnirehtet/releases/download/v2.5/gnirehtet-rust-linux64-v2.5.zip
unzip gnirehtet-rust-linux64-v2.5.zip
```

Important Quest browser behavior: with `adb reverse`, use `localhost`, not
`127.0.0.1`.

---

## Running the Teleop (Every Session)

### Terminal 1 — Start the Docker container
```bash
cd ~/Wato/humanoid
./watod up
Go into the container you just started
```

Inside the container, start the quest_teleop node and WebXR server:
```bash
ros2 run quest_teleop quest_teleop_node
```

In a second pane inside the container, start the WebXR server:
```bash
python3 /root/ament_ws/src/teleop/quest_teleop/scripts/webxr_server.py
```

### Terminal 2 — Start the Isaac Lab sim node (local machine)
```bash
source /opt/ros/humble/setup.bash
source ~/Wato/humanoid/autonomy/wato_msgs/install/setup.bash
source ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/install/setup.bash

ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ~/robotics/IsaacLab/isaaclab.sh -p \
  ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py
```

For the VR/CloudXR view, launch the same sim with the OpenXR extension enabled:

```bash
source /opt/ros/humble/setup.bash
source ~/Wato/humanoid/autonomy/wato_msgs/install/setup.bash
source ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/install/setup.bash

ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ~/robotics/IsaacLab/isaaclab.sh -p \
  ~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py \
  --xr
```

`quest_Isaac.py` should enable cameras before creating the `AppLauncher`:

```python
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
```

In Isaac's XR panel, select the CloudXR/WebRTC OpenXR runtime and start XR. The
CloudXR.js client connects to this server on port `49100`.

### Terminal 3 — Start the CloudXR.js client

Start the browser client on the local machine:

```bash
cd ~/robotics/cloudxr-js-samples/simple
npm run dev-server
```

Leave this terminal running. The client should print:

```text
Loopback: http://localhost:8080/
On Your Network (IPv4): http://<your-pc-ip>:8080/
```

### Terminal 4 — Start Quest USB forwarding

In a new terminal, verify the Quest is connected:

```bash
adb devices
```

Start `gnirehtet` so CloudXR/WebRTC UDP media can travel over USB:

```bash
cd ~/Downloads/gnirehtet-rust-linux64
./gnirehtet run
```

Accept the VPN prompt inside the Quest. Leave this terminal running.

In another terminal, reverse the local web ports:

```bash
adb reverse tcp:8080 tcp:8080
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
adb reverse --list
```

Expected reverse list includes:

```text
tcp:8080 tcp:8080
tcp:8443 tcp:8443
tcp:9090 tcp:9090
```

Open the CloudXR.js client in the Quest browser:

```text
http://localhost:8080/
```

Open the Quest teleop WebXR page in the Quest browser:

```text
https://localhost:8443/
```

`9090` is rosbridge/WebSocket traffic, not a normal webpage.

### CloudXR.js connection details

In the CloudXR.js page on the Quest, use these values:

```text
Server IP Address: <your-pc-ip>
Port: 48010
Immersive Mode: VR
```

Find `<your-pc-ip>` on the local machine:

```bash
hostname -I
```

Use the real LAN/school-network address, for example:

```text
10.36.145.22
```

Do not use Docker bridge addresses like:

```text
172.17.0.1
172.18.0.1
```
give me teh steps tp set this up
Use `localhost` only for browser pages opened through `adb reverse`, such as:

```text
http://localhost:8080/
https://localhost:8443/
```

Use `<your-pc-ip>` inside CloudXR.js when it asks where the CloudXR server is.
With `gnirehtet` running, that traffic should travel over the USB VPN instead of
depending on eduroam peer-to-peer access.

### Terminal 5 — Verify ROS (optional)
```bash
# Check topic is visible
ros2 topic list

# Check publisher and subscriber are both connected
ros2 topic info /quest_teleop

# Inspect messages
ros2 topic echo /quest_teleop
```

Expected output of `ros2 topic info /quest_teleop`:
```
Type: common_msgs/msg/QuestHandPose
Publisher count: 1
Subscription count: 1   # increases by 1 for each ros2 topic echo you run
```

---

## Key Files Changed

| File | Change |
|------|--------|
| `modules/docker-compose.teleop.yaml` | Added `ROS_DOMAIN_ID=0` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` to container environment |
| `~/.bashrc` | Exported `ROS_DOMAIN_ID=0` and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for local shell |
