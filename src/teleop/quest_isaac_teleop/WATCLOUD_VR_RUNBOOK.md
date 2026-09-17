# Complete WATCloud Meta Quest VR Teleoperation Runbook

This runbook gives you the exact, terminal-by-terminal commands to start the interactive Slurm session, launch the Docker container, run the WebXR server, ROS 2 WebSocket node, and start Isaac Sim VR teleoperation with dual-camera recording.

---

## Architecture Flow

```
[Meta Quest Headset] (Browser visits https://localhost:8443 & https://localhost:9090)
        │
        ▼ (USB-C Cable via `adb reverse`)
[Windows PC] (SSH Tunnel via `ssh -L 5900... -L 8443... -L 9090... asd-dev-session`)
        │
        ▼ (WATCloud Cluster Network)
[WATCloud Host: trpro-slurm1]
   ├── webxr_server.py (HTTPS Web Server on Port 8443)
   └── [Docker Container: isaac-lab-ros2]
          ├── quest_teleop_node (ROS 2 WebSocket on Port 9090)
          └── run_quest_bimanual_teleop.sh (Isaac Sim + Dual Cameras + DLS IK)
```

---

## Terminal 1 (Windows Git Bash): Start Interactive Slurm Session

In a **fresh Git Bash terminal on Windows**:

```bash
cd ~/Downloads/Humanoid/wato_asd_tooling
bash start_interactive_session.sh
```

> **Keep this terminal window open at all times.** This maintains your Slurm GPU allocation and SSH daemon on WATCloud.

---

## Terminal 2 (Windows Git Bash): Start Docker & Enter Container

In a **second Git Bash window on Windows**:

```bash
# 1. SSH into the active dev session with full port forwarding
ssh -L 5900:localhost:5900 -L 8443:localhost:8443 -L 9090:localhost:9090 asd-dev-session

# 2. Export display & start rootless Docker daemon
cd ~/IsaacLab
export DISPLAY=:1
slurm-start-dockerd.sh

# 3. Start the Isaac Lab ROS 2 container
./docker/container.py start ros2
```

---

## Terminal 3 (Windows Git Bash): Run WebXR HTTPS Server (Port 8443)

In a **third Git Bash window on Windows** (runs directly on `trpro-slurm1` host):

```bash
# 1. SSH into dev session
ssh asd-dev-session

# 2. Go to humanoid repo and pull latest code
cd ~/FallRepo/humanoid
git pull origin act-simulation

# 3. Launch WebXR server (automatically generates SSL certs on first run)
python3 src/teleop/quest_teleop/scripts/webxr_server.py
```

> **Expected output**: `Serving at https://0.0.0.0:8443`  
> Leave Terminal 3 running.

---

## Terminal 4 (Windows Git Bash): ROS 2 WebSocket Bridge (Port 9090)

In a **fourth Git Bash window on Windows**:

```bash
# 1. SSH into dev session
ssh asd-dev-session

# 2. Enter the ROS 2 container
cd ~/IsaacLab
./docker/container.py enter ros2
```

Inside the container (`root@...:/workspace/isaaclab#`):

```bash
# 3. Install build dependencies (if running on a fresh container)
apt-get update && apt-get install -y nlohmann-json3-dev libboost-system-dev

# 4. Link ROS packages & build
mkdir -p /root/ament_ws/src
ln -s /workspace/isaaclab/FallRepo/humanoid/src/common_msgs /root/ament_ws/src/common_msgs 2>/dev/null || true
ln -s /workspace/isaaclab/FallRepo/humanoid/src/teleop /root/ament_ws/src/teleop 2>/dev/null || true

source /opt/ros/humble/setup.bash
cd /root/ament_ws
colcon build --packages-select common_msgs quest_teleop

# 5. Launch ROS WebSocket node
source /root/ament_ws/install/setup.bash
ros2 run quest_teleop quest_teleop_node
```

> **Expected output**: `WSS server listening on port 9090`  
> Leave Terminal 4 running.

---

## Terminal 5 (Windows Git Bash): Launch Isaac Sim VR Teleop

In a **fifth Git Bash window on Windows**:

```bash
# 1. SSH into dev session
ssh asd-dev-session

# 2. Enter the ROS 2 container
cd ~/IsaacLab
./docker/container.py enter ros2
```

Inside the container (`root@...:/workspace/isaaclab#`):

```bash
# 3. Ensure NumPy 1.26.4 compatibility and source ROS 2
/workspace/isaaclab/isaaclab.sh -p -m pip install "numpy==1.26.4"
source /opt/ros/humble/setup.bash
source /root/ament_ws/install/setup.bash

# 4. Pull latest teleop & launch Isaac Sim
cd /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_isaac_teleop
git pull origin act-simulation
./run_quest_bimanual_teleop.sh --record
```

> **Expected output**:
> ```text
> [Quest] Simulation ready (lightbox enclosure walls added).
> [Quest] Stereo head-tracked RealSense D455 pair attached.
> [Quest] Ready. Waiting for /quest_teleop messages.
> ```

---

## Terminal 6 (Windows Git Bash / PowerShell): Connect Headset via ADB

Plug your **Meta Quest** headset into your PC via USB-C, then in a **local Windows Git Bash / PowerShell window**:

```bash
# 1. Check device connection
adb devices
# (Ensure it says 'device'. If unauthorized, put on headset and click 'Allow USB Debugging')

# 2. Forward ports over USB
adb reverse --remove-all
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
adb reverse --list
```

---

## Inside the Meta Quest Headset

1. Put on your **Meta Quest** headset.
2. Open the **Meta Quest Browser**.
3. **Step A (Trust WebSocket SSL Certificate)**:
   - Go to: `https://localhost:9090`
   - Click **Advanced** $\rightarrow$ **Proceed to localhost (unsafe)**.  
   *(A blank page or websocket error is expected here)*.
4. **Step B (Open WebXR Teleop App)**:
   - Go to: `https://localhost:8443`
   - Click **Advanced** $\rightarrow$ **Proceed to localhost (unsafe)**.
5. Click **"Enter VR" / "Start"** on the screen.

---

## Keyboard Controls & Teleop Actions (In Terminal 5)

| Key / Gesture | Action |
| :--- | :--- |
| **Move Quest Controllers** | Robot dual arms follow hand poses in real time via DLS IK |
| **Index Triggers / Grips** | Open and close left and right grippers |
| **`O` Key** | **Save** current episode recording (`.h5` / `.zarr`) and auto-reset environment |
| **`P` Key** | **Discard** current episode recording and auto-reset environment |
| **`R` Key** | **Reset** simulation scene manually |
| **`C` Key** | **Recalibrate** controller home offset to current hand positions |
