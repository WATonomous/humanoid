# Complete WATCloud Meta Quest VR Teleoperation Runbook

This document provides the exhaustive, terminal-by-terminal step-by-step guide to launch the entire VR Teleoperation pipeline on WATCloud using the `wato_asd_tooling` session workflow.

---

## Architecture Overview

```
[Meta Quest Headset]  (Browser visiting https://localhost:8443 & :9090)
        │
        ▼ (USB-C Cable via `adb reverse`)
[Windows PC]  (SSH Tunnel via `asd-dev-session`)
        │
        ▼ (WATCloud Cluster Network)
[WATCloud Host: trpro-slurm1]  (Running `port_bridge.py`)
        │
        ▼ (`docker exec` Bridge)
[Docker Container: isaac-lab-ros2]
   ├── webxr_server.py (HTTPS 8443)
   ├── quest_teleop_node (WSS 9090)
   └── run_quest_bimanual_teleop.sh (Isaac Sim + Stereo RealSense + DLS IK)
```

---

## Terminal 1 (Windows Git Bash): Start Interactive Slurm Session

Open **Git Bash on Windows**:

```bash
cd ~/Downloads/Humanoid/wato_asd_tooling
bash start_interactive_session.sh
```

> **Note**: Leave this terminal window open at all times. It keeps your Slurm GPU session alive on WATCloud.

---

## Terminal 2 (Windows Git Bash): Start Docker & Host Port Bridge

Open a **second Git Bash window on Windows**:

```bash
# 1. SSH into the active dev session with port forwarding
ssh -L 5900:localhost:5900 -L 8443:localhost:8443 -L 9090:localhost:9090 asd-dev-session

# 2. Navigate to IsaacLab and export display
cd ~/IsaacLab
export DISPLAY=:1

# 3. Start Rootless Docker daemon
slurm-start-dockerd.sh

# 4. Start the IsaacLab container (select 'N' when asked for X11 forwarding)
./docker/container.py start ros2

# 5. Start the Host Port Bridge (pull latest first)
cd ~/IsaacLab/FallRepo/humanoid
git pull origin act-simulation
python3 src/teleop/quest_teleop/scripts/port_bridge.py
```

> **Expected Output**:
> ```text
> [PortBridge] Starting bridge for container 'isaac-lab-ros2'...
> [PortBridge] Listening on host 0.0.0.0:8443 -> container:8443
> [PortBridge] Port 9090 on host already in use -- skipping host bind.
> ```
> **Leave Terminal 2 running in the background.**

---

## Terminal 3 (Windows Git Bash): WebXR HTTPS Server (Port 8443)

Open a **third Git Bash window on Windows**:

```bash
# 1. SSH into the dev session
ssh asd-dev-session

# 2. Enter the running Docker container
cd ~/IsaacLab
./docker/container.py enter ros2

# 3. Generate SSL certificates & symlink (one-time setup per fresh container)
mkdir -p /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs

openssl req -x509 -newkey rsa:4096 \
  -keyout /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs/key.pem \
  -out /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs/cert.pem \
  -days 3650 -nodes \
  -subj "/CN=localhost"

ln -s /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs /certs 2>/dev/null || true

# 4. Start WebXR server
cd /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/scripts
python3 webxr_server.py
```

> **Expected Output**: `Serving at https://0.0.0.0:8443`  
> **Leave Terminal 3 running.**

---

## Terminal 4 (Windows Git Bash): ROS 2 Hand-Tracking Node (Port 9090)

Open a **fourth Git Bash window on Windows**:

```bash
# 1. SSH into the dev session
ssh asd-dev-session

# 2. Enter the running Docker container
cd ~/IsaacLab
./docker/container.py enter ros2

# 3. Install build headers & compile ROS 2 packages (one-time setup per fresh container)
apt-get update && apt-get install -y nlohmann-json3-dev libboost-system-dev

mkdir -p /root/ament_ws/src
ln -s /workspace/isaaclab/FallRepo/humanoid/src/common_msgs /root/ament_ws/src/common_msgs 2>/dev/null || true
ln -s /workspace/isaaclab/FallRepo/humanoid/src/teleop /root/ament_ws/src/teleop 2>/dev/null || true

source /opt/ros/humble/setup.bash
cd /root/ament_ws
colcon build --packages-select common_msgs quest_teleop

# 4. Source workspace and launch hand tracking node
source /root/ament_ws/install/setup.bash
ros2 run quest_teleop quest_teleop_node
```

> **Expected Output**: `WSS server listening on port 9090`  
> **Leave Terminal 4 running.**

---

## Terminal 5 (Windows Git Bash): Isaac Sim VR Simulation

Open a **fifth Git Bash window on Windows**:

```bash
# 1. SSH into the dev session
ssh asd-dev-session

# 2. Enter the running Docker container
cd ~/IsaacLab
./docker/container.py enter ros2

# 3. Pin NumPy and source ROS environment
/workspace/isaaclab/isaaclab.sh -p -m pip install "numpy==1.26.4"
source /opt/ros/humble/setup.bash
source /root/ament_ws/install/setup.bash

# 4. Launch Isaac Sim teleoperation
cd /workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_isaac_teleop
./run_quest_bimanual_teleop.sh --record
```

> **Expected Output** (takes ~1-2 min to load assets):
> ```text
> [Quest] Simulation ready (lightbox enclosure walls added).
> [Quest] Stereo head-tracked RealSense D455 pair attached.
> [Quest] Ready. Waiting for /quest_teleop messages.
> [Quest][fps] measured POV capture rate: 7.2 fps
> ```
> **Leave Terminal 5 running.**

---

## Terminal 6 (Windows PowerShell): Forward Ports to Meta Quest

In a **PowerShell window on your Windows PC** (where your Quest is plugged in via USB):

```powershell
# 1. Confirm Quest is recognized
adb devices
# (Must say: "device". If unauthorized, put on headset and click "Allow USB Debugging")

# 2. Reverse-forward ports over USB
adb reverse --remove-all
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
```

---

## Step 7: Connect Inside the Meta Quest Headset

1. Put on your **Meta Quest Headset**.
2. Open the **Meta Quest Browser**.
3. **Step A (Trust WebSocket SSL)**:
   - Type in the URL: `https://localhost:9090`
   - Click **Advanced** $\rightarrow$ Click **Proceed to localhost (unsafe)**.  
   *(A blank page or error is normal here)*.
4. **Step B (Open WebXR Teleop App)**:
   - Type in the URL: `https://localhost:8443`
   - Click **Advanced** $\rightarrow$ Click **Proceed to localhost (unsafe)**.
5. Click the **Start** button on the screen!

---

## VR Controls Summary

| Control | Action |
|---|---|
| **Left Hand / Wrist** | Controls Robot Left Arm |
| **Right Hand / Wrist** | Controls Robot Right Arm |
| **Pinch (Thumb + Index)** | Closes the respective hand's gripper |
| **`R` key** (in Isaac Sim terminal) | Recalibrate home pose to current hand positions |
| **`T` key** (in Isaac Sim terminal) | Reset scene (robot, table, and box) |
| **`S` key** (in Isaac Sim terminal) | Save recorded episode to dataset |
| **`D` key** (in Isaac Sim terminal) | Discard recorded episode |
