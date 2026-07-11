# Camera-Based Teleoperation — Launch Guide

## System Architecture

```
┌─────────────────── Windows PC ──────────────────────┐
│  hand_landmark_publisher.py                          │
│  (webcam + MediaPipe → rosbridge → Docker)          │
└───────────────────────┬─────────────────────────────┘
                        │  rosbridge ws://localhost:9090
                        │  SSH tunnel: -L 9090:localhost:9090
┌─────────────────── Docker Container ────────────────┐
│  Terminal 1: rosbridge server                        │
│  Terminal 2: wato_hand_ros2_node.py                 │
│              (/wato/hand_landmarks → joint angles)  │
│              → writes /tmp/wato_joints.json         │
│  Terminal 3: wato_hand_isaaclab_teleop.py           │
│              (reads JSON → drives Isaac Lab sim)    │
└─────────────────────────────────────────────────────┘
```

**New repo path (Docker):** `/workspace/isaaclab/final_repo/humanoid`

---

## One-Time Setup

### SSH + Port Forwarding
```bash
ssh -L 5900:localhost:5900 -L 9090:localhost:9090 asd-dev-session
```

### Pull the repo
```bash
source /opt/ros/humble/setup.bash
cd /workspace/isaaclab/final_repo/humanoid
git pull origin rijul
cd /workspace/isaaclab
```

### Install ROS2 rosbridge
```bash
apt-get update && apt-get install -y ros-humble-rosbridge-server
```

### Install Python packages for Isaac Lab
```bash
/workspace/isaaclab/isaaclab.sh -p -m pip install setuptools==69.5.1
/workspace/isaaclab/isaaclab.sh -p -m pip install dex-retargeting
/workspace/isaaclab/isaaclab.sh -p -m pip install "numpy<2.0.0"
```

### Fix Isaac Sim RTX driver version check (one-time)
```bash
echo 'rtx.verifyDriverVersion.enabled = false' >> /workspace/isaaclab/apps/isaaclab.python.rendering.kit
echo 'rtx.verifyDriverVersion.enabled = false' >> /workspace/isaaclab/apps/isaaclab.python.kit
python3 -c 'import json; p="/isaac-sim/kit/data/Kit/Isaac-Sim/4.5/user.config.json"; d=json.load(open(p)); d.setdefault("rtx", {})["verifyDriverVersion"] = {"enabled": False}; json.dump(d, open(p, "w"), indent=4)'
```

---

## Launch Order (Every Session)

### Terminal 1 — rosbridge server
```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Terminal 2 — ROS2 landmark-to-joints node
```bash
source /opt/ros/humble/setup.bash
cd "/workspace/isaaclab/final_repo/humanoid/autonomy/simulation/Teleop/camera_based_teleoperation"
python wato_hand_ros2_node.py
```

### Terminal 3 — Isaac Lab simulation
```bash
source /opt/ros/humble/setup.bash
cd "/workspace/isaaclab/final_repo/humanoid/autonomy/simulation/Teleop/camera_based_teleoperation"
/workspace/isaaclab/isaaclab.sh -p wato_hand_isaaclab_teleop.py
```

> **Note:** No `PYTHONPATH` export required — `teleop_tuned_arm_cfg.py` is local to this folder.

### Windows PC — webcam publisher
```powershell
# PowerShell — activate venv from root first, then cd
.\myenv\Scripts\Activate.ps1
cd "autonomy\simulation\Teleop\camera_based_teleoperation"
pip install mediapipe==0.10.35 roslibpy opencv-python
python hand_landmark_publisher.py
```

---

## Monitoring (Optional)

```bash
# In any Docker terminal with ROS2 sourced:
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic echo /wato/fist_state
ros2 topic echo /wato/hand_landmarks
ros2 topic echo /wato/hand_joint_angles
```

---

## File Overview

### Main scripts (run directly)

| File | Where it runs | Purpose |
|------|--------------|---------|
| `hand_landmark_publisher.py` | Windows (webcam host) | Reads webcam via MediaPipe, publishes landmarks over rosbridge |
| `wato_hand_ros2_node.py` | Docker (ROS2) | Converts landmarks → joint angles, writes `wato_joints.json` to temp dir |
| `wato_hand_isaaclab_teleop.py` | Docker (Isaac Lab Python) | Reads JSON, drives arm+hand sim via DexRetargeting |
| `teleop_tuned_arm_cfg.py` | Docker (local import) | `TELEOP_TUNED_ARM_CFG` — few-line overrides on `humanoid_arm_hand.ARM_CFG` |

### `utils/` — diagnostic / test scripts

| File | Where it runs | Purpose |
|------|--------------|---------|
| `utils/test_camera.py` | Windows (webcam host) | Simplified webcam + rosbridge connectivity test |
| `utils/camera_messages.py` | Windows (webcam host) | Streams raw webcam frames over rosbridge to Docker |
| `utils/websocket_test.py` | Windows | Raw WebSocket ping to confirm SSH tunnel is active |

### `camera-based arm/` — URDF and IK support files

| File | Purpose |
|------|---------|
| `Humanoid_Wato/arm_assembly/right_arm_assembly.urdf` | URDF used by DexRetargeting |
| `camera-based arm/simple_door.urdf` | Optional door articulation for the sim scene (currently disabled) |

---

## Debugging

| Problem | Fix |
|---------|-----|
| `teleop_tuned_arm_cfg` import error | Check you are running from `camera_based_teleoperation/` |
| `right_arm_assembly.urdf not found` | Verify `Humanoid_Wato/arm_assembly/right_arm_assembly.urdf` exists |
| rosbridge connection refused | Check Terminal 1 is running and SSH tunnel is active (`-L 9090:localhost:9090`) |
| Hand landmarks not appearing | Run `ros2 topic echo /wato/hand_landmarks` to confirm `hand_landmark_publisher.py` is publishing |
| Enable curl debug output | Set `DEBUG_CURL = True` in `wato_hand_ros2_node.py` — outputs to temp dir `curl_debug.txt` |
