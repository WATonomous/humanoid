# Camera-Based Teleoperation — Launch Guide

> [!NOTE]
> **This folder is location-independent.** Every script resolves its own file and model paths
> dynamically based on where it lives in the repository — no hardcoded absolute paths.
> You can clone or move the `humanoid` repository to any directory on your host machine or
> inside the Docker container and everything will still work, as long as the internal
> folder structure of the repo is unchanged.

## System Architecture

```
┌─────────────────── Windows PC ──────────────────────┐
│  hand_recorder.py                                    │
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

### Build colcon workspace (ROS2 bridge)
```bash
mkdir -p final_repo/humanoid/autonomy/teleop/colcon_teleop_ws/src
cd final_repo/humanoid/autonomy/teleop/colcon_teleop_ws/src

ln -s /workspace/isaaclab/final_repo/humanoid/autonomy/wato_msgs/sample_msgs sample_msgs
ln -s /workspace/isaaclab/final_repo/humanoid/autonomy/teleop/rosbridge_example rosbridge_example

cd ..
colcon build
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
source /workspace/isaaclab/final_repo/humanoid/autonomy/teleop/colcon_teleop_ws/install/setup.bash
ros2 launch rosbridge_example rosbridge_example.launch.py
```

### Terminal 2 — ROS2 landmark-to-joints node
```bash
source /opt/ros/humble/setup.bash
cd "/workspace/isaaclab/final_repo/humanoid/autonomy/simulation/Teleop/camera-based teleoperation"
python wato_hand_ros2_node.py
```

### Terminal 3 — Isaac Lab simulation
```bash
source /opt/ros/humble/setup.bash
cd "/workspace/isaaclab/final_repo/humanoid/autonomy/simulation/Teleop/camera-based teleoperation"
/workspace/isaaclab/isaaclab.sh -p wato_hand_isaaclab_teleop.py
```

> **Note:** No `PYTHONPATH` export required — `arm_cfg.py` is local to this folder,
> and `fingertip_ik2.py` (in `camera-based arm/`) is loaded via `sys.path` at runtime.

### Windows PC — webcam publisher
```powershell
# PowerShell — activate venv from root first, then cd
.\myenv\Scripts\Activate.ps1
cd "autonomy\simulation\Teleop\camera-based teleoperation"
pip install mediapipe==0.10.35 roslibpy opencv-python
python hand_recorder.py
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
| `hand_recorder.py` | Windows (webcam host) | Reads webcam via MediaPipe, publishes landmarks over rosbridge |
| `wato_hand_ros2_node.py` | Docker (ROS2) | Converts landmarks → joint angles, writes `wato_joints.json` to temp dir |
| `wato_hand_isaaclab_teleop.py` | Docker (Isaac Lab Python) | Reads JSON, drives arm+hand sim via DexRetargeting (or `fingertip_ik2` fallback) |
| `arm_cfg.py` | Docker (local import) | `ARM_CFG` articulation config — local copy, no PYTHONPATH needed |

### `utils/` — diagnostic / test scripts

| File | Where it runs | Purpose |
|------|--------------|---------|
| `utils/test_camera.py` | Windows (webcam host) | Simplified webcam + rosbridge connectivity test |
| `utils/camera_messages.py` | Windows (webcam host) | Streams raw webcam frames over rosbridge to Docker |
| `utils/websocket_test.py` | Windows | Raw WebSocket ping to confirm SSH tunnel is active |

### `camera-based arm/` — URDF and IK support files

| File | Purpose |
|------|---------|
| `camera-based arm/arm_assembly_fixed.urdf` | URDF with fixed joint limits used by DexRetargeting IK solver |
| `camera-based arm/simple_door.urdf` | Optional door articulation for the sim scene (currently disabled) |
| `camera-based arm/fingertip_ik2.py` | MuJoCo gradient-based IK fallback when `dex-retargeting` is unavailable |

---

## Debugging

| Problem | Fix |
|---------|-----|
| `arm_cfg` import error | Check you are running from `camera-based teleoperation/` |
| `arm_assembly_fixed.urdf not found` | Verify `camera-based arm/arm_assembly_fixed.urdf` exists in this folder |
| rosbridge connection refused | Check Terminal 1 is running and SSH tunnel is active (`-L 9090:localhost:9090`) |
| Hand landmarks not appearing | Run `ros2 topic echo /wato/hand_landmarks` to confirm `hand_recorder.py` is publishing |
| Enable curl debug output | Set `DEBUG_CURL = True` in `wato_hand_ros2_node.py` — outputs to temp dir `curl_debug.txt` |
