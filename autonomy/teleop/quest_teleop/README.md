# Quest Teleop Controller

This package starts the Quest WebXR controller bridge. The Quest browser sends
hand poses over WSS to `quest_teleop_node`, which publishes:

```text
/quest_teleop
```

For the full Quest + Isaac Sim workflow, prefer
[quest_isaac_teleop/README.md](../../simulation/quest_isaac_teleop/README.md)
(`ACTIVE_MODULES="simulation_isaac"`, shell into `simulation_isaac_dev`).

## Start the Controller (standalone)

### 1. Start the Isaac Lab container

```bash
# watod-config.local.sh
ACTIVE_MODULES="simulation_isaac"
MODE_OF_OPERATION="develop"

./watod up -d
./watod -t simulation_isaac_dev
```

### 2. Build and source the workspace

Inside the container (`.bashrc` may already colcon-build on entry):

```bash
cd /root/ament_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select common_msgs quest_teleop
source install/setup.bash
```

### 3. Start the ROS controller node

```bash
ros2 run quest_teleop quest_teleop_node
```

This starts the secure WebSocket server on port `9090`.

### 4. Start the Quest WebXR page

From a second host terminal:

```bash
./watod -t simulation_isaac_dev
python3 /workspace/humanoid/autonomy/teleop/quest_teleop/scripts/webxr_server.py
```

This serves the Quest page on port `8443`.

### 5. Connect the Quest

On the host machine, with the Quest plugged in over USB:

```bash
adb devices
adb reverse tcp:8443 tcp:8443
adb reverse tcp:9090 tcp:9090
adb reverse --list
```

Expected entries:

```text
tcp:8443 tcp:8443
tcp:9090 tcp:9090
```

Open this page in the Quest browser:

```text
https://localhost:8443/
```

Accept the self-signed certificate warning if prompted.

## Verify

In another container shell:

```bash
source /root/ament_ws/install/setup.bash
ros2 topic info /quest_teleop
ros2 topic echo /quest_teleop
```

Expected topic type:

```text
common_msgs/msg/QuestHandPose
```

## Regenerate Certificates

Only do this if the certs are missing or stale:

```bash
mkdir -p autonomy/teleop/quest_teleop/certs
cd autonomy/teleop/quest_teleop/certs
openssl genrsa -out key.pem 2048
openssl req -new -x509 -key key.pem -out cert.pem -days 365 \
  -subj "/CN=localhost"
```
