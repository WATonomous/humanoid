# Teleop
## Samples

rosbridge listener: a node that subscribes to the "teleop" topic and prints all received messages to the console
rosbridge publisher: a node that publishes a VRHandPose message to the "teleop" topic on a timer

running humanoid-server will allow you to listen to, and publish over the "teleop" topic over websocket. You should be able to see messages published from humanoid-server on the ros side, and messages published from ros in humanoid-server.

To run rosbridge listener:
```
ros2 run rosbridge_listener rosbridge_listener_node
```

To run rosbridge publisher:

```
ros2 run rosbridge_publisher rosbridge_publisher_node
```

## Running Rosbridge Server
Rosbridge server will relay ros topics to humanoid-server. To run, use:
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Full Setup for Testing VR -> ROS
These commands were run with the autonomy/samples folder deleted, since some packages may overlap

```bash

# In humanoid repo:
cd autonomy/teleop
colcon build
source install/setup.bash
ros2 run rosbridge_listener rosbridge_listener_node

# Then, in another terminal:
cd autonomy
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# In humanoid-server repo:
npm run dev
```

Then, connect to ws://localhost:3000 (default websocket url) and send an example hand pose message:
```
[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
```
The message should appear in the terminal where you ran the rosbridge listener node

