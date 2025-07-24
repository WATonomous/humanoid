# Teleop
## Samples
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
cd autonomy
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

