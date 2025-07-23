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
make sure to
```
source install/setup.bash
```
any packages containing messages etc. that you need 

