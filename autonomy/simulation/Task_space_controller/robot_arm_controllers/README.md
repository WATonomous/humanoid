# Deployment Pipeline: `task_space_real.py`

## Prerequisites

- ROS2 workspace built: `cd autonomy && colcon build`
- CAN-USB adapter connected (`/dev/canable`)
- Isaac Lab / Isaac Sim conda environment available (`env_isaaclab` below)

## Powering up the arm

1. Turn on the e-stop switch as well as the small power supply at the left end of the desk.
2. Get a USB-C to USB-C cable. Plug one end into the CANable and the other into your laptop.

## Terminal 1 -- CAN node

```bash
source /opt/ros/jazzy/setup.bash
source /home/rwahib/wato/humanoid/autonomy/install/setup.bash
ros2 launch can can.launch.py
```

## Terminal 2 -- joint_command node

```bash
source /opt/ros/jazzy/setup.bash
source /home/rwahib/wato/humanoid/autonomy/install/setup.bash
ros2 launch joint_command joint_command.launch.py
```

## Terminal 3 -- UDP-to-ROS2 bridge

```bash
source /opt/ros/jazzy/setup.bash
source /home/rwahib/wato/humanoid/autonomy/install/setup.bash
/usr/bin/python3 autonomy/behaviour/joint_command/scripts/ros_bridge.py
```

## Terminal 4 -- task_space_real.py

```bash
conda activate env_isaaclab
cd /home/rwahib/wato/humanoid
python autonomy/simulation/Task_space_controller/robot_arm_controllers/task_space_real.py --udp
```

Drag the cube in the viewport to move the arm. There's a 5-second delay after startup
before anything is sent to real hardware.
