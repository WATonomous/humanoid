# Running SLAM with RealSense D455 (ROS2 Jazzy)

## Prerequisites (one-time setup)

```bash
sudo apt install ros-jazzy-imu-filter-madgwick ros-jazzy-rtabmap-ros ros-jazzy-robot-localization
```

Use a USB 3.0 / USB-C port, USB 2.1 can cause issues with performance and dropouts.

---

## Running it
You will need 3 terminals to run the 3 seperate nodes.

### Terminal 1: Camera

```bash
source /opt/ros/jazzy/setup.bash && ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1
```

### Terminal 2: IMU filter

```bash
source /opt/ros/jazzy/setup.bash && ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false --remap /imu/data_raw:=/camera/camera/imu --remap /imu/data:=/imu/data_filtered
```

### Terminal 3: SLAM (RTAB-Map)

```bash
source /opt/ros/jazzy/setup.bash && ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info frame_id:=camera_link approx_sync:=true rgbd_sync:=true imu_topic:=/imu/data_filtered wait_imu_to_init:=true rtabmap_viz:=false rviz:=true odom_args:="--Vis/MinInliers 10 --Vis/MaxFeatures 1500 --Odom/ResetCountdown 5"
```

Move the camera slowly around a textured environment (avoid blank walls/ceilings) and the map builds in RViz.

---

## Viewing the map

2D occupancy grid in RViz: click **Add**, **Map**, topic `/rtabmap/map`

View a saved map later without running SLAM:
```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

## Starting a fresh map

```bash
rm ~/.ros/rtabmap.db
```
Or add `args:="-d"` to the Terminal 3 command to always start fresh.

---

## Troubleshooting

**`Device or resource busy` when launching the camera:**

Something else has a lock on the device (leftover process, or camera wasn't released cleanly last time).
```bash
ps aux | grep realsense
killall realsense2_camera_node
```
Then unplug/replug the camera and relaunch.

**Terminal 3 repeats `Not enough inliers X/20 (matches=Y)` and mapping stalls:**

This is visual odometry (Terminal 3) failing to match enough features between the current and previous camera frame, so it can't compute motion and causes mapping to freeze. Usually happens when pointing at blank surface or low texture surface.
Work needs to continue on making system more robust to prevent this, in practice currently can try moving slower to prevent this issue.

**Mapping freezes:**

Check what's actually still running or if anything has failed:
```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/imu
ros2 topic hz /rtabmap/odom
```
Each command shows how many messages per second are coming through on that topic. If color/image_raw drops to zero, it's a camera or USB dropout. Check the cable and port, switch to USB 3.0, replug the camera.

If imu drops to 0 but color is still updating fine, the IMU stream has failed. Restart Terminal 2, the imu_filter_madgwick node.

If both are still updating at a normal rate but rtabmap/odom stalls, that's visual tracking loss described above.



## Pipeline overview

```
RealSense camera: raw color/depth + raw gyro/accel
       ↓
Madgwick filter: takes gyro+accel to output orientation quaternion
       ↓
RTAB-Map: visual odometry (motion tracking) + mapping (loop closures, 3D/2D map)
```