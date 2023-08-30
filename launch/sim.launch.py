import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    gz_sim = ExecuteProcess(cmd=['ign', 'launch', '-v 4', 'launch/sim.ign'])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/model/robot/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])