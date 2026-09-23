import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("whole_body_humanoid")
    urdf = os.path.join(pkg, "urdf", "whole_body_humanoid.urdf")
    with open(urdf) as f:
        robot_description = f.read()
    rviz = os.path.join(pkg, "rviz", "whole_body_humanoid.rviz")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
