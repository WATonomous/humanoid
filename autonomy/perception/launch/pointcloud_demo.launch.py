#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch the dummy publisher (publishes depth images and camera info)
        Node(
            package='perception',
            executable='dummy_publisher_node',
            name='dummy_publisher',
            output='screen',
            parameters=[],
        ),
        
        # Launch the point cloud publisher (converts depth to point cloud)
        Node(
            package='perception',
            executable='pc_publisher_node',
            name='pointcloud_publisher', 
            output='screen',
            parameters=[],
        ),
    ])