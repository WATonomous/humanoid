from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_example',
            executable='rosbridge_listener_node',
        ),
        Node(
            package='rosbridge_example',
            executable='rosbridge_publisher_node',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0'
            }]
        )
    ])
