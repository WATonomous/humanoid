from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch rosbridge listener node."""
    rosbridge_listener_node = Node(
        package='rosbridge_listener',
        executable='rosbridge_listener',
    )

    return LaunchDescription([
        rosbridge_listener_node
    ])
