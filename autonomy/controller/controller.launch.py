from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch rosbridge listener and publisher nodes."""
    rosbridge_listener_node = Node(
        package='rosbridge_listener',
        executable='rosbridge_listener',
    )

    rosbridge_publisher_node = Node(
        package='rosbridge_publisher',
        executable='rosbridge_publisher',
    )

    return LaunchDescription([
        rosbridge_listener_node,
        rosbridge_publisher_node
    ]) 