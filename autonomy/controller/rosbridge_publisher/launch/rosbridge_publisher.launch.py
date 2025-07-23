from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch costmap node."""
    costmap_node = Node(
        package='rosbridge_publisher',
        executable='rosbridge_publisher',
    )

    return LaunchDescription([
        costmap_node
    ])
