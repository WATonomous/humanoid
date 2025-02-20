from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch costmap node."""
    costmap_node = Node(
        package='costmap',
        executable='costmap_node',
    )

    return LaunchDescription([
        costmap_node
    ])
