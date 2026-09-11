from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('joint_command'),
        'config',
        'joint_command.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the joint command node configuration YAML file',
        ),
        Node(
            package='joint_command',
            executable='joint_command_node',
            name='joint_command_node',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
