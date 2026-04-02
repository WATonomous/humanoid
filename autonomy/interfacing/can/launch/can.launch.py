from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('can'),
        'config',
        'params.yaml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the CAN node configuration YAML file'
    )

    # Create the CAN node
    can_node = Node(
        package='can',
        executable='can_node',
        name='can_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # ============================================================================
    # TEST CONTROLLER NODE (for development/testing only)
    # Comment out the following section if you want to disable the test controller
    # ============================================================================
    test_controller_node = Node(
        package='can',
        executable='test_controller_node',
        name='test_controller',
        output='screen'
    )
    # ============================================================================
    # End of test controller section
    # ============================================================================

    # Return the launch description
    return LaunchDescription([
        config_file_arg,
        can_node,
        test_controller_node  # Comment out this line to disable test controller
    ])
