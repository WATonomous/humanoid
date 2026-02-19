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

    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='Name of the CAN interface to use (e.g., can0)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50',
        description='Rate in Hz at which to check for CAN messages'
    )

    # Create the CAN node
    can_node = Node(
        package='can',
        executable='can_node',
        name='can_node',
        parameters=[config_file, {
            'can_interface': LaunchConfiguration('can_interface'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz')
        }],
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
        can_interface_arg,
        publish_rate_arg,
        can_node,
        test_controller_node  # Comment out this line to disable test controller
    ])
