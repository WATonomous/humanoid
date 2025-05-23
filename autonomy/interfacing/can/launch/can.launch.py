from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz')
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        can_interface_arg,
        publish_rate_arg,
        can_node
    ])
