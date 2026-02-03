from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the dummy publisher node'
        ),
        
        Node(
            package='perception',
            executable='dummy_publisher_node',
            name='dummy_publisher_node',
            output='screen',
            parameters=[
                {'log_level': LaunchConfiguration('log_level')}
            ],
            remappings=[
                # Add any topic remappings here if needed
            ]
        )
    ])