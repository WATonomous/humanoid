from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            name='octo_map_node',
            package='octo_map',
            executable='octo_map_node',
            output='screen',
            emulate_tty=True),
    ])
