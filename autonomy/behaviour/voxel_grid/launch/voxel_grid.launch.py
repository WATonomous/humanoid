from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            name='voxel_grid_node',
            package='voxel_grid',
            executable='voxel_grid_node',
            output='screen',
            emulate_tty=True),
    ])