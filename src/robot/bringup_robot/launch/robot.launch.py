from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    #################### Costmap Node #####################
    costmap_pkg_prefix = get_package_share_directory('costmap')
    costmap_param_file = os.path.join(
        costmap_pkg_prefix, 'config', 'params.yaml')
    
    costmap_param = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='Path to config file for producer node'
    )
    costmap_node = Node(
        package='costmap',
        name='costmap_node',
        executable='costmap_node',
        parameters=[LaunchConfiguration('costmap_param_file')],
    )
    
    ##################### Nav Node #####################
    nav_pkg_prefix = get_package_share_directory('nav')
    nav_param_file = os.path.join(
        nav_pkg_prefix, 'config', 'params.yaml')
    
    nav_param = DeclareLaunchArgument(
        'nav_param_file',
        default_value=nav_param_file,
        description='Path to config file for producer node'
    )
    nav_node = Node(
        package='nav',
        name='nav_node',
        executable='nav_node',
        parameters=[LaunchConfiguration('nav_param_file')],
    )
    
    ##################### Control Node #####################
    control_pkg_prefix = get_package_share_directory('control')
    control_param_file = os.path.join(
        control_pkg_prefix, 'config', 'params.yaml')
    
    control_param = DeclareLaunchArgument(
        'control_param_file',
        default_value=control_param_file,
        description='Path to config file for producer node'
    )
    control_node = Node(
        package='control',
        name='control_node',
        executable='control_node',
        parameters=[LaunchConfiguration('control_param_file')],
    )

    ##################### Add to Launch Description #####################
    ld = LaunchDescription()

    ld.add_action(costmap_param)
    ld.add_action(nav_param)
    ld.add_action(control_param)

    ld.add_action(costmap_node)
    ld.add_action(nav_node)
    ld.add_action(control_node)

    return ld
