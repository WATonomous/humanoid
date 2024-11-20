from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    ld = LaunchDescription() # Begin building a launch description

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
    ld.add_action(costmap_param)
    ld.add_action(costmap_node)

    # #################### Map Memory Node #####################
    # map_memory_pkg_prefix = get_package_share_directory('map_memory')
    # map_memory_param_file = os.path.join(
    #     map_memory_pkg_prefix, 'config', 'params.yaml')
    
    # map_memory_param = DeclareLaunchArgument(
    #     'map_memory_param_file',
    #     default_value=map_memory_param_file,
    #     description='Path to config file for producer node'
    # )
    # map_memory_node = Node(
    #     package='map_memory',
    #     name='map_memory_node',
    #     executable='map_memory_node',
    #     parameters=[LaunchConfiguration('map_memory_param_file')],
    # )
    # ld.add_action(map_memory_param)
    # ld.add_action(map_memory_node)

    # #################### State Machine Node #####################
    # state_machine_pkg_prefix = get_package_share_directory('state_machine')
    # state_machine_param_file = os.path.join(
    #     state_machine_pkg_prefix, 'config', 'params.yaml')
    
    # state_machine_param = DeclareLaunchArgument(
    #     'state_machine_param_file',
    #     default_value=state_machine_param_file,
    #     description='Path to config file for producer node'
    # )
    # state_machine_node = Node(
    #     package='state_machine',
    #     name='state_machine_node',
    #     executable='state_machine_node',
    #     parameters=[LaunchConfiguration('state_machine_param_file')],
    # )
    # ld.add_action(state_machine_param)
    # ld.add_action(state_machine_node)
    
    # ##################### Planner Node #####################
    # planner_pkg_prefix = get_package_share_directory('planner')
    # planner_param_file = os.path.join(
    #     planner_pkg_prefix, 'config', 'params.yaml')
    
    # planner_param = DeclareLaunchArgument(
    #     'planner_param_file',
    #     default_value=planner_param_file,
    #     description='Path to config file for producer node'
    # )
    # planner_node = Node(
    #     package='planner',
    #     name='planner_node',
    #     executable='planner_node',
    #     parameters=[LaunchConfiguration('planner_param_file')],
    # )
    # ld.add_action(planner_param)
    # ld.add_action(planner_node)
    
    # ##################### Control Node #####################
    # control_pkg_prefix = get_package_share_directory('control')
    # control_param_file = os.path.join(
    #     control_pkg_prefix, 'config', 'params.yaml')
    
    # control_param = DeclareLaunchArgument(
    #     'control_param_file',
    #     default_value=control_param_file,
    #     description='Path to config file for producer node'
    # )
    # control_node = Node(
    #     package='control',
    #     name='control_node',
    #     executable='control_node',
    #     parameters=[LaunchConfiguration('control_param_file')],
    # )
    # ld.add_action(control_param)
    # ld.add_action(control_node)

    return ld
