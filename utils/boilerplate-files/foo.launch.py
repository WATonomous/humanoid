import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Dynamically determine package name and configuration path.

    This function is called by OpaqueFunction to resolve launch arguments
    and create the Node action.
    """
    # These values are resolved from the DeclareLaunchArgument definitions below
    package_name = LaunchConfiguration('package_name').perform(context)
    executable_name = LaunchConfiguration('executable_name').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)

    # Construct the path to the parameter file within the current package
    # This assumes your params.yaml is in a 'config' subdirectory within your package's share directory.
    param_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'  # <-- REPLACE if your parameter file has a different name
    )

    # Check if the params.yaml file actually exists
    node_parameters = []
    if os.path.exists(param_file_path):
        node_parameters.append(param_file_path)
        print(f"INFO: Using parameter file for {node_name}: {param_file_path}")
    else:
        print(
            f"WARNING: No params.yaml found for {node_name} at {param_file_path}. Launching without parameters.")

    # Define the Node action
    node = Node(
        package=package_name,
        executable=executable_name,
        name=node_name,
        parameters=node_parameters,
        output='screen',  # Optional: 'screen' or 'log'. 'screen' prints output to the console.
        emulate_tty=True,  # Optional: Set to True for colored output in the console.
    )

    return [node]


def generate_launch_description():
    """
    Generate the launch description for a generic ROS 2 package.

    This template is designed to be placed in any ROS 2 package's
    launch directory. It expects the package to have a main executable
    and optionally a 'config/params.yaml' file.
    """
    return LaunchDescription([
        # Declare the package name argument.
        # This argument specifies WHICH ROS 2 package this launch file should target.
        # When running, you will set this:
        # e.g., ros2 launch <path_to_this_launch_file> generic_package.launch.py package_name:=your_actual_package_name
        DeclareLaunchArgument(
            'package_name',
            description='Name of the ROS 2 package to launch.'
        ),
        # Declare the executable name argument.
        # This argument specifies WHICH executable within the 'package_name' should be run.
        # When running, you will set this:
        # e.g., ros2 launch ... executable_name:=your_node_executable_name
        DeclareLaunchArgument(
            'executable_name',
            description='Name of the executable to run from the package.'
        ),
        # Declare the node name argument (optional, defaults to executable_name)
        # This argument sets the ROS 2 node name. If not provided, it defaults to the executable name.
        # e.g., ros2 launch ... node_name:=my_custom_node_name
        DeclareLaunchArgument(
            'node_name',
            default_value=LaunchConfiguration('executable_name'),  # Defaults to the executable name
            description='Name to assign to the ROS 2 node.'
        ),
        # OpaqueFunction defers the creation of the Node action until launch arguments are resolved.
        # This is necessary because we need the actual string values of package_name, executable_name, etc.
        OpaqueFunction(function=launch_setup)
    ])
