import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("tracknetv3"),
        "config",
        "tracknetv3_config.yaml",
    )

    model_dir = LaunchConfiguration("model_dir")
    tracknet_checkpoint = LaunchConfiguration("tracknet_checkpoint")
    inpaintnet_checkpoint = LaunchConfiguration("inpaintnet_checkpoint")
    device = LaunchConfiguration("device")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    enable_debug = LaunchConfiguration("enable_debug")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_dir",
            default_value="",
            description="Directory containing TrackNetV3 checkpoint files",
        ),
        DeclareLaunchArgument(
            "tracknet_checkpoint",
            default_value="TrackNet_best.pt",
            description="TrackNet checkpoint filename or absolute path",
        ),
        DeclareLaunchArgument(
            "inpaintnet_checkpoint",
            default_value="",
            description="Optional InpaintNet checkpoint filename or absolute path",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cuda:0",
            description="Torch device, for example cuda:0 or cpu",
        ),
        DeclareLaunchArgument(
            "confidence_threshold",
            default_value="0.5",
            description="Minimum heatmap confidence for a visible shuttle point",
        ),
        DeclareLaunchArgument(
            "enable_debug",
            default_value="false",
            description="Publish annotated debug images",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        Node(
            package="tracknetv3",
            executable="tracknetv3_node",
            name="tracknetv3_node",
            output="screen",
            parameters=[
                config_path,
                {
                    "model.model_dir": model_dir,
                    "model.tracknet_checkpoint": tracknet_checkpoint,
                    "model.inpaintnet_checkpoint": inpaintnet_checkpoint,
                    "model.device": device,
                    "inference.confidence_threshold": ParameterValue(
                        confidence_threshold,
                        value_type=float,
                    ),
                    "debug.enabled": ParameterValue(enable_debug, value_type=bool),
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
