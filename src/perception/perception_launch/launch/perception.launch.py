# Copyright 2023 WATonomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Load depth estimation parameters
    depth_param_file_path = os.path.join(
        get_package_share_directory('depth_estimation'),
        'config',
        'params.yaml'
    )
    
    # Load pose estimation parameters
    pose_param_file_path = os.path.join(
        get_package_share_directory('pose_estimation'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='depth_estimation',
            name='depth_estimation_node',
            executable='depth_estimation_node',
            parameters=[depth_param_file_path]
        ),
        Node(
            package='pose_estimation',
            name='pose_estimation_node',
            executable='pose_estimation_node',
            parameters=[pose_param_file_path]
        )
    ])
