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

import time

import rclpy
from rclpy.node import Node

from sample_msgs.msg import Unfiltered
from depth_estimation.depth_estimation_core import DepthEstimationCore


class DepthEstimationNode(Node):

    def __init__(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    return

if __name__ == '__main__':
    main()
