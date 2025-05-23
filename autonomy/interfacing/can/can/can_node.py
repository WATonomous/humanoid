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
from can.can_core import CanCore


class CanNode(Node):
    def __init__(self):
        print("test")
    def publish_position(self):
        print("test")

def main(args=None):
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
