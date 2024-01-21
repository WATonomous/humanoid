#!/bin/bash
set -e

# setup ROS2 environment
source /root/ament_ws/install/setup.bash

exec "$@"