#!/bin/bash
set -e

# setup ROS2 environment
source /opt/ros/humble/setup.bash

exec "$@"