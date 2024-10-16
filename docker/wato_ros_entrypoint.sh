#!/bin/bash
set -e

# setup ROS2 environment
source /opt/watonomous/setup.bash
exec "$@"
