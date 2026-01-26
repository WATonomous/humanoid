#!/bin/bash

# Create shared ROS network for container communication
docker network create ros_network 2>/dev/null || echo "Network ros_network already exists"

echo "ROS2 network setup complete!"
echo "Containers can now communicate via the 'ros_network' bridge network"