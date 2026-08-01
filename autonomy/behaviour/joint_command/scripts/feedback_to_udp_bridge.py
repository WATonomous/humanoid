#!/usr/bin/env python3
"""ROS2-to-UDP bridge for live_arm_isaacsim.py.

live_arm_isaacsim.py runs inside the env_isaaclab conda environment, where rclpy can't
be imported (its compiled extension is built for the system ROS Python, not conda's --
see udp_to_ros_bridge.py for the same constraint in the opposite direction). This script
runs under the SYSTEM python (where rclpy works), subscribes to /interfacing/motorFeedback,
and forwards each message as a UDP packet the Isaac Sim viewer can read without rclpy.

Packet format (matches live_arm_isaacsim.py's run_simulator unpack):
  struct.pack("=id", motor_id, position)  -- native int32 motor_id + native double position

Usage (system python, ROS sourced):
  source /opt/ros/jazzy/setup.bash
  source /home/rwahib/wato/humanoid/autonomy/install/setup.bash
  /usr/bin/python3 feedback_to_udp_bridge.py [--host 127.0.0.1] [--port 5006]
"""

import argparse
import socket
import struct

import rclpy
from rclpy.node import Node

from common_msgs.msg import MotorFeedback

PACKET_FORMAT = "=id"


class UdpBridge(Node):
    def __init__(self, host, port):
        super().__init__("feedback_to_udp_bridge")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (host, port)
        self.create_subscription(MotorFeedback, "/interfacing/motorFeedback", self._on_feedback, 20)
        self.get_logger().info(f"Forwarding /interfacing/motorFeedback to UDP {host}:{port}")

    def _on_feedback(self, msg: MotorFeedback) -> None:
        packet = struct.pack(PACKET_FORMAT, int(msg.motor_id), float(msg.position))
        self.sock.sendto(packet, self.dest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5006)
    args = parser.parse_args()

    rclpy.init()
    node = UdpBridge(args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
