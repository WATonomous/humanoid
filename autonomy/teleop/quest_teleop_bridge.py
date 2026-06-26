#!/usr/bin/env python3
"""Forward /quest_teleop ROS2 messages as JSON UDP packets.

Run this inside the *teleop* container alongside quest_teleop_node.
The simulation_il container listens on the same localhost port (both containers
share the host network so localhost is the same interface).

Usage (inside teleop container):
    source /root/ament_ws/install/setup.bash
    python3 /workspace/humanoid/autonomy/simulation/quest_isaac_teleop/quest_teleop_bridge.py
"""

import json
import socket

import rclpy
from rclpy.node import Node
from common_msgs.msg import QuestHandPose

DEST_HOST = "127.0.0.1"
DEST_PORT = 19090


class QuestTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__("quest_teleop_bridge")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(QuestHandPose, "/quest_teleop", self._cb, 1)
        self.get_logger().info(f"Bridging /quest_teleop → UDP {DEST_HOST}:{DEST_PORT}")

    def _cb(self, msg: QuestHandPose) -> None:
        lw, rw = msg.left_wrist, msg.right_wrist
        payload = {
            "left_wrist": {
                "position": {"x": lw.position.x, "y": lw.position.y, "z": lw.position.z},
                "orientation": {"x": lw.orientation.x, "y": lw.orientation.y,
                                "z": lw.orientation.z, "w": lw.orientation.w},
            },
            "right_wrist": {
                "position": {"x": rw.position.x, "y": rw.position.y, "z": rw.position.z},
                "orientation": {"x": rw.orientation.x, "y": rw.orientation.y,
                                "z": rw.orientation.z, "w": rw.orientation.w},
            },
            "left_hand_joints": list(msg.left_hand_joints),
            "right_hand_joints": list(msg.right_hand_joints),
        }
        data = json.dumps(payload).encode()
        self._sock.sendto(data, (DEST_HOST, DEST_PORT))


def main() -> None:
    rclpy.init()
    node = QuestTeleopBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
