#!/usr/bin/env python3
"""Continuously print real motor feedback (position/angle) to the console.

can_node already parses incoming CAN frames and publishes a MotorFeedback message on
/interfacing/motorFeedback every time a motor sends its ServoStatusFeedback frame. This
script does NOT talk to CAN directly -- it just subscribes to what can_node publishes
and prints it, so can_node must already be running and connected to the real bus
(ros2 launch can can.launch.py).

Usage:
  /usr/bin/python3 monitor_motor_feedback.py
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorFeedback


def find_hardware_mapping():
    local = Path(__file__).resolve().parent.parent / "config" / "hardware_mapping.yaml"
    if local.exists():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("joint_command")) / "config" / "hardware_mapping.yaml"
        if installed.exists():
            return installed
    except Exception:
        pass
    return None


def build_can_id_to_label():
    """Best-effort reverse lookup can_id -> joint label (e.g. 'shoulder_roll'), read
    fresh from hardware_mapping.yaml so it stays in sync with any edits. Falls back to
    showing just the raw CAN ID for anything not found (e.g. hand placeholders)."""
    path = find_hardware_mapping()
    if path is None:
        return {}
    import yaml
    with open(path) as f:
        config = yaml.safe_load(f)

    labels = {}

    def walk(node, path_parts):
        if isinstance(node, dict):
            if "can_id" in node:
                labels[node["can_id"]] = "_".join(path_parts)
            else:
                for k, v in node.items():
                    walk(v, path_parts + [k])

    walk(config.get("left", {}), [])
    return labels


class MotorFeedbackMonitor(Node):
    def __init__(self, can_id_to_label):
        super().__init__("motor_feedback_monitor")
        self.can_id_to_label = can_id_to_label
        self.create_subscription(MotorFeedback, "/interfacing/motorFeedback", self.on_feedback, 10)
        self.get_logger().info("Listening on /interfacing/motorFeedback ...")

    def on_feedback(self, msg: MotorFeedback):
        label = self.can_id_to_label.get(msg.motor_id, f"0x{msg.motor_id:02X}")
        print(f"[{label:16s}] can_id=0x{msg.motor_id:02X}  position={msg.position:+8.3f}  "
              f"velocity={msg.velocity:+8.3f}  current={msg.current:+7.3f}  "
              f"temp={msg.temperature:3d}  err={msg.error_code}")


def main():
    rclpy.init()
    can_id_to_label = build_can_id_to_label()
    node = MotorFeedbackMonitor(can_id_to_label)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
