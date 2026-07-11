#!/usr/bin/env python3
"""UDP-to-ROS2 bridge for task_space_real.py's --udp mode.

task_space_real.py runs inside the env_isaaclab conda environment, where rclpy can't
be imported (its compiled extension is built for the system ROS Python, not conda's
Python). So when --udp is passed instead of --ros, task_space_real.py sends its 6
left-arm joint angles (degrees) as a UDP packet instead of publishing directly. This
script runs under the SYSTEM python (where rclpy works) and does that publishing on
its behalf.

Packet format (matches task_space_real.py's publish_joint_pos_udp):
  struct.pack("6d", *degrees)  -- 6 native-endian doubles:
  [shoulder_flexion, shoulder_abduction, shoulder_rotation,
   elbow_flexion, forearm_rotation, wrist_extension]

Two modes:
  Default: republishes each packet as an ArmPose message on /behaviour/arm_pose, which
    joint_command_node consumes -- joint_command_node always derives a command for all
    6 arm joints from every ArmPose it receives, so this mode moves the whole arm.

  --only-joint <label>: bypasses joint_command_node entirely and publishes a raw
    MotorCmd straight to /interfacing/motorCMD for ONLY that one joint's CAN ID (read
    from hardware_mapping.yaml). No other joint is ever sent a command by this bridge,
    so nothing else can move as a side effect. Applies the same position clamp and a
    simple delta-limit itself, since joint_command_node's own safety ramping is
    bypassed along with everything else.

Usage:
  /usr/bin/python3 ros_bridge.py [--host 127.0.0.1] [--port 5005]
  /usr/bin/python3 ros_bridge.py --only-joint shoulder_roll
"""

import argparse
import socket
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from common_msgs.msg import ArmPose, JointState, MotorCmd

PACKET_FORMAT = "6d"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# hardware_mapping.yaml label -> index in the 6-value packet, matching
# task_space_real.py's publish_joint_pos_udp ordering.
JOINT_INDEX = {
    "shoulder_pitch": 0,
    "shoulder_roll": 1,
    "shoulder_yaw": 2,
    "elbow_pitch": 3,
    "elbow_roll": 4,
    "wrist_pitch": 5,
}

# Conservative per-publish delta cap (degrees) used only in --only-joint mode, since
# joint_command_node's own velocity/delta ramping is bypassed entirely in that mode.
DELTA_MAX_DEG = 2.0


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
    raise FileNotFoundError("Could not locate hardware_mapping.yaml")


def load_joint_config(label, arm_side="left"):
    import yaml

    group, name = label.split("_", 1)
    path = find_hardware_mapping()
    with open(path) as f:
        config = yaml.safe_load(f)
    node = config[arm_side][group][name]
    return {
        "can_id": node["can_id"],
        "lower_limit": float(node["lower_limit"]),
        "upper_limit": float(node["upper_limit"]),
        "direction": int(node["direction"]),
        "zero_offset": float(node["zero_offset"]),
        "limit_range": bool(node["limit_range"]),
    }


class RosBridge(Node):
    """Default mode: full arm, via ArmPose -> joint_command_node."""

    def __init__(self, host, port):
        super().__init__("ros_bridge")
        self.pub = self.create_publisher(ArmPose, "/behaviour/arm_pose", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.1)
        self.get_logger().info(f"Listening for UDP joint packets on {host}:{port}")

    def spin_forever(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                rclpy.spin_once(self, timeout_sec=0.0)
                continue
            if len(data) != PACKET_SIZE:
                self.get_logger().warn(
                    f"Dropping malformed packet ({len(data)} bytes, expected {PACKET_SIZE})"
                )
                continue
            q = struct.unpack(PACKET_FORMAT, data)
            self.publish_pose(q)
            rclpy.spin_once(self, timeout_sec=0.0)

    def publish_pose(self, q):
        msg = ArmPose()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = "task_space_real_udp_bridge"
        msg.is_left = True
        msg.shoulder = JointState(position=[q[0], q[1], q[2]])
        msg.elbow = JointState(position=[q[3], q[4]])
        msg.wrist = JointState(position=[q[5]])
        msg.include_hand_pose = False
        self.pub.publish(msg)


class SingleJointBridge(Node):
    """--only-joint mode: bypasses joint_command_node. Only this one joint's CAN ID
    ever receives a command from this bridge -- nothing else is touched."""

    def __init__(self, host, port, label, cfg):
        super().__init__("ros_bridge_single_joint")
        self.pub = self.create_publisher(MotorCmd, "/interfacing/motorCMD", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.1)
        self.index = JOINT_INDEX[label]
        self.cfg = cfg
        self.prev_target = None
        self.get_logger().info(
            f"Listening for UDP joint packets on {host}:{port} -- ONLY publishing "
            f"MotorCmd for '{label}' (can_id=0x{cfg['can_id']:02X}). No other motor "
            f"will receive any command from this bridge."
        )

    def spin_forever(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                rclpy.spin_once(self, timeout_sec=0.0)
                continue
            if len(data) != PACKET_SIZE:
                self.get_logger().warn(
                    f"Dropping malformed packet ({len(data)} bytes, expected {PACKET_SIZE})"
                )
                continue
            q = struct.unpack(PACKET_FORMAT, data)
            self.publish_motor_cmd(q[self.index])
            rclpy.spin_once(self, timeout_sec=0.0)

    def publish_motor_cmd(self, raw_deg):
        cfg = self.cfg
        target = raw_deg
        if cfg["limit_range"]:
            target = max(cfg["lower_limit"], min(cfg["upper_limit"], target))

        if self.prev_target is not None:
            step = max(-DELTA_MAX_DEG, min(DELTA_MAX_DEG, target - self.prev_target))
            target = self.prev_target + step
        self.prev_target = target

        calibrated = cfg["direction"] * (target - cfg["zero_offset"])

        msg = MotorCmd()
        msg.motor_id = cfg["can_id"]
        msg.control_type = MotorCmd.POSITION_LOOP
        msg.position = float(calibrated)
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--only-joint", type=str, default=None,
                        help="bypass joint_command_node and only publish a raw MotorCmd "
                        "for this one joint, e.g. shoulder_roll. Nothing else moves.")
    args = parser.parse_args()

    rclpy.init()
    if args.only_joint:
        if args.only_joint not in JOINT_INDEX:
            print(f"Unknown joint '{args.only_joint}'. Valid joints: {', '.join(JOINT_INDEX)}")
            rclpy.shutdown()
            return
        cfg = load_joint_config(args.only_joint)
        node = SingleJointBridge(args.host, args.port, args.only_joint, cfg)
    else:
        node = RosBridge(args.host, args.port)

    try:
        node.spin_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
