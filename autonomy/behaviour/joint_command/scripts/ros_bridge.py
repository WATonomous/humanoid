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

Each packet is republished as an ArmPose on /behaviour/arm_pose, which joint_command_node
consumes and turns into position-clamped, velocity/delta-limited, smoothed MotorCmds -- so
every motor command from this bridge goes through the safety layer. (A former --only-joint
mode that published a raw MotorCmd straight to /interfacing/motorCMD, bypassing
joint_command_node, has been removed: nothing here can command a motor outside the safety
layer anymore.)

Usage:
  /usr/bin/python3 ros_bridge.py [--host 127.0.0.1] [--port 5005]
"""

import argparse
import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from common_msgs.msg import ArmPose, JointState

PACKET_FORMAT = "6d"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


class RosBridge(Node):
    """Full arm, via ArmPose -> joint_command_node (the safety layer)."""

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    rclpy.init()
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
