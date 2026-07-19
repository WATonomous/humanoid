#!/usr/bin/env python3
"""Wave demo: alternate the arm between the 'raised' and 'wave' recorded poses.

Publishes ArmPose on /behaviour/arm_pose at 50 Hz (a single long-lived publisher, so it
doesn't churn FastDDS discovery), switching the target pose every --period seconds. The
joint_command_node safety layer turns each target into velocity-limited, smoothed MotorCmds
-- so the arm eases between the two poses, i.e. waves. Poses are the ones in
recorded_poses.yaml.

Run inside the joint_command container AFTER joint_command_node is up and shows
Publisher/Subscription count 1/1 on /interfacing/motorCMD (see MOVE_ARM_RUNBOOK.md):

    python3 /root/ament_ws/src/joint_command/scripts/wave_demo.py [--period 2.0]

Ctrl+C to stop (the node then holds its last commanded pose).
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from common_msgs.msg import ArmPose, JointState

# [shoulder pitch, roll, yaw], [elbow pitch, roll], [wrist pitch]  -- from recorded_poses.yaml
POSES = {
    "raised": ([-124.0, -40.5, -165.4], [-88.7, 14.1], [0.0]),
    "wave":   ([-110.2, -40.4, -114.2], [-76.2, 13.1], [0.0]),
}
SEQUENCE = ["raised", "wave"]  # alternates through this list


class WaveDemo(Node):
    def __init__(self, period):
        super().__init__("wave_demo")
        self.pub = self.create_publisher(ArmPose, "/behaviour/arm_pose", 10)
        self.period = period
        self.idx = 0
        self.last_switch = self.get_clock().now()
        self.get_logger().info(
            f"Waving: alternating {SEQUENCE} every {period}s. Starting at '{SEQUENCE[0]}'."
        )
        self.timer = self.create_timer(1.0 / 50.0, self.tick)  # 50 Hz publish

    def tick(self):
        now = self.get_clock().now()
        if (now - self.last_switch).nanoseconds * 1e-9 >= self.period:
            self.idx = (self.idx + 1) % len(SEQUENCE)
            self.last_switch = now
            self.get_logger().info(f"-> {SEQUENCE[self.idx]}")
        sh, el, wr = POSES[SEQUENCE[self.idx]]
        msg = ArmPose()
        msg.header = Header()
        msg.header.stamp = now.to_msg()
        msg.name = "wave_demo"
        msg.is_left = True
        msg.shoulder = JointState(position=sh)
        msg.elbow = JointState(position=el)
        msg.wrist = JointState(position=wr)
        msg.include_hand_pose = False
        self.pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--period", type=float, default=2.0, help="seconds between pose switches")
    args = p.parse_args()
    rclpy.init()
    node = WaveDemo(args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
