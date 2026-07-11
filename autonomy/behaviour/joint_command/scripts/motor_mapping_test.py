#!/usr/bin/env python3
"""Interactive tool to verify hardware_mapping.yaml against the real left arm.

Moves exactly one joint at a time through the real pipeline
(/behaviour/arm_pose -> joint_command_node -> /interfacing/motorCMD -> can_node)
so you can watch the physical arm and confirm the CAN ID in
config/hardware_mapping.yaml really drives the motor you think it does.

This does NOT use Isaac Sim: the sim stack isn't wired to this ROS topic
chain, so it can't tell you anything about a real CAN motor.

Prerequisites (run in separate terminals, sourced against the built workspace):
  ros2 launch can can.launch.py
  ros2 launch joint_command joint_command.launch.py

Safety:
  - Arm should start from a known, clear, safe pose. Keep the e-stop in reach.
  - Motion is ramped in small steps by this script (not left to the node's
    internal smoothing, which can jump on the very first command it ever
    receives).
  - Test angles are automatically kept inside each joint's lower/upper limit
    as defined in hardware_mapping.yaml, and picked to move away from the
    limit if zero sits on a boundary (e.g. the elbow's range is [-130, 0]).

Usage:
  python3 motor_mapping_test.py [--angle 15] [--hold 2.0] [--joint shoulder_pitch]
"""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from common_msgs.msg import ArmPose, JointState

# (group, joint, index into that JointState.position array) in the same
# order joint_command_core.cpp loads them in -- this is exactly what
# hardware_mapping.yaml's "left" section maps to CAN IDs for the arm.
JOINT_SPECS = [
    ("shoulder", "pitch", "shoulder", 0),
    ("shoulder", "roll", "shoulder", 1),
    ("shoulder", "yaw", "shoulder", 2),
    ("elbow", "pitch", "elbow", 0),
    ("elbow", "roll", "elbow", 1),
    ("wrist", "pitch", "wrist", 0),
]

RATE_HZ = 50.0
STEPS_PER_MOVE = 60  # ~1.2s ramp at 50Hz


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


def load_joints(arm_side="left"):
    import yaml

    path = find_hardware_mapping()
    with open(path) as f:
        config = yaml.safe_load(f)

    arm = config[arm_side]
    joints = []
    for group, joint_name, msg_field, idx in JOINT_SPECS:
        node = arm[group][joint_name]
        joints.append({
            "label": f"{group}_{joint_name}",
            "msg_field": msg_field,
            "index": idx,
            "can_id": node["can_id"],
            "lower_limit": float(node["lower_limit"]),
            "upper_limit": float(node["upper_limit"]),
        })
    return path, joints


def safe_target(joint, current, angle):
    """Pick a test target that stays inside [lower_limit, upper_limit],
    moving off zero in whichever direction is actually free."""
    candidate = current + angle
    if candidate > joint["upper_limit"]:
        candidate = current - angle
    if candidate < joint["lower_limit"]:
        candidate = current
    return max(joint["lower_limit"], min(joint["upper_limit"], candidate))


class MotorMappingTester(Node):
    def __init__(self):
        super().__init__("motor_mapping_tester")
        self.pub = self.create_publisher(ArmPose, "/behaviour/arm_pose", 10)
        self.positions = {"shoulder": [0.0, 0.0, 0.0], "elbow": [0.0, 0.0], "wrist": [0.0]}

    def publish_pose(self):
        msg = ArmPose()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = "motor_mapping_test"
        msg.is_left = True
        msg.shoulder = JointState(position=list(self.positions["shoulder"]))
        msg.elbow = JointState(position=list(self.positions["elbow"]))
        msg.wrist = JointState(position=list(self.positions["wrist"]))
        msg.include_hand_pose = False
        self.pub.publish(msg)

    def ramp_joint_to(self, msg_field, index, target, steps=STEPS_PER_MOVE, rate_hz=RATE_HZ):
        start = self.positions[msg_field][index]
        for step in range(1, steps + 1):
            self.positions[msg_field][index] = start + (target - start) * step / steps
            self.publish_pose()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate_hz)
        self.positions[msg_field][index] = target
        self.publish_pose()

    def hold(self, seconds, rate_hz=RATE_HZ):
        ticks = max(1, int(seconds * rate_hz))
        for _ in range(ticks):
            self.publish_pose()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate_hz)

    def zero_all(self):
        for field in self.positions:
            for i in range(len(self.positions[field])):
                self.positions[field][i] = 0.0
        self.publish_pose()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle", type=float, default=15.0, help="test angle in degrees")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds to hold at target")
    parser.add_argument("--joint", type=str, default=None,
                         help="test only this joint, e.g. shoulder_pitch")
    args = parser.parse_args()

    path, joints = load_joints("left")
    print(f"Loaded mapping from {path}\n")

    if args.joint:
        valid_labels = [f"{group}_{name}" for group, name, _, _ in JOINT_SPECS]
        joints = [j for j in joints if j["label"] == args.joint]
        if not joints:
            print(f"Unknown joint '{args.joint}'. Valid joints: {', '.join(valid_labels)}")
            sys.exit(1)

    print("This will move the REAL left arm, one joint at a time.")
    print("Make sure it's clear of obstacles/people and the e-stop is in reach.")
    confirm = input("Type 'YES' to proceed: ")
    if confirm != "YES":
        print("Aborted.")
        sys.exit(0)

    rclpy.init()
    node = MotorMappingTester()
    results = []
    try:
        # give the publisher time to match with subscribers
        time.sleep(1.0)
        for joint in joints:
            current = node.positions[joint["msg_field"]][joint["index"]]
            target = safe_target(joint, current, args.angle)
            if target == current:
                print(f"\n[{joint['label']}] can_id=0x{joint['can_id']:02X} "
                      "-- no free range to move from current position, skipping.")
                continue

            print(f"\n[{joint['label']}] can_id=0x{joint['can_id']:02X} "
                  f"limits=({joint['lower_limit']}, {joint['upper_limit']})")
            input(f"  Press Enter to move this joint to {target:+.1f} deg...")

            node.ramp_joint_to(joint["msg_field"], joint["index"], target)
            node.hold(args.hold)

            observed = input("  Which joint actually moved (or 'none' / 'n/a')? ").strip()

            node.ramp_joint_to(joint["msg_field"], joint["index"], current)
            node.hold(0.5)

            match = observed.strip().lower() in (joint["label"], joint["label"].replace("_", " "))
            results.append((joint["label"], joint["can_id"], observed, match))

    except KeyboardInterrupt:
        print("\nInterrupted -- returning all joints to zero.")
    finally:
        try:
            node.zero_all()
            node.hold(1.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    print("\n=== Summary ===")
    for label, can_id, observed, match in results:
        status = "OK" if match else "CHECK MAPPING"
        print(f"  {label:16s} can_id=0x{can_id:02X}  observed='{observed}'  -> {status}")


if __name__ == "__main__":
    main()
