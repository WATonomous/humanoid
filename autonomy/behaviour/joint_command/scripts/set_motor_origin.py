#!/usr/bin/env python3
"""Interactive tool to zero-calibrate each real arm motor against the sim's zero pose.

task_space_real.py (and joint_command_node) publish/consume joint angles that are
relative to whatever the USD/URDF calls joint position 0.0. For the real arm to end
up in the same physical pose the sim thinks it's in, each real motor's own internal
zero (set via a SET_ORIGIN CAN command) needs to correspond to that exact same pose.

This script does NOT go through joint_command_node/ArmPose (that pipeline only ever
sends POSITION_LOOP commands). It publishes raw MotorCmd messages straight to
/interfacing/motorCMD so it can send DISABLE and SET_ORIGIN, which joint_command_node
doesn't expose.

For each of the 6 arm joints, in order:
  1. Prints the joint name + can_id, and reminds you to have the sim-zero reference
     pose visible (run, in another terminal:
       python autonomy/simulation/Wato_additional_scripts/robot_arm_controllers/joint_mapping_reference.py --joint <name> --amplitude 0
     which holds that one joint at its sim-zero position so you can compare by eye).
  2. On confirmation, sends DISABLE for that motor so you can move it by hand.
     WARNING: disabling cuts active holding torque -- the joint may sag/swing under
     gravity or its own momentum. Support it by hand before disabling, especially
     shoulder/elbow.
  3. Waits for you to manually position the real joint to match the sim-zero pose.
  4. On confirmation, sends SET_ORIGIN (permanent) so the motor's current position
     becomes its new 0 degrees.
  5. Sends a POSITION_LOOP command at position=0 to re-engage holding and let you
     visually confirm it didn't drift.

If it's mechanically inconvenient to zero a joint at the exact sim-zero pose, skip
SET_ORIGIN for it and instead set hardware_mapping.yaml's zero_offset for that joint
to the motor's feedback reading once manually placed at the sim-zero pose -- see
/interfacing/motorFeedback (published by can_node) for that reading.

Prerequisites (separate terminals, sourced against the built workspace):
  ros2 launch can can.launch.py
  ros2 launch joint_command joint_command.launch.py   # optional for this script,
                                                        # but fine to leave running

Usage:
  python3 set_motor_origin.py [--joint shoulder_pitch]
"""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorCmd

JOINT_SPECS = [
    ("shoulder", "pitch"),
    ("shoulder", "roll"),
    ("shoulder", "yaw"),
    ("elbow", "pitch"),
    ("elbow", "roll"),
    ("wrist", "pitch"),
]


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
    for group, joint_name in JOINT_SPECS:
        node = arm[group][joint_name]
        joints.append({"label": f"{group}_{joint_name}", "can_id": node["can_id"]})
    return path, joints


class OriginSetter(Node):
    def __init__(self):
        super().__init__("motor_origin_setter")
        self.pub = self.create_publisher(MotorCmd, "/interfacing/motorCMD", 10)

    def _publish(self, msg, times=5, rate_hz=20.0):
        for _ in range(times):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate_hz)

    def disable(self, can_id):
        msg = MotorCmd()
        msg.motor_id = can_id
        msg.control_type = MotorCmd.DISABLE
        self._publish(msg)

    def set_origin(self, can_id, permanent=True):
        msg = MotorCmd()
        msg.motor_id = can_id
        msg.control_type = MotorCmd.SET_ORIGIN
        msg.temporary = not permanent
        self._publish(msg)

    def hold_zero(self, can_id, seconds=1.5, rate_hz=50.0):
        msg = MotorCmd()
        msg.motor_id = can_id
        msg.control_type = MotorCmd.POSITION_LOOP
        msg.position = 0.0
        self._publish(msg, times=max(1, int(seconds * rate_hz)), rate_hz=rate_hz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", type=str, default=None,
                         help="calibrate only this joint, e.g. shoulder_pitch")
    args = parser.parse_args()

    path, joints = load_joints("left")
    print(f"Loaded mapping from {path}\n")

    if args.joint:
        valid_labels = [f"{g}_{n}" for g, n in JOINT_SPECS]
        joints = [j for j in joints if j["label"] == args.joint]
        if not joints:
            print(f"Unknown joint '{args.joint}'. Valid joints: {', '.join(valid_labels)}")
            sys.exit(1)

    print("This will DISABLE and re-zero REAL arm motors, one joint at a time.")
    print("Each disabled joint may sag/swing under gravity -- support it by hand first.")
    print("SET_ORIGIN is sent as PERMANENT: it overwrites the motor's stored zero.")
    confirm = input("Type 'YES' to proceed: ")
    if confirm != "YES":
        print("Aborted.")
        sys.exit(0)

    rclpy.init()
    node = OriginSetter()
    try:
        time.sleep(1.0)  # let publisher match with can_node's subscriber
        for joint in joints:
            label, can_id = joint["label"], joint["can_id"]
            print(f"\n[{label}] can_id=0x{can_id:02X}")
            print(f"  Reference: run in another terminal --")
            print(f"    python autonomy/simulation/Wato_additional_scripts/robot_arm_controllers/"
                  f"joint_mapping_reference.py --joint {label} --amplitude 0")

            confirm = input("  Type 'YES' to disable this motor now: ")
            if confirm != "YES":
                print("  Skipped.")
                continue
            node.disable(can_id)

            input("  Manually move the joint to match the sim-zero pose, then press Enter...")

            confirm = input("  Type 'YES' to permanently set this position as motor zero: ")
            if confirm != "YES":
                print("  Skipped SET_ORIGIN -- motor left disabled, zero unchanged.")
                continue
            node.set_origin(can_id, permanent=True)
            print("  Origin set. Re-engaging position hold at 0 to verify...")
            node.hold_zero(can_id)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\nDone. Any joint you didn't SET_ORIGIN for is still on its old zero --")
    print("either measure its offset via /interfacing/motorFeedback and set")
    print("hardware_mapping.yaml's zero_offset for it, or rerun this script for it later.")


if __name__ == "__main__":
    main()
