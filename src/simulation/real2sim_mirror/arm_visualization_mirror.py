import math
import yaml

import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorFeedback


class Real2SimMirrorNode(Node):
    def __init__(self):
        super().__init__("real2sim_mirror_node")

        self.subscription = self.create_subscription(
            MotorFeedback,
            "/interfacing/motorFeedback",
            self.feedback_callback,
            10,
        )

        self.lookup_table = self.load_hardware_mapping(
            "src/joint_command/config/hardware_mapping.yaml"
        )

        # +1 = same direction, -1 = mirrored direction
        # These are temporary until we verify the MuJoCo joint axes.
        self.mirror_directions = {
            "shoulder_pitch": 1,
            "shoulder_roll": -1,
            "shoulder_yaw": 1,
            "elbow_pitch": 1,
            "elbow_roll": -1,
            "wrist_pitch": 1,
        }

    def load_hardware_mapping(self, yaml_file_path):
        with open(yaml_file_path, "r") as f:
            data = yaml.safe_load(f)

        lookup_table = {}

        for side, limbs in data.items():
            for limb, joints in limbs.items():
                for joint_type, config in joints.items():
                    joint_name = f"{side}_{limb}_{joint_type}"

                    entry = config.copy()
                    entry["joint_name"] = joint_name

                    lookup_table[config["can_id"]] = entry

        return lookup_table

    def angle_computation(self, motor_id, position_deg):
        if motor_id not in self.lookup_table:
            self.get_logger().warn(
                f"Unknown Motor ID {motor_id}"
            )
            return None

        config = self.lookup_table[motor_id]

        true_angle_deg = (
            position_deg - config["zero_offset"]
        ) * config["direction"]

        return math.radians(true_angle_deg)

    def mirror_angle(self, angle_rad, joint_name):
        joint_type = "_".join(joint_name.split("_")[-1])
        direction = self.mirror_directions[joint_type]

        return angle_rad * direction

    def feedback_callback(self, msg):
        motor_id = msg.motor_id
        position_deg = msg.position

        angle_rad = self.angle_computation(
            motor_id,
            position_deg,
        )

        if angle_rad is None:
            return

        joint_name = self.lookup_table[motor_id]["joint_name"]

        mirrored_angle_rad = self.mirror_angle(
            angle_rad,
            joint_name,
        )

        self.get_logger().info(
            f"{joint_name}: "
            f"{angle_rad:.3f} rad -> "
            f"{mirrored_angle_rad:.3f} rad"
        )
    
def test():
    lookup_table = Real2SimMirrorNode.load_hardware_mapping(
        None,
        "src/joint_command/config/hardware_mapping.yaml"
    )

    test_motor_id = 12
    test_position_deg = -80.4

    config = lookup_table[test_motor_id]

    true_angle_deg = (
        test_position_deg - config["zero_offset"]
    ) * config["direction"]

    true_angle_rad = math.radians(true_angle_deg)

    joint_name = config["joint_name"]

    joint_type = "_".join(joint_name.split("_")[1:])

    mirror_direction = {
        "shoulder_pitch": 1,
        "shoulder_roll": -1,
        "shoulder_yaw": -1,
        "elbow_pitch": 1,
        "elbow_roll": -1,
        "wrist_pitch": 1,
    }[joint_type]

    mirrored_angle_rad = true_angle_rad * mirror_direction

    print(f"Joint: {joint_name}")
    print(f"Position: {test_position_deg} deg")
    print(f"True angle: {true_angle_rad:.3f} rad")
    print(f"Mirrored angle: {mirrored_angle_rad:.3f} rad")


if __name__ == "__main__":
    test()