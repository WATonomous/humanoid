import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorFeedback
from config_loader import load_hardware_mapping, angle_computation

class Real2SimMirrorNode(Node):
    def __init__(self):
        super().__init__("real2sim_mirror_node")
        self.subscription = self.create_subscription(
            MotorFeedback, 
            "/interfacing/motorFeedback",
            self.feedback_callback, 
            10
        )
        self.lookup_table = load_hardware_mapping("src/interfacing/joint_command/config/hardware_mapping.yaml")

    def feedback_callback(self, msg):
        motor_id = msg.motor_id
        raw_position = msg.position
        angle = angle_computation(motor_id, raw_position, self.lookup_table)
        print(f"{msg.motor_id}: {angle} rad")

if __name__ == "__main__":
    rclpy.init()
    node = Real2SimMirrorNode()
    rclpy.spin(node)
    rclpy.shutdown()