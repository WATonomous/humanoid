import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorFeedback
from config_loader import load_hardware_mapping, angle_computation
import mujoco

class Real2SimMirrorNode(Node):
    def __init__(self):
        super().__init__("real2sim_mirror_node")
        self.subscription = self.create_subscription(
            MotorFeedback, 
            "/interfacing/motorFeedback",
            self.feedback_callback, 
            10
        )
        self.lookup_table = load_hardware_mapping("src/joint_command/config/hardware_mapping.yaml")

        self.model = mujoco.MjModel.from_xml_path("src/simulation/real2sim_mirror/mirror.xml")
        self.data = mujoco.MjData(self.model)

    def feedback_callback(self, msg):
        motor_id = msg.motor_id
        position = msg.position

        angle_rad = angle_computation(motor_id, position, self.lookup_table)
        
        if angle_rad is None:
            return
        
        joint_name = self.lookup_table[motor_id]["joint_name"]
        joint_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name
        )

        qpos_addr = self.model.jnt.qposadr[joint_id]
        self.data.qpos[qpos_addr] = angle_rad

        mujoco.mj_forward(self.model, self.data)


if __name__ == "__main__":
    rclpy.init()
    node = Real2SimMirrorNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()