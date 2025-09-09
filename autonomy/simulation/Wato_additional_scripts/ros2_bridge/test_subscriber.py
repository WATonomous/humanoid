import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class HandPoseSubscriber(Node):
    def __init__(self):
        super().__init__('hand_pose_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_joint_positions',
            self.listener_callback,
            10)
        self.joint_positions = None
        self.get_logger().info('Hand Pose Subscriber initialized')

    def listener_callback(self, msg):
        if len(msg.data) == 15:
            self.joint_positions = list(msg.data)
            self.get_logger().info(f'Received joint positions: {self.joint_positions}')
        else:
            self.get_logger().warn(f'Received invalid joint positions: expected 15 floats, got {len(msg.data)}')

    def get_latest_joint_positions(self):
        return self.joint_positions

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()