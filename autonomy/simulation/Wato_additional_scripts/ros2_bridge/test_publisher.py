import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class TestFloatPublisher(Node):
    def __init__(self):
        super().__init__('test_float_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/hand_joint_positions', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Test Float Publisher initialized')

    def timer_callback(self):
        msg = Float32MultiArray()
        # Publish a list of 15 floats (e.g., all 1.0 for testing)
        msg.data = [2.0] * 15
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint positions: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TestFloatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
