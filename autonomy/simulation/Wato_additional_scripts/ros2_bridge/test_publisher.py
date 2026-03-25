import rclpy
from rclpy.node import Node
from common_msgs.msg import VRHandState, VRFingerFeatures


# TODO: confirm topic name with Yichen
TOPIC_NAME = '/vr_hand_state'


def make_finger(curl, flexion=0.0, abduction=0.0, opposition=0.0):
    f = VRFingerFeatures()
    f.curl = curl
    f.flexion = flexion
    f.abduction = abduction
    f.opposition = opposition
    return f


class TestVRHandPublisher(Node):
    def __init__(self):
        super().__init__('test_vr_hand_publisher')
        self.publisher_ = self.create_publisher(VRHandState, TOPIC_NAME, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Test VR Hand Publisher initialized')

    def timer_callback(self):
        msg = VRHandState()
        # Publish test values: all fingers curled at 1.0
        msg.thumb = make_finger(curl=1.0)
        msg.index = make_finger(curl=1.0, flexion=1.0, abduction=0.5)
        msg.middle = make_finger(curl=1.0, flexion=1.0, abduction=0.5)
        msg.ring = make_finger(curl=1.0, flexion=1.0, abduction=0.5)
        msg.pinky = make_finger(curl=1.0, flexion=1.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Published VR hand state')


def main(args=None):
    rclpy.init(args=args)
    node = TestVRHandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
