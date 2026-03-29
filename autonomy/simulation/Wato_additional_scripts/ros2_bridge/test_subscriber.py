import rclpy
from rclpy.node import Node
from common_msgs.msg import VRHandState


# TODO: confirm topic name with Yichen
TOPIC_NAME = '/vr_hand_state'


class HandPoseSubscriber(Node):
    def __init__(self):
        super().__init__('hand_pose_subscriber')
        self.subscription = self.create_subscription(
            VRHandState,
            TOPIC_NAME,
            self.listener_callback,
            10)
        self.joint_positions = None
        self.get_logger().info('Hand Pose Subscriber initialized')

    def listener_callback(self, msg):
        # Map VRHandState fields to 15 joint positions.
        # Joint order matches HAND_CFG (Revolute_1 to Revolute_15) as loaded
        # from hand.usd. TODO: verify exact ordering with simulation.
        #
        # Mapping:
        #   index:  mcp=curl, pip=flexion, dip=flexion
        #   middle: mcp=curl, pip=flexion, dip=flexion
        #   ring:   mcp=curl, pip=flexion, dip=flexion
        #   pinky:  mcp=curl, pip=flexion, dip=flexion
        #   thumb:  cmc=curl, mcp=curl,    ip=curl
        #
        # abduction and opposition are not used (no corresponding joints in
        # the 15-DOF hand model)
        self.joint_positions = [
            msg.index.curl,
            msg.index.flexion,
            msg.index.flexion,
            msg.middle.curl,
            msg.middle.flexion,
            msg.middle.flexion,
            msg.ring.curl,
            msg.ring.flexion,
            msg.ring.flexion,
            msg.pinky.curl,
            msg.pinky.flexion,
            msg.pinky.flexion,
            msg.thumb.curl,
            msg.thumb.curl,
            msg.thumb.curl,
        ]
        self.get_logger().info(
            f'Received joint positions: {self.joint_positions}')

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
