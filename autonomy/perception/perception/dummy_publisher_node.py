#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

from cv_bridge import CvBridge
import numpy as np


class DummyPublisherNode(Node):
    def __init__(self):
        super().__init__('dummy_publisher_node')

        self.bridge = CvBridge()

        # Publishers (sensor QoS is IMPORTANT)
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            qos_profile_sensor_data
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            qos_profile_sensor_data
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(1.0, self.publish_dummy_data)

        self.get_logger().info('Dummy camera publisher started (1 Hz)')

    def publish_dummy_data(self):
        now = self.get_clock().now().to_msg()

        header = Header()
        header.stamp = now
        header.frame_id = 'camera_link'

        rgb_image = np.random.randint(
            0, 255, (480, 640, 3), dtype=np.uint8
        )

        rgb_msg = self.bridge.cv2_to_imgmsg(
            rgb_image, encoding='bgr8'
        )
        rgb_msg.header = header
        self.rgb_pub.publish(rgb_msg)

        depth_image = np.full(
            (480, 640), 1500, dtype=np.uint16
        )  # flat plane at 1.5m

        depth_msg = self.bridge.cv2_to_imgmsg(
            depth_image, encoding='mono16'
        )
        depth_msg.header = header
        self.depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header = header
        info.height = 480
        info.width = 640
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = 525.0
        fy = 525.0
        cx = 320.0
        cy = 240.0

        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.camera_info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
