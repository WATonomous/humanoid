#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D
from builtin_interfaces.msg import Time
from vision_msgs.msg import Detection2D, BoundingBox2D, Pose2D, Point2D
from cv_bridge import CvBridge
import numpy as np


class DummyShuttlecockPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_shuttlecock_publisher")

        self.fx = self.fy = 500.0
        self.cx, self.cy = 320.0, 240.0
        self.u, self.v = 370.0, 290.0
        self.depth_mm = 2000  # 2.0m, will be read back as uint16 mm by the real node
        self.bridge = CvBridge()
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", 10
        )
        self.det_pub = self.create_publisher(
            Detection2D, "/perception/shuttlecock/detections_2d", 10
        )
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)

        self.timer = self.create_timer(
            0.5, self._publish_all
        )  # 2Hz is plenty for a test

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()
        
        # Build and Publish CameraInfo
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp 
        info_msg.header.frame_id = "camera_link"
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        self.info_pub.publish(info_msg)

        # Build and publish Detection2D using stamp + self.u/self.v
        det_image = Detection2D()
        det_image.header.stamp = stamp
        det_image.header.frame_id = "camera_link"
        det_image.bbox = BoundingBox2D()
        det_image.bbox.center.position = Point2D()
        det_image.bbox.center.position.x = self.u
        det_image.bbox.center.position.y = self.v
        self.det_pub.publish(det_image)
        
        # Build and publish Image (depth) using stamp + self.depth_mm
        depth_array = np.full((480,640), self.depth_mm, dtype=np.uint16)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding="16UC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_link"
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyShuttlecockPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()