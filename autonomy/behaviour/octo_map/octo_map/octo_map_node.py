import rclpy
from rclpy.node import Node
import numpy as np
import torch
from spconv.pytorch.utils import PointToVoxel
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class OctoMapNode(Node):
    def __init__(self):
        super().__init__('octo_map_node')
        
        # Subscribe to processed perception data
        self.rgb_sub = self.create_subscription(
            Image, '/perception/rgb_processed', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/perception/depth_processed', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/perception/camera_info', self.info_callback, 10)

        self.octo_pub = self.create_publisher(PointCloud2, '/behaviour/octo_map', 10)
        
        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera info)
        self.fx, self.fy, self.cx, self.cy = 525.0, 525.0, 320.0, 240.0
        
        self.voxel_size = 0.04  
        self.max_range = 3.5    
        
        self.get_logger().info('OctoMap Node started - subscribing to processed perception data')

    def info_callback(self, msg):
        """Update camera intrinsics from perception node."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f'Updated camera intrinsics: fx={self.fx}, fy={self.fy}')

    def rgb_callback(self, msg):
        """Receive processed RGB image from perception node."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_rgbd_if_ready()
        self.get_logger().info('Received processed RGB image from perception')

    def depth_callback(self, msg):
        """Receive processed depth image from perception node."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.process_rgbd_if_ready()
        self.get_logger().info('Received processed depth image from perception')

    # TODO: Implement point cloud generation and octomap updating

    
def main(args=None):
    rclpy.init(args=args)
    node = OctoMapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()