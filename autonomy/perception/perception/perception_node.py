#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np


class PerceptionNode(Node):
    """Main perception node for processing and relaying sensor data."""

    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers for raw camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers for processed data
        self.processed_rgb_pub = self.create_publisher(
            Image,
            '/perception/rgb_processed',
            10
        )
        
        self.processed_depth_pub = self.create_publisher(
            Image,
            '/perception/depth_processed',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/perception/camera_info',
            10
        )
        
        # Store latest images for processing
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        self.get_logger().info('Perception node initialized - publishing processed camera data')

    def camera_info_callback(self, msg):
        """Relay camera info to downstream nodes."""
        self.camera_info = msg
        self.camera_info_pub.publish(msg)

    def image_callback(self, msg):
        """Process RGB camera images and publish."""
        try:
            self.latest_rgb = msg
            
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the image (edge detection example)
            processed_image = self.process_rgb_image(cv_image)
            
            # Convert back to ROS image and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            processed_msg.header = msg.header
            self.processed_rgb_pub.publish(processed_msg)
            
            self.get_logger().info('Published processed RGB image')
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Process depth camera images and publish."""
        try:
            self.latest_depth = msg
            
            # Convert ROS depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            
            # Process the depth image (filtering example)
            processed_depth = self.process_depth_image(depth_image)
            
            # Convert back to ROS image and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed_depth, 'passthrough')
            processed_msg.header = msg.header
            self.processed_depth_pub.publish(processed_msg)
            
            self.get_logger().info('Published processed depth image')
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def process_rgb_image(self, image):
        """Process RGB image for feature detection."""
        # Example: edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        processed = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return processed

    def process_depth_image(self, depth_image):
        """Process depth image for filtering and enhancement."""
        # Example: depth filtering and noise reduction
        # Convert to float for processing
        depth_float = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters
        
        # Filter out invalid depths (too close or too far)
        mask = (depth_float > 0.1) & (depth_float < 10.0)
        filtered_depth = np.where(mask, depth_float, 0.0)
        
        # Apply median filter to reduce noise
        filtered_depth = cv2.medianBlur(filtered_depth.astype(np.float32), 5)
        
        # Convert back to mm
        return (filtered_depth * 1000.0).astype(np.uint16)


def main(args=None):
    rclpy.init(args=args)
    
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()