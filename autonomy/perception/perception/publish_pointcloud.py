import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointField
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np


class PCPublisherNode(Node):
    def __init__(self):
        super().__init__('pc_publisher_node')

        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_2_pointcloud, qos_profile_sensor_data)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, qos_profile_sensor_data)

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, qos_profile_sensor_data)

        self.pc_pub = self.create_publisher(PointCloud2, '/camera/depth/pointcloud', qos_profile_sensor_data)
        
        self.fx, self.fy, self.cx, self.cy = 525.0, 525.0, 320.0, 240.0  
        self.rgb_image = None
        
        # Same filtering as voxel grid node
        self.max_range = 3.5
        self.min_range = 0.1

        self.get_logger().info('Point cloud publisher with RGB started')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f'Camera info received')
    
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def depth_2_pointcloud(self, depth_msg):

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        h, w = depth_image.shape

        z = depth_image.astype(np.float32) / 1000.0  
        
        uu, vv = np.meshgrid(np.arange(w), np.arange(h))

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        valid_mask = (z > self.min_range) & (z < self.max_range) & np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        
        x_valid = x[valid_mask]
        y_valid = y[valid_mask]
        z_valid = z[valid_mask]
        
        colors = None
        if self.rgb_image is not None and self.rgb_image.shape[:2] == (h, w):
            v_indices, u_indices = np.where(valid_mask)
            colors = self.rgb_image[v_indices, u_indices]  # Shape: (N, 3)
        
        pts = np.column_stack((x_valid, y_valid, z_valid)).astype(np.float32)
        
        self.get_logger().info(f'Processed point cloud: {pts.shape[0]} valid points out of {h*w} pixels')
        self.get_logger().info(f'Point ranges - X: [{np.min(x_valid):.3f}, {np.max(x_valid):.3f}], '
                               f'Y: [{np.min(y_valid):.3f}, {np.max(y_valid):.3f}], '
                               f'Z: [{np.min(z_valid):.3f}, {np.max(z_valid):.3f}]')
        
        self.publish_pointcloud(pts, colors)


    def publish_pointcloud(self, pts, colors=None):
        if len(pts) == 0:
            self.get_logger().warn('No valid points to publish!')
            return
            
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_depth_optical_frame'  # Use standard ROS camera frame

        # Prepare point data with optional colors
        if colors is not None:
            # Create XYZRGB points
            points_list = []
            for i in range(len(pts)):
                # Convert RGB to packed RGB format
                r, g, b = colors[i]
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                points_list.append([pts[i][0], pts[i][1], pts[i][2], rgb])
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            self.get_logger().info(f'Published colored point cloud with {len(points_list)} points')
        else:
            # Create XYZ-only points with bright color for visibility
            points_list = []
            bright_green = (0 << 16) | (255 << 8) | 0  # Bright green RGB
            for i in range(len(pts)):
                points_list.append([pts[i][0], pts[i][1], pts[i][2], bright_green])
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            self.get_logger().info(f'Published bright green point cloud with {len(points_list)} points')

        pc_msg = pc2.create_cloud(header, fields, points_list)
        self.pc_pub.publish(pc_msg)



def main(args=None):
    rclpy.init(args=args)
    node = PCPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
