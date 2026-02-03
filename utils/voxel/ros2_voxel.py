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

class VoxelGridNode(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')
        
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)

        self.voxel_pub = self.create_publisher(PointCloud2, '/voxel_grid', 10)
        
        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()
        
        # Camera intrinsics (ovewrritten for your camera)
        self.fx, self.fy, self.cx, self.cy = 525.0, 525.0, 320.0, 240.0
        
        self.voxel_size = 0.04  
        self.max_range = 3.5    
        
        self.get_logger().info('Voxel Grid Node started')

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_rgbd_if_ready()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.process_rgbd_if_ready()

    def process_rgbd_if_ready(self):
        if self.rgb_image is None or self.depth_image is None:
            return
            
        # Convert RGBD to point cloud
        points = self.rgbd_to_pointcloud(self.rgb_image, self.depth_image)
        
        if len(points) > 0:
            # Create voxel grid
            voxel_centers = self.create_voxel_grid(points)
            
            # Publish voxel grid as point cloud
            self.publish_voxel_grid(voxel_centers)

    def rgbd_to_pointcloud(self, rgb, depth):
        points = []
        h, w = depth.shape
        
        for v in range(0, h, 4):  #Skipping every 4 pixels to downsample, can change if necessary
            for u in range(0, w, 4):
                z = depth[v, u] / 1000.0 
                
                if z > 0.1 and z < self.max_range:  
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    points.append([x, y, z])
        
        return np.array(points, dtype=np.float32)

    def create_voxel_grid(self, points):

        points_scaled = points / self.voxel_size
        
        min_coords = points_scaled.min(axis=0)
        max_coords = points_scaled.max(axis=0)
        padding = 2.0
        
        coors_range = [
            min_coords[0] - padding, min_coords[1] - padding, min_coords[2] - padding,
            max_coords[0] + padding, max_coords[1] + padding, max_coords[2] + padding
        ]
        
        voxel_gen = PointToVoxel(
            vsize_xyz=[1.0, 1.0, 1.0], 
            coors_range_xyz=coors_range,
            num_point_features=3,
            max_num_voxels=10000,
            max_num_points_per_voxel=20
        )
        
        pc_tensor = torch.from_numpy(points_scaled)
        voxels, indices, num_points_per_voxel = voxel_gen(pc_tensor)
        
        voxel_centers = indices.numpy() * self.voxel_size + np.array(coors_range[:3]) * self.voxel_size + self.voxel_size/2
        
        self.get_logger().info(f'Created {len(voxels)} voxels from {len(points)} points')
        
        return voxel_centers

    def publish_voxel_grid(self, voxel_centers):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        pc_msg = pc2.create_cloud(header, fields, voxel_centers)
        self.voxel_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()