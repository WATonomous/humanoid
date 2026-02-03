import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import torch
from spconv.pytorch.utils import PointToVoxel

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class VoxelGridNode(Node):
    """
    World-modelling voxel grid node.

    - Expects depth in millimeters (uint16) or meters (float32/float16).
    - Publishes voxel centers as PointCloud2.
    """

    def __init__(self):
        super().__init__('voxel_grid_node')

        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, qos_profile_sensor_data
        )

        self.voxel_pub = self.create_publisher(
            PointCloud2, '/behaviour/voxel_grid', qos_profile_sensor_data
        )

        self.rgb_image = None
        self.depth_image = None

        self.fx, self.fy, self.cx, self.cy = 525.0, 525.0, 320.0, 240.0
        self.have_intrinsics = False

        self.voxel_size = 0.04   
        self.max_range = 3.5    
        self.min_range = 0.1  
        self.stride = 4      

        self.get_logger().info('Voxel Grid Node started (world modelling, headless)')

    # ---------- message decoding helpers ----------
    @staticmethod
    def _decode_rgb_bgr8(msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return arr.reshape(msg.height, msg.width, 3)

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray:
        """
        Returns depth in meters as float32 HxW.
        Supports:
          - mono16 / 16UC1: uint16 in millimeters
          - 32FC1: float32 in meters
          - 16FC1: float16 in meters
        """
        enc = (msg.encoding or '').lower()

        if enc in ('mono16', '16uc1'):
            d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            return (d.astype(np.float32) / 1000.0)  # mm -> m

        if enc == '32fc1':
            d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            return d

        if enc == '16fc1':
            d = np.frombuffer(msg.data, dtype=np.float16).reshape(msg.height, msg.width)
            return d.astype(np.float32)

        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return (d.astype(np.float32) / 1000.0)

    # ---------- callbacks ----------
    def info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.have_intrinsics = True

        self.get_logger().info(f'Intrinsics updated: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')

    def rgb_callback(self, msg: Image):
        self.rgb_image = self._decode_rgb_bgr8(msg)
        self.process_rgbd_if_ready()

    def depth_callback(self, msg: Image):
        self.depth_image = self._decode_depth(msg)
        self.process_rgbd_if_ready()

    def process_rgbd_if_ready(self):
        # Check depth + intrinsics are for voxelization
        if self.depth_image is None or not self.have_intrinsics:
            return

        points = self.depth_to_pointcloud(self.depth_image)

        if points.shape[0] == 0:
            self.get_logger().warn('No valid points for voxelization')
            return

        voxel_centers = self.create_voxel_grid(points)
        if voxel_centers.shape[0] == 0:
            self.get_logger().warn('Voxelization returned 0 voxels')
            return

        self.publish_voxel_grid(voxel_centers)

        #reset
        self.depth_image = None
        self.rgb_image = None

    # ---------- core logic ----------
    def depth_to_pointcloud(self, depth_m: np.ndarray) -> np.ndarray:
        """
        Depth image (meters) -> Nx3 point cloud in camera frame.
        Uses stride downsampling and range filtering.
        """
        h, w = depth_m.shape

        v = np.arange(0, h, self.stride, dtype=np.int32)
        u = np.arange(0, w, self.stride, dtype=np.int32)
        uu, vv = np.meshgrid(u, v)  # shapes: (len(v), len(u))

        z = depth_m[vv, uu]
        valid = (z > self.min_range) & (z < self.max_range) & np.isfinite(z)

        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float32)

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        pts = np.stack([x, y, z], axis=1).astype(np.float32)
        return pts

    def create_voxel_grid(self, points: np.ndarray) -> np.ndarray:
        """
        Uses spconv PointToVoxel to voxelize points, then returns voxel center coordinates (meters).
        """
        # Here we set it from observed points with a little padding.
        mins = points.min(axis=0)
        maxs = points.max(axis=0)
        padding = 0.5  # meters

        coors_range_m = np.array([
            mins[0] - padding, mins[1] - padding, mins[2] - padding,
            maxs[0] + padding, maxs[1] + padding, maxs[2] + padding
        ], dtype=np.float32)

        voxel_gen = PointToVoxel(
            vsize_xyz=[self.voxel_size, self.voxel_size, self.voxel_size],
            coors_range_xyz=coors_range_m.tolist(), #Convert numpy to python
            num_point_features=3,
            max_num_voxels=10000,
            max_num_points_per_voxel=20
        )

        pc_tensor = torch.from_numpy(points)  # float32
        voxels, indices, num_points = voxel_gen(pc_tensor)  # indices: [M, 3] (z,y,x) or (x,y,z) depending on impl

        if indices.numel() == 0:
            return np.zeros((0, 3), dtype=np.float32)

        # spconv PointToVoxel indices are typically in (z, y, x) order.
        idx = indices.cpu().numpy().astype(np.float32)

        # If your output looks swapped, flip this reorder.
        idx_xyz = idx[:, ::-1]  # (z,y,x) -> (x,y,z)

        origin = coors_range_m[:3]  # meters
        centers = origin + (idx_xyz + 0.5) * self.voxel_size  # meters

        self.get_logger().info(f'Voxels: {centers.shape[0]} from points: {points.shape[0]}')
        return centers.astype(np.float32)

    def publish_voxel_grid(self, voxel_centers: np.ndarray):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc_msg = pc2.create_cloud(header, fields, voxel_centers.tolist())
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
