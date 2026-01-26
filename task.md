# Depth → Voxel/OctoMap → IsaacLab Integration Plan

## Goal
Integrate an Intel D455 depth camera into the humanoid stack, generate a voxel or octomap world representation, and visualize it inside IsaacLab.

The system is split into **three independent containers**:
- Perception (sensor IO)
- Behavior (world modeling)
- IsaacLab (simulation & visualization)

All communication happens via **ROS 2 topics across containers**.

---

## High-Level Architecture

```
[ Intel D455 ]
       ↓
[ Perception Container ]
       ↓ (ROS2 topics)
[ Behavior Container ]
       ↓ (ROS2 topics)
[ IsaacLab Container ]
```

---

## Current Repository State Analysis

### What Currently Exists
| Component | Location | Status |
|-----------|----------|--------|
| Voxel Grid Node | `utils/voxel/ros2_voxel.py` | ⚠️ Prototype only (not a ROS2 package) |
| Voxel Dockerfile | `docker/controller/voxel_grid/voxel_grid.Dockerfile` | ✅ Has spconv/CUDA setup |
| Controller Compose | `modules/docker-compose.controller.yaml` | ⚠️ References missing package |
| Perception Compose | `modules_future/docker-compose.perception.yaml` | ❌ Empty file |
| Simulation Compose | `modules_future/docker-compose.simulation.yaml` | ❌ Empty file |
| Perception Package | `autonomy/perception_pkg/` | ❌ Empty structure |
| Sample Packages | `autonomy/samples/python/` | ✅ Reference implementation |

### What Needs to Be Created
| Component | Target Location | Priority |
|-----------|-----------------|----------|
| Behavior ROS2 Package | `autonomy/behavior/world_model/` | P0 |
| Perception ROS2 Package | `autonomy/perception/realsense_driver/` | P0 |
| Behavior Dockerfile | `docker/behavior/world_model.Dockerfile` | P0 |
| Perception Dockerfile | `docker/perception/realsense.Dockerfile` | P0 |
| Behavior Compose | `modules/docker-compose.behavior.yaml` | P0 |
| Perception Compose | `modules/docker-compose.perception.yaml` | P0 |
| Isaac Voxel Subscriber | `autonomy/simulation/Wato_additional_scripts/voxel_viz/` | P1 |
| Simulation Compose | `modules/docker-compose.simulation.yaml` | P1 |

---

## Container Responsibilities

### 1. Perception Container (Depth Camera)

**Purpose**
- Interface with the Intel D455
- Publish raw sensor data only

**Responsibilities**
- Run RealSense driver
- No voxelization
- No stateful world modeling

**Publishes**
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/depth/camera_info` (sensor_msgs/CameraInfo)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo)

**Notes**
- This container is hardware-bound
- Replaceable with sim or bag replay later

---

### 2. Behavior Container (Voxel / OctoMap)

**Purpose**
- Convert raw depth data into a spatial world representation

**Responsibilities**
- Subscribe to camera topics
- Generate voxel grid or octomap
- Maintain spatial representation (optionally temporal)

**Subscribes**
- `/camera/depth/image_raw`
- `/camera/color/image_raw`
- `/camera/depth/camera_info`

**Publishes**
- `/voxel_grid` (sensor_msgs/PointCloud2)

**Notes**
- This is world modeling, not perception
- Decoupled from sensor rate
- Feeds planning and simulation

---

### 3. IsaacLab Container (Visualization / Simulation)

**Purpose**
- Visualize the robot's perceived world
- Act as a simulation consumer

**Responsibilities**
- Subscribe to voxel or octomap topics
- Convert to Isaac-compatible primitives
- Render in simulation

**Subscribes**
- `/voxel_grid` (sensor_msgs/PointCloud2)

**Notes**
- No sensor drivers
- No voxel generation
- Visualization only

---

## Detailed Implementation Plan

### Phase 1: Behavior Container (World Model) — PRIORITY

This moves the existing `ros2_voxel.py` into a proper ROS2 package structure.

#### 1.1 Create Directory Structure

```
autonomy/
└── behavior/
    └── world_model/
        ├── world_model/
        │   ├── __init__.py
        │   ├── voxel_grid_core.py      # Core logic (no ROS2 deps)
        │   └── voxel_grid_node.py      # ROS2 node wrapper
        ├── config/
        │   └── voxel_params.yaml       # Configurable parameters
        ├── launch/
        │   └── world_model.launch.py
        ├── resource/
        │   └── world_model
        ├── test/
        │   └── test_voxel_grid.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

#### 1.2 Create `voxel_grid_core.py` (Core Logic)

```python
# autonomy/behavior/world_model/world_model/voxel_grid_core.py
"""
Voxel grid generation logic - no ROS2 dependencies.
This enables unit testing without ROS2 infrastructure.
"""
import numpy as np
import torch
from spconv.pytorch.utils import PointToVoxel
from typing import Tuple, Optional

class VoxelGridCore:
    """Core voxelization logic separated from ROS2 node."""
    
    def __init__(
        self,
        voxel_size: float = 0.04,
        max_range: float = 3.5,
        max_voxels: int = 10000,
        max_points_per_voxel: int = 20,
        downsample_step: int = 4
    ):
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.max_voxels = max_voxels
        self.max_points_per_voxel = max_points_per_voxel
        self.downsample_step = downsample_step
        
    def rgbd_to_pointcloud(
        self,
        depth: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float
    ) -> np.ndarray:
        """Convert depth image to 3D point cloud."""
        h, w = depth.shape
        points = []
        
        for v in range(0, h, self.downsample_step):
            for u in range(0, w, self.downsample_step):
                z = depth[v, u] / 1000.0  # mm to meters
                
                if 0.1 < z < self.max_range:
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        
        return np.array(points, dtype=np.float32)
    
    def create_voxel_grid(self, points: np.ndarray) -> np.ndarray:
        """Generate voxel grid from point cloud using spconv."""
        if len(points) == 0:
            return np.array([], dtype=np.float32)
            
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
            max_num_voxels=self.max_voxels,
            max_num_points_per_voxel=self.max_points_per_voxel
        )
        
        pc_tensor = torch.from_numpy(points_scaled)
        voxels, indices, num_points = voxel_gen(pc_tensor)
        
        voxel_centers = (
            indices.numpy() * self.voxel_size +
            np.array(coors_range[:3]) * self.voxel_size +
            self.voxel_size / 2
        )
        
        return voxel_centers
    
    def process_depth(
        self,
        depth: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float
    ) -> Tuple[np.ndarray, int, int]:
        """Full pipeline: depth image → voxel centers."""
        points = self.rgbd_to_pointcloud(depth, fx, fy, cx, cy)
        num_points = len(points)
        
        if num_points == 0:
            return np.array([]), 0, 0
            
        voxel_centers = self.create_voxel_grid(points)
        num_voxels = len(voxel_centers)
        
        return voxel_centers, num_points, num_voxels
```

#### 1.3 Create `voxel_grid_node.py` (ROS2 Node)

```python
# autonomy/behavior/world_model/world_model/voxel_grid_node.py
"""
ROS2 node wrapper for voxel grid generation.
Follows WATonomous Core/Node separation pattern.
"""
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from world_model.voxel_grid_core import VoxelGridCore


class VoxelGridNode(Node):
    """ROS2 node that converts RGBD to voxel grid."""

    def __init__(self):
        super().__init__('voxel_grid_node')
        
        # Declare parameters
        self.declare_parameter('voxel_size', 0.04)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('max_voxels', 10000)
        self.declare_parameter('max_points_per_voxel', 20)
        self.declare_parameter('downsample_step', 4)
        self.declare_parameter('frame_id', 'camera_link')
        
        # Initialize core logic with parameters
        self.core = VoxelGridCore(
            voxel_size=self.get_parameter('voxel_size').value,
            max_range=self.get_parameter('max_range').value,
            max_voxels=self.get_parameter('max_voxels').value,
            max_points_per_voxel=self.get_parameter('max_points_per_voxel').value,
            downsample_step=self.get_parameter('downsample_step').value
        )
        
        self.frame_id = self.get_parameter('frame_id').value
        self.bridge = CvBridge()
        
        # Camera intrinsics (updated from CameraInfo)
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)
        
        # Publisher
        self.voxel_pub = self.create_publisher(PointCloud2, '/voxel_grid', 10)
        
        self.get_logger().info('Voxel Grid Node initialized')

    def info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from CameraInfo."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id or self.frame_id

    def depth_callback(self, msg: Image):
        """Process depth image and publish voxel grid."""
        if self.fx is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=5.0)
            return
            
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        
        voxel_centers, num_points, num_voxels = self.core.process_depth(
            depth, self.fx, self.fy, self.cx, self.cy
        )
        
        if num_voxels > 0:
            self.publish_voxel_grid(voxel_centers, msg.header.stamp)
            self.get_logger().debug(f'Published {num_voxels} voxels from {num_points} points')

    def publish_voxel_grid(self, voxel_centers, stamp):
        """Publish voxel centers as PointCloud2."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        
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
```

#### 1.4 Create `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>world_model</name>
  <version>0.1.0</version>
  <description>Voxel grid world model for humanoid perception</description>
  <maintainer email="team@watonomous.ca">WATonomous</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>sensor_msgs_py</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 1.5 Create `setup.py`

```python
import os
from glob import glob
from setuptools import setup

package_name = 'world_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WATonomous',
    maintainer_email='team@watonomous.ca',
    description='Voxel grid world model for humanoid perception',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voxel_grid_node = world_model.voxel_grid_node:main',
        ],
    },
)
```

#### 1.6 Create `config/voxel_params.yaml`

```yaml
voxel_grid_node:
  ros__parameters:
    # Voxel grid parameters
    voxel_size: 0.04          # meters per voxel
    max_range: 3.5            # max depth in meters
    max_voxels: 10000         # maximum voxels to generate
    max_points_per_voxel: 20  # points averaged per voxel
    downsample_step: 4        # skip every N pixels for speed
    frame_id: "camera_link"   # TF frame for output
```

#### 1.7 Create `launch/world_model.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('world_model'),
        'config',
        'voxel_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='world_model',
            executable='voxel_grid_node',
            name='voxel_grid_node',
            output='screen',
            parameters=[config],
            emulate_tty=True,
        ),
    ])
```

#### 1.8 Create Dockerfile: `docker/behavior/world_model.Dockerfile`

```dockerfile
ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY autonomy/behavior/world_model world_model

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS dependencies
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    apt-get install -y --no-install-recommends $(cat /tmp/colcon_install_list) && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies for spconv
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip \
      python3-dev && \
    rm -rf /var/lib/apt/lists/*

# Install spconv and dependencies
# Note: Use spconv-cu120 for CUDA 12.x or spconv-cu118 for CUDA 11.8
RUN pip3 install --no-cache-dir \
      numpy \
      torch \
      spconv-cu120 \
      opencv-python

# Copy source
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

################################ Build ################################
FROM dependencies AS build

WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
```

#### 1.9 Create Docker Compose: `modules/docker-compose.behavior.yaml`

```yaml
services:
  world_model:
    build: &world_model_build
      context: ..
      dockerfile: docker/behavior/world_model.Dockerfile
      cache_from:
        - "${BEHAVIOR_WORLD_MODEL_IMAGE:?}:${TAG}"
        - "${BEHAVIOR_WORLD_MODEL_IMAGE:?}:main"
      args:
        BASE_IMAGE: ${BASE_IMAGE_OVERRIDE-}
    image: "${BEHAVIOR_WORLD_MODEL_IMAGE:?}:${TAG}"
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    profiles: [deploy]
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu]
    command: ros2 launch world_model world_model.launch.py
    networks:
      - ros_network

  world_model_dev:
    build: *world_model_build
    image: "${BEHAVIOR_WORLD_MODEL_IMAGE:?}:dev_${TAG}"
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    profiles: [develop]
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu]
    volumes:
      - ${MONO_DIR}/autonomy/behavior/world_model:/root/ament_ws/src/world_model
    command: tail -F anything
    networks:
      - ros_network

networks:
  ros_network:
    driver: bridge
```

---

### Phase 2: Perception Container (RealSense Driver)

#### 2.1 Create Directory Structure

```
autonomy/
└── perception/
    └── realsense_driver/
        ├── realsense_driver/
        │   ├── __init__.py
        │   └── realsense_node.py
        ├── config/
        │   └── d455_params.yaml
        ├── launch/
        │   └── realsense.launch.py
        ├── resource/
        │   └── realsense_driver
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

#### 2.2 Create `config/d455_params.yaml`

```yaml
# Intel RealSense D455 configuration
realsense_node:
  ros__parameters:
    serial_no: ""                    # Leave empty for auto-detect
    depth_module.profile: "640x480x30"
    rgb_camera.profile: "640x480x30"
    enable_depth: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    depth_module.enable_auto_exposure: true
    align_depth.enable: true         # Align depth to color frame
```

#### 2.3 Create `launch/realsense.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Use official RealSense ROS2 wrapper
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'align_depth.enable': 'true',
        }.items()
    )
    
    return LaunchDescription([
        realsense_launch,
    ])
```

#### 2.4 Create `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>realsense_driver</name>
  <version>0.1.0</version>
  <description>Intel RealSense D455 driver wrapper for humanoid</description>
  <maintainer email="team@watonomous.ca">WATonomous</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>realsense2_camera</exec_depend>
  <exec_depend>realsense2_description</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 2.5 Create Dockerfile: `docker/perception/realsense.Dockerfile`

```dockerfile
ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY autonomy/perception/realsense_driver realsense_driver

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

ENV DEBIAN_FRONTEND=noninteractive

# Install Intel RealSense SDK and ROS2 wrapper
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      software-properties-common \
      apt-transport-https && \
    apt-key adv --keyserver keyserver.ubuntu.com \
      --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      librealsense2-dkms \
      librealsense2-utils \
      librealsense2-dev \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-realsense2-description && \
    rm -rf /var/lib/apt/lists/*

# Copy source
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

################################ Build ################################
FROM dependencies AS build

WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
```

#### 2.6 Create Docker Compose: `modules/docker-compose.perception.yaml`

```yaml
services:
  realsense:
    build: &realsense_build
      context: ..
      dockerfile: docker/perception/realsense.Dockerfile
      cache_from:
        - "${PERCEPTION_REALSENSE_IMAGE:?}:${TAG}"
        - "${PERCEPTION_REALSENSE_IMAGE:?}:main"
      args:
        BASE_IMAGE: ${BASE_IMAGE_OVERRIDE-}
    image: "${PERCEPTION_REALSENSE_IMAGE:?}:${TAG}"
    privileged: true                 # Required for USB device access
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    profiles: [deploy]
    devices:
      - /dev/bus/usb:/dev/bus/usb    # USB passthrough for RealSense
    volumes:
      - /dev:/dev
    command: ros2 launch realsense_driver realsense.launch.py
    networks:
      - ros_network

  realsense_dev:
    build: *realsense_build
    image: "${PERCEPTION_REALSENSE_IMAGE:?}:dev_${TAG}"
    privileged: true
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    profiles: [develop]
    devices:
      - /dev/bus/usb:/dev/bus/usb
    volumes:
      - /dev:/dev
      - ${MONO_DIR}/autonomy/perception/realsense_driver:/root/ament_ws/src/realsense_driver
    command: tail -F anything
    networks:
      - ros_network

networks:
  ros_network:
    driver: bridge
```

---

### Phase 3: IsaacLab Voxel Visualization (Simulation Consumer)

#### 3.1 Create Directory Structure

```
autonomy/
└── simulation/
    └── Wato_additional_scripts/
        └── voxel_viz/
            ├── voxel_subscriber.py    # Subscribe to /voxel_grid
            └── isaac_voxel_render.py  # Render voxels in Isaac
```

#### 3.2 Create `voxel_subscriber.py`

```python
"""
ROS2 subscriber for voxel grid visualization in Isaac Sim.
Subscribes to /voxel_grid (PointCloud2) and renders in simulation.
"""
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Isaac Sim imports (available in Isaac container)
try:
    import omni
    from pxr import UsdGeom, Gf
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False


class VoxelVisualizerNode(Node):
    """Visualize voxel grid in Isaac Sim."""
    
    def __init__(self):
        super().__init__('voxel_visualizer')
        
        self.declare_parameter('voxel_size', 0.04)
        self.voxel_size = self.get_parameter('voxel_size').value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/voxel_grid',
            self.voxel_callback,
            10
        )
        
        self.get_logger().info('Voxel Visualizer started')
        
        if ISAAC_AVAILABLE:
            self.setup_isaac_stage()
    
    def setup_isaac_stage(self):
        """Initialize Isaac Sim stage for voxel rendering."""
        self.stage = omni.usd.get_context().get_stage()
        self.voxel_prim_path = "/World/VoxelGrid"
        
    def voxel_callback(self, msg: PointCloud2):
        """Process incoming voxel grid."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z")))
        
        if ISAAC_AVAILABLE:
            self.render_voxels_isaac(points)
        else:
            self.get_logger().info(f'Received {len(points)} voxels')
    
    def render_voxels_isaac(self, points):
        """Render voxels as cubes in Isaac Sim."""
        # Clear existing voxels
        if self.stage.GetPrimAtPath(self.voxel_prim_path):
            self.stage.RemovePrim(self.voxel_prim_path)
        
        # Create instance group for efficient rendering
        voxel_group = UsdGeom.Xform.Define(self.stage, self.voxel_prim_path)
        
        for i, (x, y, z) in enumerate(points[:5000]):  # Limit for performance
            cube_path = f"{self.voxel_prim_path}/voxel_{i}"
            cube = UsdGeom.Cube.Define(self.stage, cube_path)
            cube.GetSizeAttr().Set(self.voxel_size)
            cube.AddTranslateOp().Set(Gf.Vec3f(x, y, z))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.3 Update `modules/docker-compose.simulation.yaml` (or create in modules_future/)

```yaml
services:
  isaac_sim:
    build:
      context: ..
      dockerfile: docker/simulation/isaac_sim/isaac_sim.Dockerfile
      cache_from:
        - "${SIMULATION_ISAAC_IMAGE:?}:${TAG}"
        - "${SIMULATION_ISAAC_IMAGE:?}:main"
    image: "${SIMULATION_ISAAC_IMAGE:?}:${TAG}"
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    profiles: [deploy]
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu]
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ${MONO_DIR}/autonomy/simulation:/root/simulation
    networks:
      - ros_network

networks:
  ros_network:
    driver: bridge
```

---

### Phase 4: Environment Configuration Updates

#### 4.1 Update `modules/.env` (add these variables)

```bash
# Add to modules/.env or watod-config.local.sh
BEHAVIOR_WORLD_MODEL_IMAGE=ghcr.io/watonomous/humanoid/behavior/world_model
PERCEPTION_REALSENSE_IMAGE=ghcr.io/watonomous/humanoid/perception/realsense
SIMULATION_ISAAC_IMAGE=ghcr.io/watonomous/humanoid/simulation/isaac_sim
ROS_DOMAIN_ID=0
```

#### 4.2 Update `watod-config.local.sh`

```bash
# Enable the new modules
ACTIVE_MODULES="perception behavior infrastructure"

# For development with simulation
# ACTIVE_MODULES="perception behavior simulation infrastructure"

MODE_OF_OPERATION="develop"
```

---

## Communication Model

- All containers run on the **same Docker network** (`ros_network`)
- All containers share:
  - `ROS_DOMAIN_ID=0`
  - DDS implementation (FastDDS via ROS2 Humble default)

**ROS 2 DDS handles discovery and transport**
- No explicit ports needed
- No bridges required
- No direct container coupling

---

## Data Flow Summary

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Network                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐      ┌─────────────────┐                  │
│  │   Perception    │      │    Behavior     │                  │
│  │   Container     │      │   Container     │                  │
│  │                 │      │                 │                  │
│  │  [RealSense]    │─────▶│ [VoxelGrid]     │                  │
│  │                 │      │                 │                  │
│  └─────────────────┘      └────────┬────────┘                  │
│                                    │                            │
│  Topics:                           │  Topics:                   │
│  /camera/depth/image_raw           │  /voxel_grid               │
│  /camera/color/image_raw           │                            │
│  /camera/depth/camera_info         ▼                            │
│                           ┌─────────────────┐                  │
│                           │   IsaacLab      │                  │
│                           │   Container     │                  │
│                           │                 │                  │
│                           │ [VoxelRender]   │                  │
│                           │                 │                  │
│                           └─────────────────┘                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## File Checklist

### Files to Create

| # | File Path | Purpose |
|---|-----------|---------|
| 1 | `autonomy/behavior/world_model/world_model/__init__.py` | Package init |
| 2 | `autonomy/behavior/world_model/world_model/voxel_grid_core.py` | Core voxel logic |
| 3 | `autonomy/behavior/world_model/world_model/voxel_grid_node.py` | ROS2 node |
| 4 | `autonomy/behavior/world_model/config/voxel_params.yaml` | Parameters |
| 5 | `autonomy/behavior/world_model/launch/world_model.launch.py` | Launch file |
| 6 | `autonomy/behavior/world_model/resource/world_model` | Ament marker |
| 7 | `autonomy/behavior/world_model/package.xml` | ROS2 package manifest |
| 8 | `autonomy/behavior/world_model/setup.py` | Python setup |
| 9 | `autonomy/behavior/world_model/setup.cfg` | Setup config |
| 10 | `docker/behavior/world_model.Dockerfile` | Behavior container |
| 11 | `modules/docker-compose.behavior.yaml` | Behavior compose |
| 12 | `autonomy/perception/realsense_driver/...` | Perception package |
| 13 | `docker/perception/realsense.Dockerfile` | Perception container |
| 14 | `modules/docker-compose.perception.yaml` | Perception compose |
| 15 | `autonomy/simulation/Wato_additional_scripts/voxel_viz/...` | Isaac viz |

### Files to Update

| File | Change |
|------|--------|
| `modules/.env` | Add new image variables |
| `watod-config.local.sh` | Update `ACTIVE_MODULES` |

---

## Testing Plan

### Unit Tests (No ROS2)
```bash
# Test core voxel logic
cd autonomy/behavior/world_model
python3 -m pytest test/
```

### Integration Test (With ROS2)
```bash
# Terminal 1: Start perception (or use bag file)
watod up perception

# Terminal 2: Start behavior
watod up behavior

# Terminal 3: Verify topics
watod -t world_model_dev
ros2 topic echo /voxel_grid

# Terminal 4: Visualize in Foxglove
watod up infrastructure
# Open http://localhost:8765 in browser
```

### Bag File Testing (No Hardware)
```bash
# Record a bag with real camera
ros2 bag record /camera/depth/image_raw /camera/depth/camera_info -o test_depth

# Replay for testing
ros2 bag play test_depth --loop
```

---

## Quick Start Commands

### Build Everything
```bash
# 1. Setup environment (if not done)
./watod_scripts/watod-setup-docker-env.sh

# 2. Update watod-config.local.sh
cp watod-config.sh watod-config.local.sh
# Edit: ACTIVE_MODULES="perception behavior infrastructure"
# Edit: MODE_OF_OPERATION="develop"

# 3. Build and start
watod up --build
```

### Development Workflow
```bash
# Enter behavior container for development
watod -t world_model_dev

# Inside container: rebuild after code changes
cd /root/ament_ws
colcon build --packages-select world_model
source install/setup.bash

# Run the node
ros2 launch world_model world_model.launch.py
```

### Verify ROS2 Communication
```bash
# List all topics
ros2 topic list

# Check voxel grid output
ros2 topic echo /voxel_grid --once

# Monitor topic frequency
ros2 topic hz /voxel_grid
```

---

## Design Rules

- **Perception** = facts from sensors (raw data only)
- **Behavior** = belief about the world (processed representations)
- **Simulation** = visualization + testing (consumers only)
- Infrastructure never touches autonomy code
- Containers communicate only via topics

---

## Outcome

- ✅ Clean separation of concerns
- ✅ Scalable to multiple sensors
- ✅ Replayable and simulatable
- ✅ Matches industry humanoid architecture
- ✅ Follows WATonomous Core/Node pattern
- ✅ GPU-accelerated voxelization with spconv
