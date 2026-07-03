# TrackNetV3 ROS2 Module

ROS2 wrapper for TrackNetV3 shuttle trajectory tracking. The node subscribes to
RGB camera images, buffers a sequence of frames, runs TrackNetV3 inference, and
publishes the current shuttle point, visibility flag, optional debug image, and
runtime stats.

This package is the v1 2D pixel detector. It publishes image-plane `(x, y)`
locations only; depth fusion, 3D trajectory estimation, and impact prediction are
downstream tasks. The full upstream TrackNetV3 repository is not required at
runtime. This package carries the adapted model definitions needed for inference,
and local checkpoint files are mounted separately.

## Topics

Subscribed:

- `/camera/color/image_raw` (`sensor_msgs/Image`)

Published:

- `/tracknetv3/shuttle_point` (`geometry_msgs/PointStamped`)
- `/tracknetv3/shuttle_visible` (`std_msgs/Bool`)
- `/tracknetv3/debug_image` (`sensor_msgs/Image`)
- `/tracknetv3/stats` (`std_msgs/String`, compact JSON)

## Checkpoints

Place checkpoints in `autonomy/tracknetv3/models/weights/`:

- `TrackNet_best.pt`
- `InpaintNet_best.pt` optional and experimental for this ROS wrapper

The launch file also accepts absolute checkpoint paths or a mounted model
directory:

```bash
ros2 launch tracknetv3 tracknetv3.launch.py \
  model_dir:=/models/tracknetv3 \
  tracknet_checkpoint:=TrackNet_best.pt \
  inpaintnet_checkpoint:=InpaintNet_best.pt
```

If CUDA is requested but unavailable, inference falls back to CPU.

## Runtime Dependencies

The Docker image installs PyTorch explicitly so the CUDA wheel can match the
runtime image. For non-Docker use, install a compatible PyTorch build before
running `ros2 run tracknetv3 tracknetv3_node` or launching this package. The
Python package metadata also lists `torch` for standard Python environments, but
ROS `rosdep` does not choose CUDA-specific PyTorch wheels.

## Docker

```bash
docker compose -f modules/docker-compose.tracknetv3.yaml --profile develop build
docker compose -f modules/docker-compose.tracknetv3.yaml --profile develop up
```
