# TrackNetV3 ROS2 Module

ROS2 wrapper for TrackNetV3 shuttle trajectory tracking. The node subscribes to
RGB camera images, buffers a sequence of frames, runs TrackNetV3 inference, and
publishes the current shuttle point, visibility flag, optional debug image, and
runtime stats.

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
- `InpaintNet_best.pt` optional

The launch file also accepts absolute checkpoint paths or a mounted model
directory:

```bash
ros2 launch tracknetv3 tracknetv3.launch.py \
  model_dir:=/models/tracknetv3 \
  tracknet_checkpoint:=TrackNet_best.pt \
  inpaintnet_checkpoint:=InpaintNet_best.pt
```

If CUDA is requested but unavailable, inference falls back to CPU.

## Docker

```bash
docker compose -f modules/docker-compose.tracknetv3.yaml --profile develop build
docker compose -f modules/docker-compose.tracknetv3.yaml --profile develop up
```
