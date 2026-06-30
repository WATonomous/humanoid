from collections import deque
import os
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from tracknetv3.inference import TrackNetV3Inference, TrackNetV3Prediction
from tracknetv3.utils import FrameSequenceBuffer, format_stats_json


class TrackNetV3Node(Node):
    """ROS2 node wrapping online TrackNetV3 inference."""

    def __init__(self):
        super().__init__("tracknetv3_node")
        self.bridge = CvBridge()
        self._declare_parameters()

        rgb_topic = self._get_str("camera.rgb_image_topic")
        self.camera_frame = self._get_str("camera.camera_frame")
        tracknet_checkpoint = self._resolve_checkpoint(
            self._get_str("model.tracknet_checkpoint"),
            self._get_str("model.model_dir"),
        )
        inpaintnet_checkpoint = self._resolve_optional_checkpoint(
            self._get_str("model.inpaintnet_checkpoint"),
            self._get_str("model.model_dir"),
        )

        self.inference = TrackNetV3Inference(
            tracknet_checkpoint=tracknet_checkpoint,
            inpaintnet_checkpoint=inpaintnet_checkpoint,
            device=self._get_str("model.device"),
            use_fp16=self._get_bool("model.use_fp16"),
            seq_len=self._get_int("model.seq_len"),
            bg_mode=self._get_str("model.bg_mode"),
            confidence_threshold=self._get_float("inference.confidence_threshold"),
        )
        self.frame_buffer = FrameSequenceBuffer(self.inference.seq_len)
        self.debug_enabled = self._get_bool("debug.enabled")
        self.draw_trajectory = self._get_bool("debug.draw_trajectory")
        self.trajectory = deque(maxlen=self._get_int("debug.trajectory_length"))
        self.latest_prediction: Optional[TrackNetV3Prediction] = None
        self.frames_seen = 0

        self.point_pub = self.create_publisher(
            PointStamped,
            self._get_str("publishers.point_topic"),
            10,
        )
        self.visible_pub = self.create_publisher(
            Bool,
            self._get_str("publishers.visible_topic"),
            10,
        )
        self.stats_pub = self.create_publisher(
            String,
            self._get_str("publishers.stats_topic"),
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            self._get_str("publishers.debug_image_topic"),
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.stats_timer = self.create_timer(1.0, self.publish_stats)

        self.get_logger().info(
            "TrackNetV3 node started with "
            f"device={self.inference.device}, seq_len={self.inference.seq_len}, "
            f"bg_mode={self.inference.bg_mode}, rgb_topic={rgb_topic}"
        )

    def image_callback(self, msg: Image) -> None:
        """Convert an image, run inference when buffered, and publish results."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        self.frames_seen += 1
        sequence = self.frame_buffer.append(cv_image)
        if sequence is None:
            return

        try:
            prediction = self.inference.predict(sequence)
        except Exception as exc:
            self.get_logger().error(f"TrackNetV3 inference failed: {exc}")
            return

        self.latest_prediction = prediction
        if prediction.visible:
            self.trajectory.append((prediction.x, prediction.y))
        else:
            self.trajectory.append(None)

        self.publish_point(prediction, msg.header)
        self.publish_visible(prediction)
        if self.debug_enabled:
            self.publish_debug_image(cv_image, prediction, msg.header)

    def publish_point(self, prediction: TrackNetV3Prediction, header) -> None:
        """Publish the latest image-plane shuttle point."""
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.header.frame_id = header.frame_id or self.camera_frame
        point_msg.point.x = float(prediction.x)
        point_msg.point.y = float(prediction.y)
        point_msg.point.z = 0.0
        self.point_pub.publish(point_msg)

    def publish_visible(self, prediction: TrackNetV3Prediction) -> None:
        """Publish whether the shuttle is visible in the latest frame."""
        msg = Bool()
        msg.data = bool(prediction.visible)
        self.visible_pub.publish(msg)

    def publish_debug_image(
        self,
        image,
        prediction: TrackNetV3Prediction,
        header,
    ) -> None:
        """Publish an annotated image for visual inspection."""
        annotated = image.copy()
        if self.draw_trajectory:
            for point in self.trajectory:
                if point is not None:
                    cv2.circle(annotated, point, 3, (255, 255, 255), -1)
        if prediction.visible:
            cv2.circle(annotated, (prediction.x, prediction.y), 6, (0, 0, 255), 2)
            label = f"{prediction.source}:{prediction.confidence:.2f}"
            cv2.putText(
                annotated,
                label,
                (prediction.x + 8, max(16, prediction.y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)

    def publish_stats(self) -> None:
        """Publish compact runtime stats."""
        prediction = self.latest_prediction
        stats = String()
        stats.data = format_stats_json(
            active=prediction is not None,
            device=str(self.inference.device),
            frames_seen=self.frames_seen,
            inference_ms=prediction.inference_ms if prediction else 0.0,
            source=prediction.source if prediction else "",
            visible=prediction.visible if prediction else False,
        )
        self.stats_pub.publish(stats)

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera.rgb_image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera.camera_frame", "camera_link")
        self.declare_parameter("model.model_dir", "")
        self.declare_parameter("model.tracknet_checkpoint", "TrackNet_best.pt")
        self.declare_parameter("model.inpaintnet_checkpoint", "")
        self.declare_parameter("model.device", "cuda:0")
        self.declare_parameter("model.use_fp16", True)
        self.declare_parameter("model.seq_len", 8)
        self.declare_parameter("model.bg_mode", "concat")
        self.declare_parameter("inference.confidence_threshold", 0.5)
        self.declare_parameter("publishers.point_topic", "/tracknetv3/shuttle_point")
        self.declare_parameter("publishers.visible_topic", "/tracknetv3/shuttle_visible")
        self.declare_parameter("publishers.debug_image_topic", "/tracknetv3/debug_image")
        self.declare_parameter("publishers.stats_topic", "/tracknetv3/stats")
        self.declare_parameter("debug.enabled", False)
        self.declare_parameter("debug.draw_trajectory", True)
        self.declare_parameter("debug.trajectory_length", 8)

    def _resolve_checkpoint(self, checkpoint: str, model_dir: str) -> Path:
        path = self._resolve_optional_checkpoint(checkpoint, model_dir)
        if path is None:
            raise FileNotFoundError(
                "model.tracknet_checkpoint is required. Mount TrackNet_best.pt "
                "and set model.model_dir or pass an absolute checkpoint path."
            )
        return path

    def _resolve_optional_checkpoint(self, checkpoint: str, model_dir: str) -> Optional[Path]:
        if not checkpoint:
            return None
        path = Path(checkpoint).expanduser()
        if path.is_absolute():
            return path
        if model_dir:
            return Path(model_dir).expanduser() / path
        env_model_dir = os.environ.get("TRACKNETV3_MODEL_DIR")
        if env_model_dir:
            return Path(env_model_dir).expanduser() / path
        source_model_dir = Path(__file__).resolve().parents[1] / "models" / "weights"
        return source_model_dir / path

    def _get_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_bool(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_int(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None):
    rclpy.init(args=args)
    node = TrackNetV3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
