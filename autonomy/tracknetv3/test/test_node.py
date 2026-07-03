from collections import deque
import unittest
from importlib.util import find_spec

if find_spec("numpy") is None or find_spec("PIL") is None:
    raise unittest.SkipTest("TrackNetV3 node tests require numpy and Pillow")

from helpers import install_inference_stubs, install_ros_stubs

install_inference_stubs()
install_ros_stubs()

from tracknetv3.inference import TrackNetV3Prediction
from tracknetv3.tracknetv3_node import TrackNetV3Node


class Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = object()


class ImageMsg:
    def __init__(self):
        self.header = Header()


class FakeBridge:
    def imgmsg_to_cv2(self, _msg, desired_encoding):
        if desired_encoding != "bgr8":
            raise AssertionError("TrackNetV3Node must request bgr8 images")
        return "cv-image"


class FakeFrameBuffer:
    def append(self, frame):
        if frame != "cv-image":
            raise AssertionError("Unexpected frame passed to sequence buffer")
        return ["frame-0", "frame-1"]


class FakeInference:
    def predict(self, sequence):
        if sequence != ["frame-0", "frame-1"]:
            raise AssertionError("Unexpected sequence passed to inference")
        return TrackNetV3Prediction(
            x=123,
            y=45,
            visible=True,
            confidence=0.91,
            inference_ms=4.2,
            source="tracknet",
        )


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestTrackNetV3Node(unittest.TestCase):

    def test_image_callback_publishes_pixel_point_and_visibility(self):
        node = object.__new__(TrackNetV3Node)
        node.bridge = FakeBridge()
        node.frame_buffer = FakeFrameBuffer()
        node.inference = FakeInference()
        node.latest_prediction = None
        node.frames_seen = 0
        node.trajectory = deque(maxlen=8)
        node.debug_enabled = False
        node.camera_frame = "camera_link"
        node.point_pub = FakePublisher()
        node.visible_pub = FakePublisher()

        node.image_callback(ImageMsg())

        self.assertEqual(node.frames_seen, 1)
        self.assertEqual(len(node.point_pub.messages), 1)
        point_msg = node.point_pub.messages[0]
        self.assertEqual(point_msg.header.frame_id, "camera_link")
        self.assertEqual(point_msg.point.x, 123.0)
        self.assertEqual(point_msg.point.y, 45.0)
        self.assertEqual(point_msg.point.z, 0.0)
        self.assertEqual(len(node.visible_pub.messages), 1)
        self.assertTrue(node.visible_pub.messages[0].data)


if __name__ == "__main__":
    unittest.main()
