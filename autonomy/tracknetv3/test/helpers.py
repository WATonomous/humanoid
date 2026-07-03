import sys
import types


def install_inference_stubs():
    """Install lightweight dependency stubs for inference unit tests."""
    import numpy as np

    cv2 = types.ModuleType("cv2")
    cv2.COLOR_BGR2RGB = 1
    cv2.RETR_EXTERNAL = 0
    cv2.CHAIN_APPROX_SIMPLE = 0

    def cvt_color(image, _code):
        return image[:, :, ::-1].copy()

    def find_contours(binary, _mode, _method):
        ys, xs = np.nonzero(binary)
        if len(xs) == 0:
            return [], None
        return [(xs, ys)], None

    def bounding_rect(contour):
        xs, ys = contour
        min_x = int(xs.min())
        max_x = int(xs.max())
        min_y = int(ys.min())
        max_y = int(ys.max())
        return min_x, min_y, max_x - min_x + 1, max_y - min_y + 1

    cv2.cvtColor = cvt_color
    cv2.findContours = find_contours
    cv2.boundingRect = bounding_rect
    sys.modules.setdefault("cv2", cv2)

    torch = types.ModuleType("torch")

    class Device:
        def __init__(self, requested):
            self.requested = requested
            self.type = str(requested).split(":", maxsplit=1)[0]

        def __str__(self):
            return self.requested

    class NoGrad:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, traceback):
            return False

    torch.device = Device
    torch.no_grad = NoGrad
    sys.modules.setdefault("torch", torch)

    model = types.ModuleType("tracknetv3.model")

    class InpaintNet:
        pass

    def create_tracknet(*_args, **_kwargs):
        raise AssertionError("create_tracknet should not be called by these tests")

    model.InpaintNet = InpaintNet
    model.create_tracknet = create_tracknet
    sys.modules["tracknetv3.model"] = model


def install_ros_stubs():
    """Install lightweight ROS message/module stubs for node unit tests."""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        def __init__(self, *_args, **_kwargs):
            pass

    rclpy_node.Node = Node
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sys.modules.setdefault("rclpy.qos", rclpy_qos)

    cv_bridge = types.ModuleType("cv_bridge")

    class CvBridge:
        pass

    cv_bridge.CvBridge = CvBridge
    sys.modules.setdefault("cv_bridge", cv_bridge)

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class PointStamped:
        def __init__(self):
            self.header = None
            self.point = Point()

    geometry_msgs_msg.PointStamped = PointStamped
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class Image:
        pass

    sensor_msgs_msg.Image = Image
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class Bool:
        def __init__(self):
            self.data = False

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = String
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)
