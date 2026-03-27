import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math

JOINT_FILE = "/tmp/wato_joints.json"

# Calibration constants for finger curl:
#   OPEN_RATIO  — tip_dist/mcp_dist when finger is fully extended (~2.2)
#   CLOSE_RATIO — tip_dist/mcp_dist when finger is in a tight fist  (~0.8)
# Curl=0 at OPEN_RATIO, curl=1 at CLOSE_RATIO, clamped to [0,1].
OPEN_RATIO  = 2.2
CLOSE_RATIO = 0.8

def finger_curl(landmarks, tip_idx, mcp_idx):
    tip = landmarks[tip_idx]
    mcp = landmarks[mcp_idx]
    wrist = landmarks[0]
    tip_dist = math.sqrt((tip["x"]-wrist["x"])**2 + (tip["y"]-wrist["y"])**2 + (tip["z"]-wrist["z"])**2)
    mcp_dist = math.sqrt((mcp["x"]-wrist["x"])**2 + (mcp["y"]-wrist["y"])**2 + (mcp["z"]-wrist["z"])**2)
    if mcp_dist == 0:
        return 0.0
    ratio = tip_dist / mcp_dist
    curl = 1.0 - min(max((ratio - CLOSE_RATIO) / (OPEN_RATIO - CLOSE_RATIO), 0.0), 1.0)
    return curl

def landmarks_to_joints(landmarks):
    index_curl  = finger_curl(landmarks, tip_idx=8,  mcp_idx=5)
    middle_curl = finger_curl(landmarks, tip_idx=12, mcp_idx=9)
    ring_curl   = 1.0 - finger_curl(landmarks, tip_idx=16, mcp_idx=13)  # inverted
    pinky_curl  = 1.0 - finger_curl(landmarks, tip_idx=20, mcp_idx=17) # inverted
    thumb_curl  = finger_curl(landmarks, tip_idx=4,  mcp_idx=2)
    return {
        "mcp_index":  -1.57 * index_curl,
        "pip_index":   2.09 * index_curl,   # widened to 120°
        "dip_index":  -2.09 * index_curl,   # widened to 120°
        "mcp_middle": -1.57 * middle_curl,
        "pip_middle":  2.09 * middle_curl,  # widened to 120°
        "dip_middle":  2.09 * middle_curl,  # widened to 120°
        "mcp_ring":    1.57 * ring_curl,
        "pip_ring":   -2.09 * ring_curl,    # widened to 120°
        "dip_ring":   -2.09 * ring_curl,    # widened to 120°
        "mcp_pinky":   1.57 * pinky_curl,
        "pip_pinky":  -2.09 * pinky_curl,   # widened to 120°
        "dip_pinky":   2.09 * pinky_curl,   # widened to 120°
        "cmc_thumb":  -0.35 + 2.44 * thumb_curl,
        "mcp_thumb":   0.785 + 1.745 * thumb_curl,
        "ip_thumb":    1.57 * thumb_curl,
    }

class WatoHandNode(Node):
    def __init__(self):
        super().__init__("wato_hand_node")
        self.get_logger().info("WatoHandNode started, waiting for landmarks...")
        self.sub = self.create_subscription(String, "/wato/hand_landmarks", self.landmarks_callback, 10)
        self.joint_pub = self.create_publisher(String, "/wato/hand_joint_angles", 10)

    def landmarks_callback(self, msg):
        try:
            landmarks = json.loads(msg.data)
            if len(landmarks) != 21:
                return
            joint_dict = landmarks_to_joints(landmarks)
            self.get_logger().info(f"Joint angles: {joint_dict}")
            out_msg = String()
            out_msg.data = json.dumps(joint_dict)
            self.joint_pub.publish(out_msg)
            # Write to shared file so Isaac Lab can read it without rclpy
            with open(JOINT_FILE, "w") as f:
                json.dump(joint_dict, f)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = WatoHandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()