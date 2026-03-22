import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from wato_dex_retargeting_utils import WatoHandDexRetargeting

WATO_HAND_JOINT_NAMES = [
    "mcp_index", "pip_index", "dip_index",
    "mcp_middle", "pip_middle", "dip_middle",
    "mcp_ring", "pip_ring", "dip_ring",
    "mcp_pinky", "pip_pinky", "dip_pinky",
    "cmc_thumb", "mcp_thumb", "ip_thumb",
]

def mediapipe_to_openxr(landmarks):
    """Convert 21 MediaPipe landmarks to OpenXR-style pose dict."""
    mp = np.array([[lm["x"], lm["y"], lm["z"]] for lm in landmarks])

    hand_poses = {}

    # Fill all indices with zeros first
    for i in range(26):
        hand_poses[i] = np.zeros(3, dtype=np.float64)

    # Map MediaPipe joints to OpenXR indices
    hand_poses[1]  = mp[0].copy()   # wrist
    hand_poses[2]  = mp[1].copy()   # thumb cmc
    hand_poses[3]  = mp[2].copy()   # thumb mcp
    hand_poses[4]  = mp[3].copy()   # thumb ip
    hand_poses[5]  = mp[4].copy()   # thumb tip
    hand_poses[7]  = mp[5].copy()   # index mcp
    hand_poses[8]  = mp[6].copy()   # index pip
    hand_poses[9]  = mp[7].copy()   # index dip
    hand_poses[10] = mp[8].copy()   # index tip
    hand_poses[12] = mp[9].copy()   # middle mcp
    hand_poses[13] = mp[10].copy()  # middle pip
    hand_poses[14] = mp[11].copy()  # middle dip
    hand_poses[15] = mp[12].copy()  # middle tip
    hand_poses[17] = mp[13].copy()  # ring mcp
    hand_poses[18] = mp[14].copy()  # ring pip
    hand_poses[19] = mp[15].copy()  # ring dip
    hand_poses[20] = mp[16].copy()  # ring tip
    hand_poses[22] = mp[17].copy()  # pinky mcp
    hand_poses[23] = mp[18].copy()  # pinky pip
    hand_poses[24] = mp[19].copy()  # pinky dip
    hand_poses[25] = mp[20].copy()  # pinky tip

    # Wrist quaternion (identity since MediaPipe is already wrist-relative)
    wrist_xyz = mp[0]
    hand_poses["wrist"] = np.array(
        [wrist_xyz[0], wrist_xyz[1], wrist_xyz[2], 1.0, 0.0, 0.0, 0.0],
        dtype=np.float64
    )

    return hand_poses


class WatoHandNode(Node):
    def __init__(self):
        super().__init__("wato_hand_node")
        self.dex = WatoHandDexRetargeting(hand_joint_names=WATO_HAND_JOINT_NAMES)
        self.get_logger().info("WatoHandNode started, waiting for landmarks...")

        self.sub = self.create_subscription(
            String,
            "/wato/hand_landmarks",
            self.landmarks_callback,
            10
        )

        # Publisher for retargeted joint angles
        self.joint_pub = self.create_publisher(
            String,
            "/wato/hand_joint_angles",
            10
        )

    def landmarks_callback(self, msg):
        try:
            landmarks = json.loads(msg.data)
            if len(landmarks) != 21:
                return

            hand_poses = mediapipe_to_openxr(landmarks)
            right_q = self.dex.compute_right(hand_poses)

            joint_dict = dict(zip(WATO_HAND_JOINT_NAMES, right_q.tolist()))

            self.get_logger().info(f"Joint angles: {joint_dict}")

            # Publish joint angles
            out_msg = String()
            out_msg.data = json.dumps(joint_dict)
            self.joint_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = WatoHandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()