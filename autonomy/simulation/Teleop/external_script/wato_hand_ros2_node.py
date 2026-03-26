import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
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
    # Ensure numeric keys are ordered 0-25, wrist key last
    hand_poses = dict(sorted({k: v for k, v in hand_poses.items() if isinstance(k, int)}.items()))
    hand_poses["wrist"] = np.array(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
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

    def finger_curl(landmarks, tip_idx, mcp_idx):
        """Returns 0.0 (open) to 1.0 (fully curled) for a finger."""
        tip = landmarks[tip_idx]
        mcp = landmarks[mcp_idx]
        wrist = landmarks[0]
        # Distance tip->wrist vs mcp->wrist
        tip_dist = math.sqrt((tip["x"]-wrist["x"])**2 + (tip["y"]-wrist["y"])**2 + (tip["z"]-wrist["z"])**2)
        mcp_dist = math.sqrt((mcp["x"]-wrist["x"])**2 + (mcp["y"]-wrist["y"])**2 + (mcp["z"]-wrist["z"])**2)
        curl = 1.0 - min(max(tip_dist / (mcp_dist * 2.2), 0.0), 1.0)
        return curl

    def landmarks_to_joints(landmarks):
        """Map MediaPipe 21 landmarks directly to 15 robot joint angles."""
        import math

        # Finger curl values 0=open, 1=closed
        # MediaPipe indices: thumb=4,mcp=2 | index=8,mcp=5 | middle=12,mcp=9 | ring=16,mcp=13 | pinky=20,mcp=17
        index_curl  = finger_curl(landmarks, tip_idx=8,  mcp_idx=5)
        middle_curl = finger_curl(landmarks, tip_idx=12, mcp_idx=9)
        ring_curl   = finger_curl(landmarks, tip_idx=16, mcp_idx=13)
        pinky_curl  = finger_curl(landmarks, tip_idx=20, mcp_idx=17)
        thumb_curl  = finger_curl(landmarks, tip_idx=4,  mcp_idx=2)

        # Map curl to joint angles within URDF limits:
        # mcp_index:  lower=-1.57, upper=0.0  → curl maps to -1.57*curl
        # pip_index:  lower=0.0,   upper=1.57 → curl maps to 1.57*curl  (opposite direction)
        # dip_index:  lower=-1.57, upper=0.0  → curl maps to -1.57*curl
        # same pattern for middle
        # mcp_ring:   lower=0.0,   upper=1.57 → curl maps to 1.57*curl
        # pip_ring:   lower=-1.57, upper=0.0
        # dip_ring:   lower=-1.57, upper=0.0
        # mcp_pinky:  lower=0.0,   upper=1.57
        # pip_pinky:  lower=-1.57, upper=0.0
        # dip_pinky:  lower=0.0,   upper=1.57
        # cmc_thumb:  lower=-0.35, upper=2.09
        # mcp_thumb:  lower=0.785, upper=2.53
        # ip_thumb:   lower=0.0,   upper=1.57

        return {
            "mcp_index":  -1.57 * index_curl,
            "pip_index":   1.57 * index_curl,
            "dip_index":  -1.57 * index_curl,
            "mcp_middle": -1.57 * middle_curl,
            "pip_middle": -1.57 * middle_curl,
            "dip_middle":  1.57 * middle_curl,
            "mcp_ring":    1.57 * ring_curl,
            "pip_ring":   -1.57 * ring_curl,
            "dip_ring":   -1.57 * ring_curl,
            "mcp_pinky":   1.57 * pinky_curl,
            "pip_pinky":  -1.57 * pinky_curl,
            "dip_pinky":   1.57 * pinky_curl,
            "cmc_thumb":  -0.35 + 2.44 * thumb_curl,
            "mcp_thumb":   0.785 + 1.745 * thumb_curl,
            "ip_thumb":    1.57 * thumb_curl,
        }

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

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = WatoHandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()