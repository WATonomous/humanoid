import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time

JOINT_FILE = "/tmp/wato_joints.json"

# ── Finger curl calibration ───────────────────────────────────────────────────
# OPEN_RATIOS — tip/mcp distance ratio when finger is fully extended
# CLOSE_RATIO — tip/mcp distance ratio when finger is in a tight fist
OPEN_RATIOS = {
    5: 1.3,   # Index
    9: 1.3,   # Middle
    13: 1.2,  # Ring
    17: 1.2,  # Pinky
}
CLOSE_RATIO = 0.8

# ── Arm position calibration ──────────────────────────────────────────────────
# Scale factors: metres of hand movement → radians of joint movement
# Tune these to feel comfortable for your arm range of motion
SHOULDER_FE_SCALE  = 4.0   # y-axis (up/down)    → shoulder_flexion_extension
SHOULDER_AA_SCALE  = 4.0   # x-axis (left/right) → shoulder_abduction_adduction
ELBOW_FE_SCALE     = 4.0   # z-axis (depth)       → elbow_flexion_extension

# URDF joint limits for arm joints (rad)
SHOULDER_FE_LIMITS  = (-0.349, 3.491)
SHOULDER_AA_LIMITS  = (-0.349, 3.491)
ELBOW_FE_LIMITS     = (-0.349, 3.491)

# Calibration state (filled during 15-frame startup hold)
_arm_ref    = None
CALIB_FRAMES = 15

# EMA smoothing state
_smoothed_joints = {}
SMOOTH_ALPHA = 0.85  # Increased from 0.35 to 0.85 to remove slow lag

def clamp(v, lo, hi):
    return max(lo, min(hi, v))




def finger_curl(landmarks, tip_idx, mcp_idx):
    """Legacy single-scalar curl (still used for thumb)."""
    tip   = landmarks[tip_idx]
    mcp   = landmarks[mcp_idx]
    wrist = landmarks[0]
    tip_dist = math.sqrt((tip["x"]-wrist["x"])**2 + (tip["y"]-wrist["y"])**2 + (tip["z"]-wrist["z"])**2)
    mcp_dist = math.sqrt((mcp["x"]-wrist["x"])**2 + (mcp["y"]-wrist["y"])**2 + (mcp["z"]-wrist["z"])**2)
    if mcp_dist == 0:
        return 0.0
    ratio = tip_dist / mcp_dist
    open_r = OPEN_RATIOS.get(mcp_idx, 1.85)
    curl  = 1.0 - clamp((ratio - CLOSE_RATIO) / (open_r - CLOSE_RATIO), 0.0, 1.0)
    return curl


def palm_normal(world):
    """
    Compute a unit vector pointing out of the palm face.
    Uses world landmarks (metric, wrist-centred).
    Cross product of (wrist→index_mcp) × (wrist→pinky_mcp).
    """
    wrist    = world[0]
    idx_mcp  = world[5]
    pnk_mcp  = world[17]
    ax = idx_mcp["x"] - wrist["x"]; ay = idx_mcp["y"] - wrist["y"]; az = idx_mcp["z"] - wrist["z"]
    bx = pnk_mcp["x"] - wrist["x"]; by = pnk_mcp["y"] - wrist["y"]; bz = pnk_mcp["z"] - wrist["z"]
    nx = ay*bz - az*by
    ny = az*bx - ax*bz
    nz = ax*by - ay*bx
    mag = math.sqrt(nx**2 + ny**2 + nz**2) or 1.0
    return {"nx": nx/mag, "ny": ny/mag, "nz": nz/mag}


def arm_joints_from_camera(landmarks):
    """
    Map absolute 2D image position of the wrist directly to shoulder joints.
    This prevents the arm from jerking/sliding forwards when closing a fist changes the MediaPipe 3D Z-scale.
    """
    global _arm_ref
    wrist = landmarks[0]
    hx, hy = wrist["x"], wrist["y"]

    if _arm_ref is None:
        _arm_ref = {"hx": hx, "hy": hy, "count": 1}
        return None   # still calibrating
    elif _arm_ref["count"] < CALIB_FRAMES:
        n = _arm_ref["count"]
        _arm_ref["hx"] = (_arm_ref["hx"] * n + hx) / (n + 1)
        _arm_ref["hy"] = (_arm_ref["hy"] * n + hy) / (n + 1)
        _arm_ref["count"] += 1
        return None   # still calibrating

    dx = hx - _arm_ref["hx"]   # left/right
    dy = hy - _arm_ref["hy"]   # up/down

    # Disable shoulder and elbow motions per user request, keep them locked at neutral (0.0)
    shoulder_fe  = 0.0
    shoulder_aa  = 0.0
    elbow_fe     = 0.0

    return {
        "shoulder_flexion_extension":    shoulder_fe,
        "shoulder_abduction_adduction":  shoulder_aa,
        "elbow_flexion_extension":       elbow_fe,
    }


def landmarks_to_joints(landmarks, world):
    # ── Primary curl signal: true 3D distance-based (immune to camera angle) ──
    # Using 'world' instead of 'landmarks' provides metric 3D vectors immune to 2D foreshortening
    index_curl  = finger_curl(world, tip_idx=8,  mcp_idx=5)
    middle_curl = finger_curl(world, tip_idx=12, mcp_idx=9)
    ring_curl   = finger_curl(world, tip_idx=16, mcp_idx=13)
    pinky_curl  = finger_curl(world, tip_idx=20, mcp_idx=17)

    thumb_curl = finger_curl(world, tip_idx=4, mcp_idx=2)

    # Wrist flexion/extension & Forearm rotation using 3D world coordinates (invariant to 2D perspective scaling jumps when clenching)
    w_wrist = world[0]
    w_mid = world[9]
    w_idx = world[5]
    w_pnk = world[17]

    dx = w_mid["x"] - w_wrist["x"]; dy = w_mid["y"] - w_wrist["y"]; dz = w_mid["z"] - w_wrist["z"]
    wrist_ext = clamp(math.atan2(-dy, math.sqrt(dx**2 + dz**2)), -0.96, 0.96)

    kvec_x = w_pnk["x"] - w_idx["x"]; kvec_y = w_pnk["y"] - w_idx["y"]
    hand_roll = math.atan2(-kvec_y, kvec_x)
    forearm_rot = clamp((hand_roll + math.pi) / 2.0, 0.0, 3.14)

    # Palm normal from world landmarks
    pn = palm_normal(world)

    # Arm joints from absolute screen position to prevent it jerking when fist bounding box changes
    arm = arm_joints_from_camera(landmarks)

    joint_dict = {
        # Index (DIP starts at 1.57 to undo baked-in 90deg base hook, interpolates to 0.0 when closed)
        "mcp_index":  -1.57 * index_curl,
        "pip_index":  -1.57 * index_curl * 0.90,
        "dip_index":   1.57 * (1.0 - index_curl),
        # Middle
        "mcp_middle": -1.57 * middle_curl,
        "pip_middle": -1.57 * middle_curl * 0.90,
        "dip_middle":  1.57 * (1.0 - middle_curl),
        # Ring (mcp and dip both start hooked inwards randomly in URDF!)
        "mcp_ring":   1.57 * (1.0 - ring_curl),
        "pip_ring":   -1.57 * ring_curl * 0.90,
        "dip_ring":    1.57 * (1.0 - ring_curl),
        # Pinky
        "mcp_pinky":  1.57 * (1.0 - pinky_curl),
        "pip_pinky":  -1.57 * pinky_curl * 0.90,
        "dip_pinky":   1.57 * (1.0 - pinky_curl),
        # Thumb
        "cmc_thumb":  -0.35 + 2.44 * thumb_curl,
        "mcp_thumb":   0.785 + 1.745 * thumb_curl,
        "ip_thumb":    1.57 * thumb_curl,
        "wrist_extension":  wrist_ext,
        "forearm_rotation": forearm_rot,
        # Palm direction (informational)
        "palm_normal_x": pn["nx"],
        "palm_normal_y": pn["ny"],
        "palm_normal_z": pn["nz"],
    }

    # Add arm joints once calibration is done
    if arm is not None:
        joint_dict.update(arm)

    # Apply Exponential Moving Average (EMA) smoothing
    global _smoothed_joints
    for k, v in joint_dict.items():
        if k not in _smoothed_joints:
            _smoothed_joints[k] = v
        else:
            _smoothed_joints[k] = SMOOTH_ALPHA * v + (1.0 - SMOOTH_ALPHA) * _smoothed_joints[k]

    return _smoothed_joints


class WatoHandNode(Node):
    def __init__(self):
        super().__init__("wato_hand_node")
        self.get_logger().info("WatoHandNode started, waiting for landmarks...")
        self.sub = self.create_subscription(String, "/wato/hand_landmarks", self.landmarks_callback, 10)
        self.joint_pub = self.create_publisher(String, "/wato/hand_joint_angles", 10)
        self._calib_announced = False
        self._frame_count = 0

    def landmarks_callback(self, msg):
        try:
            payload = json.loads(msg.data)

            # Support both old format (list) and new bundled format (dict)
            if isinstance(payload, list):
                landmarks = payload
                world     = payload   # fallback: use image landmarks as world
            else:
                landmarks = payload["landmarks"]
                world     = payload.get("world", landmarks)

            if len(landmarks) != 21:
                return

            # Calibration status
            calib_done = _arm_ref is not None and _arm_ref["count"] >= CALIB_FRAMES
            if not calib_done:
                frames_left = 0 if _arm_ref is None else CALIB_FRAMES - _arm_ref["count"]
                self.get_logger().info(f"[CALIB] Hold neutral pose... {frames_left} frames remaining")

            joint_dict = landmarks_to_joints(landmarks, world)

            # Announce once when calibration completes
            if calib_done and not self._calib_announced:
                self.get_logger().info("✅ Calibration complete! Arm tracking active.")
                self._calib_announced = True

            # Periodic arm + palm status log (every 30 frames)
            self._frame_count += 1
            if calib_done and self._frame_count % 30 == 0:
                sfe  = joint_dict.get("shoulder_flexion_extension", 0.0)
                saa  = joint_dict.get("shoulder_abduction_adduction", 0.0)
                efe  = joint_dict.get("elbow_flexion_extension", 0.0)
                pnx  = joint_dict.get("palm_normal_x", 0.0)
                pny  = joint_dict.get("palm_normal_y", 0.0)
                pnz  = joint_dict.get("palm_normal_z", 0.0)
                self.get_logger().info(
                    f"arm: sfe={sfe:.2f} saa={saa:.2f} efe={efe:.2f} | "
                    f"palm_normal=({pnx:.2f},{pny:.2f},{pnz:.2f})"
                )

            out_msg = String()
            out_msg.data = json.dumps(joint_dict)
            self.joint_pub.publish(out_msg)

            with open(JOINT_FILE, "w") as f:
                json.dump({**joint_dict, "timestamp": time.time()}, f)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = WatoHandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()