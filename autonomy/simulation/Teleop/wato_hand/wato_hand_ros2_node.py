import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time

JOINT_FILE = "/tmp/wato_joints.json"

# ── Finger curl calibration ───────────────────────────────────────────────────
# OPEN_RATIO  — tip/mcp distance ratio when finger is fully extended (~2.2)
# CLOSE_RATIO — tip/mcp distance ratio when finger is in a tight fist  (~0.8)
OPEN_RATIO  = 2.2
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
_finger_ref = None   # per-joint open-hand baseline — set dynamically
CALIB_FRAMES = 15



def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def vec(a, b):
    """Vector from landmark a to landmark b."""
    return (b["x"]-a["x"], b["y"]-a["y"], b["z"]-a["z"])


def dot(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]


def mag(u):
    return math.sqrt(u[0]**2 + u[1]**2 + u[2]**2) or 1e-9


def joint_angle_curl_raw(landmarks, a_idx, b_idx, c_idx):
    """Raw bend angle at joint b (chain a→b→c), returned as [0,1] fraction of π."""
    ab = vec(landmarks[a_idx], landmarks[b_idx])
    bc = vec(landmarks[b_idx], landmarks[c_idx])
    cos_angle = clamp(dot(ab, bc) / (mag(ab) * mag(bc)), -1.0, 1.0)
    return math.acos(cos_angle) / math.pi


def apply_finger_baseline(raw, key):
    """Subtract the calibrated open-hand baseline for this joint, clamp to [0,1]."""
    if _finger_ref is None or key not in _finger_ref:
        return clamp(raw, 0.0, 1.0)
    baseline = _finger_ref[key]
    if isinstance(baseline, dict):   # still accumulating during calibration
        return clamp(raw, 0.0, 1.0)
    return clamp((raw - baseline) / max(1.0 - baseline, 0.01), 0.0, 1.0)


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
    curl  = 1.0 - clamp((ratio - CLOSE_RATIO) / (OPEN_RATIO - CLOSE_RATIO), 0.0, 1.0)
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


def arm_joints_from_world(world):
    """
    Map 3D wrist position (world landmarks, wrist-centred relative to calibrated neutral)
    to shoulder and elbow joint angles.

    world[0] is always the wrist — its position in world coords is always (0,0,0)
    because world landmarks ARE wrist-centred.

    Instead, we use the middle-MCP (world[9]) as our hand-frame reference point,
    and track how the hand tilts relative to the calibrated neutral.

    For absolute position tracking we use the image-space position of the wrist
    combined with world Z depth to estimate camera-frame 3D displacement.
    """
    global _arm_ref

    # Use wrist→middle-MCP vector in world frame as orientation proxy
    mid_mcp = world[9]
    # In world frame, wrist is at origin — mid_mcp gives hand pointing direction
    hx, hy, hz = mid_mcp["x"], mid_mcp["y"], mid_mcp["z"]

    if _arm_ref is None:
        _arm_ref = {"hx": hx, "hy": hy, "hz": hz, "count": 1}
        return None   # still calibrating
    elif _arm_ref["count"] < CALIB_FRAMES:
        # Running average during calibration
        n = _arm_ref["count"]
        _arm_ref["hx"] = (_arm_ref["hx"] * n + hx) / (n + 1)
        _arm_ref["hy"] = (_arm_ref["hy"] * n + hy) / (n + 1)
        _arm_ref["hz"] = (_arm_ref["hz"] * n + hz) / (n + 1)
        _arm_ref["count"] += 1
        return None   # still calibrating

    # Delta from neutral
    dx = hx - _arm_ref["hx"]   # left/right
    dy = hy - _arm_ref["hy"]   # up/down  (world y: up=negative in MediaPipe)
    dz = hz - _arm_ref["hz"]   # depth

    shoulder_fe  = clamp(SHOULDER_FE_SCALE  * (-dy), *SHOULDER_FE_LIMITS)
    shoulder_aa  = clamp(SHOULDER_AA_SCALE  * (-dx), *SHOULDER_AA_LIMITS)
    elbow_fe     = clamp(ELBOW_FE_SCALE     *   dz,  *ELBOW_FE_LIMITS)

    return {
        "shoulder_flexion_extension":    shoulder_fe,
        "shoulder_abduction_adduction":  shoulder_aa,
        "elbow_flexion_extension":       elbow_fe,
    }


def landmarks_to_joints(landmarks, world):
    # ── FINGER JOINTS: collect raw angles, build calibration, then apply baseline ──
    # Joint chains: (a→b→c) — raw values measured every frame
    CHAINS = {
        "mcp_index":  (0, 5,  6),  "pip_index":  (5,  6,  7),  "dip_index":  (6,  7,  8),
        "mcp_middle": (0, 9,  10), "pip_middle": (9,  10, 11), "dip_middle": (10, 11, 12),
        "mcp_ring":   (0, 13, 14), "pip_ring":   (13, 14, 15), "dip_ring":   (14, 15, 16),
        "mcp_pinky":  (0, 17, 18), "pip_pinky":  (17, 18, 19), "dip_pinky":  (18, 19, 20),
    }
    global _finger_ref
    raw_curls = {k: joint_angle_curl_raw(landmarks, *v) for k, v in CHAINS.items()}

    # Accumulate calibration during startup hold (runs in parallel with arm calib)
    if _finger_ref is None:
        _finger_ref = {k: {"sum": v, "count": 1} for k, v in raw_curls.items()}
    elif isinstance(next(iter(_finger_ref.values())), dict):  # still accumulating
        n = next(iter(_finger_ref.values()))["count"]
        if n < CALIB_FRAMES:
            for k, v in raw_curls.items():
                _finger_ref[k]["sum"]   += v
                _finger_ref[k]["count"] += 1
        else:
            # Finalise: replace dicts with scalar averages
            _finger_ref = {k: d["sum"] / d["count"] for k, d in _finger_ref.items()}

    # Apply per-joint calibrated baseline
    fc = {k: apply_finger_baseline(raw_curls[k], k) for k in CHAINS}

    idx_mcp_c, idx_pip_c, idx_dip_c = fc["mcp_index"],  fc["pip_index"],  fc["dip_index"]
    mid_mcp_c, mid_pip_c, mid_dip_c = fc["mcp_middle"], fc["pip_middle"], fc["dip_middle"]
    rng_mcp_c, rng_pip_c, rng_dip_c = fc["mcp_ring"],   fc["pip_ring"],   fc["dip_ring"]
    pnk_mcp_c, pnk_pip_c, pnk_dip_c = fc["mcp_pinky"],  fc["pip_pinky"],  fc["dip_pinky"]

    # Thumb uses legacy distance-based curl (geometry is different)
    thumb_curl = finger_curl(landmarks, tip_idx=4, mcp_idx=2)

    # Wrist flexion/extension
    wrist   = landmarks[0]; mid_mcp = landmarks[9]
    dx = mid_mcp["x"] - wrist["x"]; dy = mid_mcp["y"] - wrist["y"]; dz = mid_mcp["z"] - wrist["z"]
    wrist_ext = clamp(math.atan2(-dy, math.sqrt(dx**2 + dz**2)), -0.96, 0.96)

    # Forearm rotation (full [-π,+π] linear map to [0, π])
    idx_mcp = landmarks[5]; pnk_mcp = landmarks[17]
    kvec_x  = pnk_mcp["x"] - idx_mcp["x"]; kvec_y = pnk_mcp["y"] - idx_mcp["y"]
    hand_roll   = math.atan2(-kvec_y, kvec_x)
    forearm_rot = clamp((hand_roll + math.pi) / 2.0, 0.0, 3.14)

    # Palm normal from world landmarks
    pn = palm_normal(world)

    # Arm joints from world position
    arm = arm_joints_from_world(world)

    joint_dict = {
        # Index — independent per-joint angles
        "mcp_index":  -1.57 * idx_mcp_c,
        "pip_index":   1.57 * idx_pip_c,
        "dip_index":  -1.57 * idx_dip_c,
        # Middle
        "mcp_middle": -1.57 * mid_mcp_c,
        "pip_middle":  1.57 * mid_pip_c,
        "dip_middle":  1.57 * mid_dip_c,
        # Ring (axis inverted in URDF)
        "mcp_ring":    1.57 * rng_mcp_c,
        "pip_ring":   -1.57 * rng_pip_c,
        "dip_ring":   -1.57 * rng_dip_c,
        # Pinky (axis inverted in URDF)
        "mcp_pinky":   1.57 * pnk_mcp_c,
        "pip_pinky":  -1.57 * pnk_pip_c,
        "dip_pinky":   1.57 * pnk_dip_c,
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

    return joint_dict


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