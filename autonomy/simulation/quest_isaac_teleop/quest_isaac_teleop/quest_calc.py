import numpy as np

JOINT_NAMES = [
    "wrist",
    "thumb-metacarpal", "thumb-proximal", "thumb-distal", "thumb-tip",
    "index-metacarpal", "index-proximal", "index-intermediate", "index-distal", "index-tip",
    "middle-metacarpal", "middle-proximal", "middle-intermediate", "middle-distal", "middle-tip",
    "ring-metacarpal", "ring-proximal", "ring-intermediate", "ring-distal", "ring-tip",
    "pinky-metacarpal", "pinky-proximal", "pinky-intermediate", "pinky-distal", "pinky-tip",
]

MAX_CURL_RAD = 2.2

JOINT_MAP = {
    "mcp_index":  (0.0,     -1.5708),
    "pip_index":  (1.5708,   0.0),
    "dip_index":  (0.0,     -1.5708),
    "mcp_middle": (0.0,     -1.5708),
    "pip_middle": (1.5708,   0.0),
    "dip_middle": (1.5708,   0.0),
    "mcp_ring":   (1.5708,   0.0),
    "pip_ring":   (0.0,     -1.5708),
    "dip_ring":   (0.0,     -1.5708),
    "mcp_pinky":  (1.5708,   0.0),
    "pip_pinky":  (0.0,     -1.5708),
    "dip_pinky":  (1.5708,   0.0),
    "cmc_thumb":  (-0.3491,  2.0944),
    "mcp_thumb":  (2.5307,   0.7854),
    "ip_thumb":   (0.0,      1.5708),
}


def parse_hand_array(values):
    joints = {}
    for i, name in enumerate(JOINT_NAMES):
        joints[name] = np.array(values[i * 3 : i * 3 + 3])
    return joints


def angle_between(a, b):
    mag_a = np.sqrt(np.sum(a ** 2))
    mag_b = np.sqrt(np.sum(b ** 2))
    if mag_a < 1e-8 or mag_b < 1e-8:
        return 0.0
    return float(np.arccos(np.clip(np.dot(a, b) / (mag_a * mag_b), -1.0, 1.0)))


def finger_curl_angles(points, finger):
    mc   = points[f"{finger}-metacarpal"]
    prox = points[f"{finger}-proximal"]
    inter = points[f"{finger}-intermediate"]
    dist = points[f"{finger}-distal"]
    tip  = points[f"{finger}-tip"]

    bone_a = prox - mc
    bone_b = inter - prox
    bone_c = dist - inter
    bone_d = tip - dist

    return (angle_between(bone_a, bone_b),
            angle_between(bone_b, bone_c),
            angle_between(bone_c, bone_d))


def thumb_curl_angles(points):
    wrist = points["wrist"]
    mc   = points["thumb-metacarpal"]
    prox = points["thumb-proximal"]
    dist = points["thumb-distal"]
    tip  = points["thumb-tip"]

    bone_a = mc - wrist
    bone_b = prox - mc
    bone_c = dist - prox
    bone_d = tip - dist

    return (angle_between(bone_a, bone_b),
            angle_between(bone_b, bone_c),
            angle_between(bone_c, bone_d))


def curl_to_joint_angle(angle_rad, joint_name):
    fraction = min(angle_rad / MAX_CURL_RAD, 1.0)
    extended, curled = JOINT_MAP[joint_name]
    return extended + fraction * (curled - extended)


def compute_all_targets(hand_array, side):
    points = parse_hand_array(hand_array)
    targets = {}
    prefix = f"{side}_" if side else ""

    for finger in ["index", "middle", "ring", "pinky"]:
        mcp, pip, dip = finger_curl_angles(points, finger)
        targets[f"{prefix}mcp_{finger}"] = curl_to_joint_angle(mcp, f"mcp_{finger}")
        targets[f"{prefix}pip_{finger}"] = curl_to_joint_angle(pip, f"pip_{finger}")
        targets[f"{prefix}dip_{finger}"] = curl_to_joint_angle(dip, f"dip_{finger}")

    cmc, mcp, ip = thumb_curl_angles(points)
    targets[f"{prefix}cmc_thumb"] = curl_to_joint_angle(cmc, "cmc_thumb")
    targets[f"{prefix}mcp_thumb"] = curl_to_joint_angle(mcp, "mcp_thumb")
    targets[f"{prefix}ip_thumb"]  = curl_to_joint_angle(ip,  "ip_thumb")

    return targets
