# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""
Test the dex_retargeting library (WatoHandDexRetargeting) for the custom Wato arm/hand.
No Isaac Lab / XR required — only dex_retargeting, torch, wato_dex_retargeting_utils, and arm_assembly URDF.

Usage:
  cd /path/to/autonomy/simulation/Teleop/wato_hand
  python run_dex_retargeting_test.py

Requires: torch, dex_retargeting, scipy, pyyaml (e.g. conda env with isaac lab deps).
Uses synthetic OpenXR-style hand poses to run left/right retargeting and print 15-DOF output.
"""

from __future__ import annotations

import cv2
import numpy as np

# Wato arm_assembly hand joint names (15 DOF per hand)
WATO_HAND_JOINT_NAMES = [
    "mcp_index", "pip_index", "dip_index",
    "mcp_middle", "pip_middle", "dip_middle",
    "mcp_ring", "pip_ring", "dip_ring",
    "mcp_pinky", "pip_pinky", "dip_pinky",
    "cmc_thumb", "mcp_thumb", "ip_thumb",
]


def make_fake_openxr_hand_poses(seed: int = 0) -> dict:
    """Build fake OpenXR-style hand pose dict for one hand (26 joints + wrist).

    convert_hand_joints expects list(hand_poses.values()) to be ordered so index i = joint i.
    So build with keys 0,1,...,25 then "wrist". Each joint value is [x,y,z]; wrist is [x,y,z, w,qx,qy,qz].
    """
    rng = np.random.default_rng(seed)
    out = {}
    for i in range(26):
        out[i] = np.array([0.0, 0.0, 0.05 + (i * 0.005)], dtype=np.float64)
    out[0] = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    out[1] = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    out["wrist"] = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return out


def build_right_openxr_pose_from_mediapipe(
    mediapipe_joint_pos: np.ndarray,  # (21, 3), wrist-centered (wrist at 0)
    wrist_rot: np.ndarray,  # (3, 3) rotation matrix
) -> dict:
    """Convert MediaPipe 21-joint world pose to OpenXR-style 26-joint + wrist.

    Your dex-retargeting Wato wrapper selects a subset of OpenXR joints defined by:
      _OPENXR_HAND_JOINT_INDICES = [1,2,3,4,5,7,...,25] (21 joints)
    so we fill those OpenXR indices from MediaPipe finger joints, including thumb/index/middle/ring/pinky.
    """
    from scipy.spatial.transform import Rotation as R

    assert mediapipe_joint_pos.shape == (21, 3), mediapipe_joint_pos.shape
    assert wrist_rot.shape == (3, 3), wrist_rot.shape

    # OpenXR-style pose dict expects keys 0..25 + "wrist".
    hand_poses: dict[int | str, np.ndarray] = {}

    # MediaPipe joint index reference:
    # 0 wrist, 1 thumb_cmc, 2 thumb_mcp, 3 thumb_ip, 4 thumb_tip
    # 5 index_mcp, 6 index_pip, 7 index_dip, 8 index_tip
    # 9 middle_mcp, 10 middle_pip, 11 middle_dip, 12 middle_tip
    # 13 ring_mcp, 14 ring_pip, 15 ring_dip, 16 ring_tip
    # 17 pinky_mcp, 18 pinky_pip, 19 pinky_dip, 20 pinky_tip
    mp = mediapipe_joint_pos

    # Wrist (OpenXR joint index 1 is used by your wrapper subset)
    wrist_xyz = mp[0]
    hand_poses[1] = wrist_xyz.copy()

    # Thumb (OpenXR 2-5 correspond to thumb proximal->tip; mp 1-4)
    hand_poses[2] = mp[1].copy()
    hand_poses[3] = mp[2].copy()
    hand_poses[4] = mp[3].copy()
    hand_poses[5] = mp[4].copy()

    # Index (OpenXR 7-10 correspond; mp 5-8)
    hand_poses[7] = mp[5].copy()
    hand_poses[8] = mp[6].copy()
    hand_poses[9] = mp[7].copy()
    hand_poses[10] = mp[8].copy()

    # Middle (OpenXR 12-15 correspond; mp 9-12)
    hand_poses[12] = mp[9].copy()
    hand_poses[13] = mp[10].copy()
    hand_poses[14] = mp[11].copy()
    hand_poses[15] = mp[12].copy()

    # Ring (OpenXR 17-20 correspond; mp 13-16)
    hand_poses[17] = mp[13].copy()
    hand_poses[18] = mp[14].copy()
    hand_poses[19] = mp[15].copy()
    hand_poses[20] = mp[16].copy()

    # Pinky (OpenXR 22-25 correspond; mp 17-20)
    hand_poses[22] = mp[17].copy()
    hand_poses[23] = mp[18].copy()
    hand_poses[24] = mp[19].copy()
    hand_poses[25] = mp[20].copy()

    # Fill missing OpenXR indices with zeros (wrapper only uses a subset, but convert_hand_joints
    # iterates values() in key order when building 21 joints).
    for i in range(26):
        if i not in hand_poses:
            hand_poses[i] = np.zeros(3, dtype=np.float64)

    # Wrist quaternion: wrapper expects hand_poses["wrist"] = [x,y,z, w,qx,qy,qz]
    quat_xyzw = R.from_matrix(wrist_rot).as_quat()  # (x,y,z,w)
    qx, qy, qz, qw = quat_xyzw
    hand_poses["wrist"] = np.array([wrist_xyz[0], wrist_xyz[1], wrist_xyz[2], qw, qx, qy, qz], dtype=np.float64)

    return hand_poses


def main():
    from wato_dex_retargeting_utils import WatoHandDexRetargeting
    from single_hand_detector import SingleHandDetector

    print("Testing dex_retargeting library (WatoHandDexRetargeting) for custom arm/hand ...")
    dex = WatoHandDexRetargeting(hand_joint_names=WATO_HAND_JOINT_NAMES)

    detector = SingleHandDetector(hand_type="Right", selfie=False)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Failed to open webcam (cv2.VideoCapture(0)).")

    print("Running live webcam retargeting. Press 'q' in the preview window to quit.")
    while True:
        ok, bgr = cap.read()
        if not ok:
            continue

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        _, joint_pos, keypoint_2d, wrist_rot = detector.detect(rgb)

        if joint_pos is None:
            cv2.imshow("wato_dex_retargeting_live_right", bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        # Visualize landmarks if you want quick feedback.
        if keypoint_2d is not None:
            bgr = detector.draw_skeleton_on_image(bgr, keypoint_2d, style="default")

        # Build OpenXR-style pose dict and retarget.
        right_poses = build_right_openxr_pose_from_mediapipe(
            mediapipe_joint_pos=joint_pos,
            wrist_rot=wrist_rot,
        )
        right_q = dex.compute_right(right_poses)

        print("Right retargeted (15 DOF hand):", right_q)
        cv2.imshow("wato_dex_retargeting_live_right", bgr)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    print("Joint names:", dex.get_right_joint_names())
    print("Dex retargeting live test done.")


if __name__ == "__main__":
    main()
