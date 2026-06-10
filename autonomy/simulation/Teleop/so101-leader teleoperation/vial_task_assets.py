"""Paths to SO101 vial-to-rack task USD assets under assets/lerobot/."""

from __future__ import annotations

import math
import os

_TELEOP_DIR = os.path.abspath(os.path.dirname(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_TELEOP_DIR, "..", "..", "..", ".."))
VIAL_TASK_USD_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task", "usd")

LIGHTBOX_USD = os.path.join(VIAL_TASK_USD_DIR, "lightbox-simple.usd")
MAT_USDA = os.path.join(VIAL_TASK_USD_DIR, "mat.usda")
VIAL_OPAQUE_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_opaque.usda")
VIAL_RACK_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_rack_simple.usda")
TRAY_USDA = os.path.join(VIAL_TASK_USD_DIR, "tray.usda")

SO101_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101")
SO101_FOLLOWER_USD = os.path.join(SO101_DIR, "so101_follower_good.usd")
SO101_ARM_CAMERA_USD = os.path.join(SO101_DIR, "so101_arm_camera.usd")

VIAL_SPAWN_Z = 0.05
VIAL_SPAWNS = (
    (0.23, -0.08, VIAL_SPAWN_Z),
    (0.23, 0.0, VIAL_SPAWN_Z),
    (0.23, -0.16, VIAL_SPAWN_Z),
)
RACK_SPAWN = (0.18, 0.08, 0.06)
TRAY_SPAWN = (0.12, 0.10, 0.035)
MAT_SPAWN_POS = (0.22, 0.0, 0.032)
LIGHTBOX_SPAWN_POS = (-0.1, 0.0, 0.0257)

HDRI_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task", "hdri")
HDRI_MOON_LAB = os.path.join(HDRI_DIR, "moon_lab_1k.exr")
# Must match sync_so101_vial_assets.sh HDRI_FILES (.exr count)
EXPECTED_HDRI_EXR_COUNT = 23


def quat_wxyz_from_euler_deg(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """XYZ Euler angles in degrees → quaternion (w, x, y, z)."""
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, yq, z)


# Workshop vials_to_rack_env_cfg orientations
VIAL_QUAT = quat_wxyz_from_euler_deg(0.0, 90.0, 0.0)
MAT_QUAT = quat_wxyz_from_euler_deg(0.0, 0.0, 90.0)


def assert_assets_present(*, require_tray: bool = True) -> None:
    required = [LIGHTBOX_USD, MAT_USDA, VIAL_OPAQUE_USDA, VIAL_RACK_USDA]
    if require_tray:
        required.append(TRAY_USDA)
    missing = [p for p in required if not os.path.isfile(p)]
    if missing:
        raise FileNotFoundError(
            "Missing vial-task assets. Run:\n"
            "  ./assets/lerobot/sync_so101_vial_assets.sh\n"
            "Missing:\n  " + "\n  ".join(missing)
        )


def assert_arm_camera_usd() -> None:
    if not os.path.isfile(SO101_ARM_CAMERA_USD):
        raise FileNotFoundError(
            "Missing workshop arm USD (visible gripper camera mesh). Run:\n"
            "  ./assets/lerobot/sync_so101_vial_assets.sh --arm\n"
            "  # or: ./assets/lerobot/sync_so101_vial_assets.sh --full\n"
            f"Expected: {SO101_ARM_CAMERA_USD}"
        )


def assert_hdri_for_domain_rand() -> None:
    """Full ``--domain_rand`` requires the complete HDRI pack for sky randomization."""
    if not os.path.isdir(HDRI_DIR):
        raise FileNotFoundError(
            "HDRI pack required for --domain_rand. Run:\n"
            "  ./assets/lerobot/sync_so101_vial_assets.sh --hdri\n"
            "  # or: ./assets/lerobot/sync_so101_vial_assets.sh --full\n"
            f"Expected directory: {HDRI_DIR}"
        )
    exr_files = sorted(f for f in os.listdir(HDRI_DIR) if f.endswith(".exr"))
    if not os.path.isfile(HDRI_MOON_LAB) or len(exr_files) < EXPECTED_HDRI_EXR_COUNT:
        raise FileNotFoundError(
            f"Incomplete HDRI pack ({len(exr_files)}/{EXPECTED_HDRI_EXR_COUNT} .exr). Run:\n"
            "  ./assets/lerobot/sync_so101_vial_assets.sh --hdri\n"
            "  # or: ./assets/lerobot/sync_so101_vial_assets.sh --full"
        )
    yaw_map = os.path.join(HDRI_DIR, "yaw_mapping.yaml")
    if not os.path.isfile(yaw_map):
        raise FileNotFoundError(
            f"Missing {yaw_map}. Re-run ./assets/lerobot/sync_so101_vial_assets.sh --hdri"
        )


def warn_if_hdri_missing() -> None:
    """Deprecated alias — use assert_hdri_for_domain_rand for --domain_rand."""
    try:
        assert_hdri_for_domain_rand()
    except FileNotFoundError as exc:
        print(f"[WARN] {exc}")
