"""Paths to SO101 vial-task assets under humanoid/assets/lerobot/."""

from __future__ import annotations

import os

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", "..", ".."))

SO101_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101")
SO101_FOLLOWER_USD = os.path.join(SO101_DIR, "so101_follower_good.usd")
SO101_ARM_CAMERA_USD = os.path.join(SO101_DIR, "so101_arm_camera.usd")

VIAL_TASK_USD_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task", "usd")
HDRI_DIR = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task", "hdri")

LIGHTBOX_USD = os.path.join(VIAL_TASK_USD_DIR, "lightbox-simple.usd")
MAT_USDA = os.path.join(VIAL_TASK_USD_DIR, "mat.usda")
TRAY_USDA = os.path.join(VIAL_TASK_USD_DIR, "tray.usda")
VIAL_OPAQUE_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_opaque.usda")
VIAL_RACK_USDA = os.path.join(VIAL_TASK_USD_DIR, "Vial_rack_simple.usda")
HDRI_MOON_LAB = os.path.join(HDRI_DIR, "moon_lab_1k.exr")

# Legacy alias used by ported task cfgs (`assets_path/usd/...`)
assets_path = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101_vial_task")
