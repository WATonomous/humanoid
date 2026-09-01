"""Capture SO101 sim camera RGB frames for ``humanoid_il`` recording."""

from __future__ import annotations

from typing import Any

import numpy as np

from humanoid_il.schema import enabled_images

# Schema image keys → InteractiveScene sensor names (see so101_cfg.py).
SO101_CAMERA_SCENE_KEYS: dict[str, str] = {
    "ego": "camera_ego",
    "external_D455": "camera_external_D455",
}


def schema_needs_cameras(cfg: dict[str, Any]) -> bool:
    return bool(enabled_images(cfg))


def schema_image_keys_to_scene(cfg: dict[str, Any]) -> dict[str, str]:
    """Map dataset_schema image keys to scene sensor entity names."""
    mapping: dict[str, str] = {}
    for key in enabled_images(cfg):
        mapping[key] = SO101_CAMERA_SCENE_KEYS.get(key, f"camera_{key}")
    return mapping


def capture_rgb_images(scene, key_to_scene: dict[str, str]) -> dict[str, np.ndarray]:
    """Read RGB tensors from TiledCamera sensors registered on the scene."""
    images: dict[str, np.ndarray] = {}
    for schema_key, scene_key in key_to_scene.items():
        if scene_key not in scene:
            raise ValueError(
                f"Camera sensor '{scene_key}' not in scene (schema key '{schema_key}'). "
                "Launch with --cameras and SO101VialTaskVisionSceneCfg."
            )
        camera = scene[scene_key]
        rgb = camera.data.output["rgb"]
        frame = rgb[0].detach().cpu().numpy()
        if frame.shape[-1] > 3:
            frame = frame[..., :3]
        images[schema_key] = np.asarray(frame, dtype=np.uint8)
    return images
