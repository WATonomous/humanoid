"""Build LeRobot feature dicts from dataset_schema.yaml."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from humanoid_il.record_utils import get_next_experiment_path_with_gap


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


def enabled_images(cfg: dict[str, Any]) -> dict[str, dict[str, Any]]:
    images = cfg.get("images") or {}
    out: dict[str, dict[str, Any]] = {}
    for key, spec in images.items():
        if not isinstance(spec, dict):
            continue
        if spec.get("enabled", True):
            out[key] = spec
    return out


def build_features(cfg: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """LeRobotDataset.create features block (pattern: lehome dataset_record.py)."""
    joint_names = list(cfg["joint_names"])
    dim = len(joint_names)
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (dim,),
            "names": joint_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (dim,),
            "names": joint_names,
        },
    }

    for key, spec in enabled_images(cfg).items():
        h = int(spec["height"])
        w = int(spec["width"])
        features[f"observation.images.{key}"] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

    return features


def create_dataset(cfg: dict[str, Any], *, record_root: Path | None = None):
    """Create a new LeRobotDataset on disk."""
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise ImportError(
            "lerobot is required for recording. Install with: "
            "pip install -e 'autonomy/il[lerobot]'"
        ) from exc

    record_cfg = cfg.get("record") or {}
    root_base = Path(record_root or record_cfg.get("root", "datasets/record"))
    root = get_next_experiment_path_with_gap(root_base)

    dataset = LeRobotDataset.create(
        repo_id=str(cfg.get("repo_id", "humanoid/local")),
        fps=int(cfg.get("fps", 30)),
        root=root,
        use_videos=bool(record_cfg.get("use_videos", True)),
        image_writer_threads=int(record_cfg.get("image_writer_threads", 4)),
        image_writer_processes=int(record_cfg.get("image_writer_processes", 0)),
        features=build_features(cfg),
    )
    return dataset, root
