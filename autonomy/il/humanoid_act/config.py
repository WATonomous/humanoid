"""Training / model configuration for humanoid ACT."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass
class ACTConfig:
    repo_id: str
    state_dim: int
    action_dim: int
    camera_names: list[str]
    chunk_size: int = 50
    kl_weight: float = 10.0
    lr: float = 1e-5
    lr_backbone: float = 1e-5
    weight_decay: float = 1e-4
    backbone: str = "resnet18"
    position_embedding: str = "sine"
    hidden_dim: int = 512
    dim_feedforward: int = 3200
    enc_layers: int = 4
    dec_layers: int = 7
    nheads: int = 8
    dropout: float = 0.1
    seed: int = 42

    @classmethod
    def from_dataset_meta(
        cls,
        repo_id: str,
        meta: Any,
        *,
        chunk_size: int = 50,
        **overrides: Any,
    ) -> ACTConfig:
        state_dim = int(meta.features["observation.state"]["shape"][0])
        action_dim = int(meta.features["action"]["shape"][0])
        camera_names = list(meta.camera_keys)
        cfg = cls(
            repo_id=repo_id,
            state_dim=state_dim,
            action_dim=action_dim,
            camera_names=camera_names,
            chunk_size=chunk_size,
        )
        for key, value in overrides.items():
            if hasattr(cfg, key):
                setattr(cfg, key, value)
        return cfg

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2))

    @classmethod
    def load(cls, path: Path) -> ACTConfig:
        return cls(**json.loads(path.read_text()))
