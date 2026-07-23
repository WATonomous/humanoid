"""State/action normalization stats for ACT training."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


STATE_KEY = "observation.state"
ACTION_KEY = "action"
MIN_STD = 1e-2


@dataclass
class NormStats:
    state_mean: np.ndarray
    state_std: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray

    def normalize_state(self, x: np.ndarray) -> np.ndarray:
        return (x - self.state_mean) / self.state_std

    def normalize_action(self, x: np.ndarray) -> np.ndarray:
        return (x - self.action_mean) / self.action_std

    def denormalize_state(self, x: np.ndarray) -> np.ndarray:
        return x * self.state_std + self.state_mean

    def denormalize_action(self, x: np.ndarray) -> np.ndarray:
        return x * self.action_std + self.action_mean

    def to_dict(self) -> dict[str, list[float]]:
        return {
            "state_mean": self.state_mean.tolist(),
            "state_std": self.state_std.tolist(),
            "action_mean": self.action_mean.tolist(),
            "action_std": self.action_std.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> NormStats:
        return cls(
            state_mean=np.asarray(data["state_mean"], dtype=np.float32),
            state_std=np.asarray(data["state_std"], dtype=np.float32),
            action_mean=np.asarray(data["action_mean"], dtype=np.float32),
            action_std=np.asarray(data["action_std"], dtype=np.float32),
        )


def save_stats(stats: NormStats, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(stats.to_dict(), indent=2))


def load_stats(path: Path) -> NormStats:
    return NormStats.from_dict(json.loads(path.read_text()))


def _clip_std(std: np.ndarray) -> np.ndarray:
    return np.clip(std, MIN_STD, np.inf).astype(np.float32)


def stats_from_lerobot_meta(meta: Any) -> NormStats | None:
    """Use dataset.meta.stats when LeRobot already computed them."""
    raw = getattr(meta, "stats", None)
    if not raw:
        return None
    if STATE_KEY not in raw or ACTION_KEY not in raw:
        return None
    state = raw[STATE_KEY]
    action = raw[ACTION_KEY]
    if "mean" not in state or "std" not in state:
        return None
    return NormStats(
        state_mean=np.asarray(state["mean"], dtype=np.float32),
        state_std=_clip_std(np.asarray(state["std"], dtype=np.float32)),
        action_mean=np.asarray(action["mean"], dtype=np.float32),
        action_std=_clip_std(np.asarray(action["std"], dtype=np.float32)),
    )


def load_or_compute_stats(
    dataset: Any,
    *,
    cache_path: Path | None = None,
    recompute: bool = False,
) -> NormStats:
    if cache_path and cache_path.exists() and not recompute:
        return load_stats(cache_path)

    stats = stats_from_lerobot_meta(dataset.meta)
    if stats is None:
        raise ValueError(
            "Dataset is missing meta.stats for observation.state and action. "
            "Use a LeRobot dataset that includes stats (HF hub datasets do), "
            "or pass a saved stats.json via cache_path."
        )

    if cache_path:
        save_stats(stats, cache_path)
        print(f"[stats] wrote {cache_path}")

    return stats
