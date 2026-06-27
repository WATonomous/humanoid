"""LeRobot dataset → ACT training batches (action chunks + padding mask)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset

from humanoid_act.normalize import NormStats, STATE_KEY, ACTION_KEY


@dataclass
class ACTBatch:
    images: torch.Tensor  # (B, num_cams, C, H, W), float32 in [0, 1]
    qpos: torch.Tensor  # (B, state_dim)
    actions: torch.Tensor  # (B, chunk_size, action_dim)
    is_pad: torch.Tensor  # (B, chunk_size), True where padded


def _action_chunk_offsets(fps: float, chunk_size: int, camera_keys: list[str]) -> dict[str, list[float]]:
    dt = 1.0 / fps
    action_offsets = [i * dt for i in range(chunk_size)]
    delta: dict[str, list[float]] = {
        STATE_KEY: [0.0],
        ACTION_KEY: action_offsets,
    }
    for cam in camera_keys:
        delta[cam] = [0.0]
    return delta


class ACTChunkDataset(Dataset):
    """
    Random-frame ACT dataset backed by LeRobot.

    Each sample:
      - images at timestep t (all cameras)
      - proprio at t
      - action chunk [a_t, a_{t+1}, ...] padded to chunk_size
      - is_pad mask for positions after episode end
    """

    def __init__(
        self,
        repo_id: str,
        *,
        root: str | None = None,
        episodes: list[int] | None = None,
        chunk_size: int = 100,
        camera_keys: list[str] | None = None,
        stats: NormStats | None = None,
    ) -> None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        # Metadata-only load to discover cameras/fps before building deltas.
        meta_ds = LeRobotDataset(repo_id, root=root, episodes=episodes)
        self.camera_keys = camera_keys or list(meta_ds.meta.camera_keys)
        if not self.camera_keys:
            raise ValueError("No camera keys found in dataset metadata")

        self.chunk_size = chunk_size
        self.stats = stats
        self.fps = float(meta_ds.meta.fps)

        offsets = _action_chunk_offsets(self.fps, chunk_size, self.camera_keys)
        self._ds = LeRobotDataset(
            repo_id,
            root=root,
            episodes=episodes,
            delta_timestamps=offsets,
        )
        selected = episodes if episodes is not None else list(range(self._ds.meta.total_episodes))
        self._episode_lengths = {
            ep: int(self._ds.meta.episodes[ep]["length"]) for ep in selected
        }

    def __len__(self) -> int:
        return len(self._ds)

    def __getitem__(self, idx: int) -> dict[str, torch.Tensor]:
        sample = self._ds[idx]
        ep = int(_tensor_scalar(sample["episode_index"]))
        frame = int(_tensor_scalar(sample["frame_index"]))
        ep_len = self._episode_lengths[ep]
        steps_left = max(ep_len - frame, 0)
        is_pad = torch.zeros(self.chunk_size, dtype=torch.bool)
        if steps_left < self.chunk_size:
            is_pad[steps_left:] = True

        images = []
        for cam in self.camera_keys:
            img = _to_numpy(sample[cam]).astype(np.float32)
            # LeRobot: (T, C, H, W) with T=1 when delta is [0]
            if img.ndim == 4:
                img = img[0]
            images.append(img)
        image_stack = np.stack(images, axis=0) / 255.0  # (num_cams, C, H, W)

        qpos = _to_numpy(sample[STATE_KEY]).astype(np.float32).reshape(-1)
        if qpos.ndim == 2:
            qpos = qpos[0]

        actions = _to_numpy(sample[ACTION_KEY]).astype(np.float32)

        if self.stats is not None:
            qpos = self.stats.normalize_state(qpos)
            actions = self.stats.normalize_action(actions)

        return {
            "images": torch.from_numpy(image_stack),
            "qpos": torch.from_numpy(qpos),
            "actions": torch.from_numpy(actions),
            "is_pad": is_pad,
        }


def collate_act_batch(items: list[dict[str, torch.Tensor]]) -> ACTBatch:
    return ACTBatch(
        images=torch.stack([x["images"] for x in items], dim=0),
        qpos=torch.stack([x["qpos"] for x in items], dim=0),
        actions=torch.stack([x["actions"] for x in items], dim=0),
        is_pad=torch.stack([x["is_pad"] for x in items], dim=0),
    )


def split_episodes(num_episodes: int, train_ratio: float = 0.8, seed: int = 42) -> tuple[list[int], list[int]]:
    rng = np.random.default_rng(seed)
    indices = rng.permutation(num_episodes).tolist()
    split = int(train_ratio * num_episodes)
    return indices[:split], indices[split:]


def make_act_dataloaders(
    repo_id: str,
    *,
    root: str | None = None,
    chunk_size: int = 100,
    batch_size: int = 8,
    train_ratio: float = 0.8,
    seed: int = 42,
    stats: NormStats | None = None,
    num_workers: int = 0,
) -> tuple[DataLoader, DataLoader, NormStats]:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    probe = LeRobotDataset(repo_id, root=root)
    train_eps, val_eps = split_episodes(probe.num_episodes, train_ratio=train_ratio, seed=seed)

    if stats is None:
        from humanoid_act.normalize import load_or_compute_stats

        stats = load_or_compute_stats(
            LeRobotDataset(repo_id, root=root, episodes=train_eps),
        )

    train_ds = ACTChunkDataset(
        repo_id,
        root=root,
        episodes=train_eps,
        chunk_size=chunk_size,
        stats=stats,
    )
    val_ds = ACTChunkDataset(
        repo_id,
        root=root,
        episodes=val_eps,
        chunk_size=chunk_size,
        stats=stats,
    )

    train_loader = DataLoader(
        train_ds,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        collate_fn=collate_act_batch,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        collate_fn=collate_act_batch,
        pin_memory=torch.cuda.is_available(),
    )
    return train_loader, val_loader, stats


def _tensor_scalar(value: Any) -> int | float:
    if isinstance(value, torch.Tensor):
        return value.item()
    return value


def _to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)
