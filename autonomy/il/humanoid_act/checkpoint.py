"""Save/load humanoid ACT checkpoints."""

from __future__ import annotations

from pathlib import Path

import torch

from humanoid_act.config import ACTConfig
from humanoid_act.normalize import NormStats, save_stats
from humanoid_act.policy import ACTPolicy


def save_checkpoint(
    path: Path,
    *,
    policy: ACTPolicy,
    config: ACTConfig,
    stats: NormStats,
    step: int,
    val_loss: float | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "step": step,
        "val_loss": val_loss,
        "policy_state_dict": policy.state_dict(),
        "optimizer_state_dict": policy.configure_optimizers().state_dict(),
    }
    torch.save(payload, path)
    config.save(path.parent / "config.json")
    save_stats(stats, path.parent / "stats.json")


def load_policy(checkpoint_path: Path, device: torch.device) -> tuple[ACTPolicy, ACTConfig, NormStats]:
    from humanoid_act.normalize import load_stats

    ckpt_dir = checkpoint_path.parent
    config = ACTConfig.load(ckpt_dir / "config.json")
    stats = load_stats(ckpt_dir / "stats.json")
    policy = ACTPolicy(config)
    payload = torch.load(checkpoint_path, map_location=device)
    policy.load_state_dict(payload["policy_state_dict"])
    policy.to(device)
    policy.eval()
    return policy, config, stats
