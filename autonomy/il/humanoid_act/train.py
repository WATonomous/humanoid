"""Train custom ACT on LeRobot datasets."""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass
from itertools import cycle
from pathlib import Path

import numpy as np
import torch

from humanoid_act.checkpoint import save_checkpoint
from humanoid_act.config import ACTConfig
from humanoid_act.dataset import ACTBatch, make_act_dataloaders
from humanoid_act.policy import ACTPolicy


@dataclass
class TrainConfig:
    chunk_size: int = 50
    batch_size: int = 8
    steps: int = 10_000
    val_freq: int = 500
    save_freq: int = 5000
    lr: float = 1e-5
    kl_weight: float = 10.0
    seed: int = 42
    device: str = "cuda" if torch.cuda.is_available() else "cpu"


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def forward_batch(policy: ACTPolicy, batch: ACTBatch, device: torch.device) -> dict[str, torch.Tensor]:
    images = batch.images.to(device)
    qpos = batch.qpos.to(device)
    actions = batch.actions.to(device)
    is_pad = batch.is_pad.to(device)
    return policy(qpos, images, actions, is_pad)


@torch.no_grad()
def validate(policy: ACTPolicy, val_loader, device: torch.device) -> dict[str, float]:
    policy.eval()
    totals: dict[str, float] = {}
    count = 0
    for batch in val_loader:
        loss_dict = forward_batch(policy, batch, device)
        count += 1
        for key, value in loss_dict.items():
            totals[key] = totals.get(key, 0.0) + float(value.item())
    return {key: value / max(count, 1) for key, value in totals.items()}


def main(argv: list[str] | None = None) -> int:
    train_cfg = TrainConfig()

    parser = argparse.ArgumentParser(description="Train humanoid ACT on a LeRobot dataset")
    parser.add_argument("repo_id", help="HuggingFace repo id or local LeRobot repo_id")
    parser.add_argument("output_dir", type=Path, help="Directory for checkpoints and config.json")
    parser.add_argument(
        "--root",
        default=None,
        help="Optional local dataset root (defaults to HF cache)",
    )
    args = parser.parse_args(argv)

    set_seed(train_cfg.seed)
    device = torch.device(train_cfg.device)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    train_loader, val_loader, stats = make_act_dataloaders(
        args.repo_id,
        root=args.root,
        chunk_size=train_cfg.chunk_size,
        batch_size=train_cfg.batch_size,
        seed=train_cfg.seed,
    )

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    meta = LeRobotDataset(args.repo_id, root=args.root).meta
    config = ACTConfig.from_dataset_meta(
        args.repo_id,
        meta,
        chunk_size=train_cfg.chunk_size,
        kl_weight=train_cfg.kl_weight,
        lr=train_cfg.lr,
        seed=train_cfg.seed,
    )
    config.save(args.output_dir / "config.json")

    policy = ACTPolicy(config).to(device)
    optimizer = policy.configure_optimizers()
    train_iter = cycle(train_loader)

    best_val = float("inf")
    print(
        f"[train] repo_id={args.repo_id} device={device} "
        f"state_dim={config.state_dim} chunk_size={config.chunk_size} "
        f"cameras={config.camera_names}"
    )

    for step in range(1, train_cfg.steps + 1):
        policy.train()
        batch = next(train_iter)
        loss_dict = forward_batch(policy, batch, device)
        loss = loss_dict["loss"]

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if step % 100 == 0 or step == 1:
            print(
                f"[train] step {step}/{train_cfg.steps} "
                f"loss={loss.item():.4f} l1={loss_dict['l1'].item():.4f} "
                f"kl={loss_dict['kl'].item():.4f}"
            )

        if step % train_cfg.val_freq == 0 or step == train_cfg.steps:
            val_metrics = validate(policy, val_loader, device)
            val_loss = val_metrics["loss"]
            print(
                f"[val] step {step} loss={val_loss:.4f} "
                f"l1={val_metrics['l1']:.4f} kl={val_metrics['kl']:.4f}"
            )
            if val_loss < best_val:
                best_val = val_loss
                save_checkpoint(
                    args.output_dir / "best.pt",
                    policy=policy,
                    config=config,
                    stats=stats,
                    step=step,
                    val_loss=val_loss,
                )
                print(f"[train] saved best checkpoint (val_loss={val_loss:.4f})")

        if step % train_cfg.save_freq == 0 or step == train_cfg.steps:
            save_checkpoint(
                args.output_dir / "last.pt",
                policy=policy,
                config=config,
                stats=stats,
                step=step,
            )
            print(f"[train] saved last checkpoint at step {step}")

    print(f"[train] done. best_val_loss={best_val:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
