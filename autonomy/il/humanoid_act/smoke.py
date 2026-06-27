"""Temporary smoke test for shape testing"""

from __future__ import annotations

import argparse
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Smoke-test ACT dataloader on a LeRobot dataset")
    parser.add_argument(
        "--dataset.repo_id",
        dest="repo_id",
        required=True,
        help="HuggingFace repo id or local repo_id (e.g. CursedRock17/so101_teleop_vials_sim_and_real)",
    )
    parser.add_argument(
        "--dataset.root",
        dest="root",
        default=None,
        help="Optional local dataset root (defaults to HF cache)",
    )
    parser.add_argument("--chunk_size", type=int, default=50, help="ACT action chunk length")
    parser.add_argument("--batch_size", type=int, default=4)
    parser.add_argument("--num_batches", type=int, default=2)
    parser.add_argument(
        "--stats_path",
        type=Path,
        default=None,
        help="Optional path to write/read stats.json",
    )
    args = parser.parse_args(argv)

    from humanoid_act.dataset import make_act_dataloaders
    from humanoid_act.normalize import save_stats

    print(f"[smoke] repo_id={args.repo_id}")
    print(f"[smoke] root={args.root or '(HF cache)'}")
    print(f"[smoke] chunk_size={args.chunk_size} batch_size={args.batch_size}")

    train_loader, val_loader, stats = make_act_dataloaders(
        args.repo_id,
        root=args.root,
        chunk_size=args.chunk_size,
        batch_size=args.batch_size,
        num_workers=0,
    )

    if args.stats_path:
        save_stats(stats, args.stats_path)

    print(
        "[smoke] norm stats:",
        f"state_dim={stats.state_mean.shape[0]}",
        f"action_dim={stats.action_mean.shape[0]}",
    )
    print(f"[smoke] train batches ~{len(train_loader)}, val batches ~{len(val_loader)}")

    for split_name, loader in ("train", train_loader), ("val", val_loader):
        print(f"\n[smoke] --- {split_name} ---")
        for i, batch in enumerate(loader):
            pad_frac = batch.is_pad.float().mean().item()
            print(
                f"  batch {i}: "
                f"images={tuple(batch.images.shape)} "
                f"qpos={tuple(batch.qpos.shape)} "
                f"actions={tuple(batch.actions.shape)} "
                f"is_pad={tuple(batch.is_pad.shape)} "
                f"pad_frac={pad_frac:.3f}"
            )
            if i + 1 >= args.num_batches:
                break

    print("\n[smoke] OK — phase 1 dataloader works.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
