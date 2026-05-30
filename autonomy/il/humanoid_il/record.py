"""Record teleoperation data to a LeRobot dataset from ROS 2 topics."""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from humanoid_il.episode_keys import EpisodeFlags, EpisodeKeyboard
from humanoid_il.record_utils import RateLimiter, resolve_config_path
from humanoid_il.schema import create_dataset, enabled_images, load_yaml

logger = logging.getLogger(__name__)

_PKG_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_SCHEMA = _PKG_ROOT / "config" / "dataset_schema.yaml"


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record a LeRobot dataset from ROS 2 (WATO humanoid arm)."
    )
    parser.add_argument(
        "--schema",
        type=str,
        default=str(_DEFAULT_SCHEMA),
        help="Path to dataset_schema.yaml",
    )
    parser.add_argument(
        "--dataset_root",
        type=str,
        default=None,
        help="Override record.root base directory (e.g. datasets/record)",
    )
    parser.add_argument("--num_episodes", type=int, default=10)
    parser.add_argument("--task_description", type=str, default="humanoid demonstration")
    parser.add_argument(
        "--episode_time_s",
        type=float,
        default=None,
        help="Auto-finish each episode after N seconds (keyboard optional)",
    )
    parser.add_argument(
        "--reset_time_s",
        type=float,
        default=5.0,
        help="Pause between saved episodes",
    )
    parser.add_argument(
        "--auto_start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Log frames immediately when an episode begins",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Synthetic data only; no ROS (tests LeRobot write path)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args(argv)


def _build_frame(
    snapshot,
    *,
    task: str,
    image_keys: list[str],
) -> dict[str, Any]:
    if snapshot.action is None or snapshot.state is None:
        raise ValueError("Missing state or action")

    frame: dict[str, Any] = {
        "observation.state": snapshot.state,
        "action": snapshot.action,
        "task": task,
    }
    for key in image_keys:
        lerobot_key = f"observation.images.{key}"
        if key not in snapshot.images:
            raise ValueError(f"Missing image '{key}' for frame")
        frame[lerobot_key] = snapshot.images[key]
    return frame


def _dry_snapshot(t: float, image_keys: list[str], dim: int):
    from humanoid_il.ros_buffer import ObservationSnapshot

    vec = np.array(
        [0.2 * math.sin(t + i) for i in range(dim)], dtype=np.float32
    )
    images = {key: np.zeros((480, 640, 3), dtype=np.uint8) for key in image_keys}
    return ObservationSnapshot(state=vec, action=vec.copy(), images=images)


def _record_loop(
    args: argparse.Namespace,
    cfg: dict[str, Any],
    get_snapshot: Callable,
) -> Path:
    image_keys = list(enabled_images(cfg).keys())
    dim = len(cfg["joint_names"])
    fps = float(cfg.get("fps", 30))
    rate = RateLimiter(fps)

    if args.dataset_root:
        record_root = Path(args.dataset_root)
    else:
        record_root = Path((cfg.get("record") or {}).get("root", "datasets/record"))

    dataset, root = create_dataset(cfg, record_root=record_root)
    logger.info("Writing dataset to %s", root)

    flags = EpisodeFlags(start=bool(args.auto_start))
    keyboard = EpisodeKeyboard(flags)
    keyboard.start()

    episode_index = 0
    episode_start = time.monotonic()
    t0 = time.monotonic()

    try:
        while episode_index < args.num_episodes and not flags.abort:
            flags.success = False
            flags.remove = False
            if args.auto_start:
                flags.start = True
            episode_start = time.monotonic()

            while not flags.success and not flags.abort:
                rate.sleep()
                get_snapshot.maybe_spin()

                if not flags.start:
                    continue

                if args.episode_time_s is not None:
                    if time.monotonic() - episode_start >= args.episode_time_s:
                        flags.success = True

                try:
                    snapshot = get_snapshot()
                    if args.dry_run:
                        snapshot = _dry_snapshot(
                            time.monotonic() - t0, image_keys, dim
                        )
                    frame = _build_frame(
                        snapshot,
                        task=args.task_description,
                        image_keys=image_keys,
                    )
                except ValueError:
                    continue

                if flags.remove:
                    dataset.clear_episode_buffer()
                    flags.remove = False
                    episode_start = time.monotonic()
                    continue

                dataset.add_frame(frame)

            if flags.abort:
                break

            dataset.save_episode()
            logger.info(
                "Saved episode %s / %s", episode_index + 1, args.num_episodes
            )
            episode_index += 1

            if episode_index < args.num_episodes and args.reset_time_s > 0:
                logger.info("Reset window %.1fs", args.reset_time_s)
                time.sleep(args.reset_time_s)

        if flags.abort:
            dataset.clear_episode_buffer()
        dataset.finalize()
    finally:
        keyboard.stop()
        get_snapshot.cleanup()

    return root


class _SnapshotSource:
    def maybe_spin(self) -> None:
        pass

    def cleanup(self) -> None:
        pass


class _DryRunSource(_SnapshotSource):
    def __init__(self, image_keys: list[str], dim: int) -> None:
        self._image_keys = image_keys
        self._dim = dim
        self._t0 = time.monotonic()

    def __call__(self):
        return _dry_snapshot(time.monotonic() - self._t0, self._image_keys, self._dim)


class _RosSource(_SnapshotSource):
    def __init__(self, cfg: dict[str, Any]) -> None:
        import rclpy
        from rclpy.node import Node

        from humanoid_il.ros_buffer import RosRecordBuffer

        self._rclpy = rclpy
        rclpy.init()
        self._node = Node("humanoid_record")
        self._buffer = RosRecordBuffer(self._node, cfg)

    def maybe_spin(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def __call__(self):
        return self._buffer.snapshot()

    def cleanup(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    schema_path = resolve_config_path(args.schema, anchor=_PKG_ROOT)
    cfg = load_yaml(schema_path)
    image_keys = list(enabled_images(cfg).keys())
    dim = len(cfg["joint_names"])

    if args.dry_run:
        source: _SnapshotSource = _DryRunSource(image_keys, dim)
    else:
        source = _RosSource(cfg)

    root = _record_loop(args, cfg, source)
    print(f"Dataset saved under: {root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
