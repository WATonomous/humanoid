"""Record teleoperation data from ROS 2 (real arm) or dry-run tests."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any

from humanoid_il.record_loop import run_record_loop
from humanoid_il.recorder import RecordSettings
from humanoid_il.record_utils import resolve_config_path
from humanoid_il.schema import enabled_images, load_yaml
from humanoid_il.sinks import parse_sink_names
from humanoid_il.snapshot import ObservationSnapshot

logger = logging.getLogger(__name__)

_PKG_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_SCHEMA = _PKG_ROOT / "config" / "dataset_schema.yaml"


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record demonstrations to LeRobot and/or HDF5 (WATO humanoid arm, ROS 2)."
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
    parser.add_argument(
        "--sink",
        type=str,
        default="lerobot",
        help="Output sinks: lerobot, hdf5, or comma-separated (e.g. lerobot,hdf5)",
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
        help="Synthetic data only; no ROS (tests write path)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args(argv)


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

    def __call__(self) -> ObservationSnapshot:
        from humanoid_il.record_loop import dry_snapshot

        return dry_snapshot(time.monotonic() - self._t0, self._image_keys, self._dim)


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

    def __call__(self) -> ObservationSnapshot:
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

    if args.dataset_root:
        record_root = Path(args.dataset_root)
    else:
        record_root = Path((cfg.get("record") or {}).get("root", "datasets/record"))

    settings = RecordSettings(
        num_episodes=args.num_episodes,
        task_description=args.task_description,
        episode_time_s=args.episode_time_s,
        reset_time_s=args.reset_time_s,
        auto_start=args.auto_start,
    )
    sink_names = parse_sink_names(args.sink)

    if args.dry_run:
        source: _SnapshotSource = _DryRunSource(image_keys, dim)
    else:
        source = _RosSource(cfg)

    output_dir = run_record_loop(
        cfg,
        source,
        settings,
        record_root=record_root,
        sink_names=sink_names,
        dry_run=args.dry_run,
    )
    print(f"Dataset saved under: {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
