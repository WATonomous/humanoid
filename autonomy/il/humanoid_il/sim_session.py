"""Helpers to start a RecordSession from Isaac Sim teleop scripts."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from humanoid_il.recorder import RecordSession, RecordSettings
from humanoid_il.sinks import create_sinks, parse_sink_names


def create_sim_record_session(
    cfg: dict[str, Any],
    *,
    record_root: Path,
    sink: str,
    num_episodes: int,
    task_description: str,
    auto_start: bool = False,
) -> RecordSession:
    """
    Build a RecordSession for step-by-step use inside an Isaac sim loop.

    Use keyboard S/N/D/Esc (pynput) for episode control — same as humanoid-record.
    """
    sink_names = parse_sink_names(sink)
    output_dir, sinks = create_sinks(cfg, record_root=record_root, sink_names=sink_names)
    settings = RecordSettings(
        num_episodes=num_episodes,
        task_description=task_description,
        auto_start=auto_start,
        use_keyboard=True,
    )
    session = RecordSession(cfg, sinks, settings, output_dir=output_dir)
    session.start_keyboard()
    session.begin_episode()
    return session
