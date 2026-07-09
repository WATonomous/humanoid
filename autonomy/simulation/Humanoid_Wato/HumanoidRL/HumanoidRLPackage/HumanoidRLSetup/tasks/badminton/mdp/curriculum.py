"""Badminton-specific curriculum helpers."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ramp_reward_weight(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    term_name: str,
    start_weight: float,
    end_weight: float,
    start_step: int,
    end_step: int,
) -> None:
    """Linearly ramp a reward term weight between ``start_step`` and ``end_step``."""
    step = env.common_step_counter
    if step <= start_step:
        weight = start_weight
    elif step >= end_step:
        weight = end_weight
    else:
        alpha = (step - start_step) / max(end_step - start_step, 1)
        weight = start_weight + alpha * (end_weight - start_weight)

    term_cfg = env.reward_manager.get_term_cfg(term_name)
    term_cfg.weight = weight
    env.reward_manager.set_term_cfg(term_name, term_cfg)
