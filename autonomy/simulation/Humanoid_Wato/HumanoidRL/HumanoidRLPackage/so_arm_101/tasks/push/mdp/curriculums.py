# Copyright (c) 2024-2025, Muammer Bay (LycheeAI), Louis Le Lay
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Curriculum terms for the push-block task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import ManagerTermBase

if TYPE_CHECKING:
    from collections.abc import Sequence

    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import CurriculumTermCfg

__all__ = ["spawn_offset_curriculum"]


class spawn_offset_curriculum(ManagerTermBase):
    """Performance-gated spawn-offset curriculum for the block reset.

    Widens the block's spawn *offset* range one stage at a time by mutating, in
    place, the ``pose_range`` of the ``reset_object_position`` event term (the
    :class:`EventManager` reads ``term_cfg.params`` fresh on every reset, so the
    new range takes effect on the next spawn). Only the offset *magnitude* grows
    -- the spawn stays a fixed default anchor + random xy offset + full yaw, the
    same mechanism as the moderate setup.

    Advancement is gated on task performance AND a minimum dwell: a stage
    advances only once (a) the policy has trained at that stage for at least
    ``min_stage_steps`` environment steps, and (b) the rolling success rate over
    the last ``window`` completed episodes then reaches ``threshold``. The dwell
    is essential -- at thousands of envs the ``window`` refills many times within
    a single PPO iteration, so without it a competent resumed policy would
    cascade through several stages in one or two iterations (skipping the actual
    graduated widening) before ever training on the wider spawns. After each
    advance the window is cleared so the next gate measures competence *at the
    new, wider spawn range*.

    Per-episode success reuses the existing success reward term
    (``success_reward_term``, e.g. ``block_on_floor``) with zero duplicated
    geometry: this term runs inside ``_reset_idx`` *before* the reward manager
    zeroes its episode sums, so a positive accumulated sum means the block
    reached the interior floor at some point during the episode.

    It also toggles an optional repositioning reward term
    (``reposition_reward_term``) on -- from weight 0 to ``reposition_weight`` --
    once the curriculum reaches ``reposition_start_stage``, tying that shaping
    term's activation directly to the curriculum stage.
    """

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        p = cfg.params
        self._stages: list[dict] = list(p["stages"])
        self._event_term: str = p["event_term_name"]
        self._success_term: str = p["success_reward_term"]
        self._window: int = int(p["window"])
        self._threshold: float = float(p["threshold"])
        self._min_stage_steps: int = int(p.get("min_stage_steps", 0))
        self._min_ep_len: int = int(p.get("min_episode_len", 2))
        self._reposition_term: str | None = p.get("reposition_reward_term")
        self._reposition_weight: float = float(p.get("reposition_weight", 0.0))
        self._reposition_start_stage: int = int(p.get("reposition_start_stage", len(self._stages)))

        self._stage = 0
        self._stage_start_step = int(env.common_step_counter)  # for the dwell gate
        # ring buffer of recent completed-episode outcomes (1.0 success / 0.0 fail)
        self._buf = torch.zeros(self._window, device=env.device)
        self._count = 0  # valid entries written so far (saturates at window)
        self._ptr = 0

        # apply stage 0 range and set the repositioning term to its stage-0 state
        self._apply_stage()

    def _apply_stage(self):
        env = self._env
        # widen the spawn offset range in place
        ev_cfg = env.event_manager.get_term_cfg(self._event_term)
        ev_cfg.params["pose_range"] = dict(self._stages[self._stage])
        env.event_manager.set_term_cfg(self._event_term, ev_cfg)
        # gate the repositioning reward on the curriculum stage
        if self._reposition_term is not None:
            rw_cfg = env.reward_manager.get_term_cfg(self._reposition_term)
            active = self._stage >= self._reposition_start_stage
            rw_cfg.weight = self._reposition_weight if active else 0.0
            env.reward_manager.set_term_cfg(self._reposition_term, rw_cfg)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        env_ids: Sequence[int],
        stages: list[dict],
        event_term_name: str,
        success_reward_term: str,
        window: int,
        threshold: float,
        min_stage_steps: int = 0,
        min_episode_len: int = 2,
        reposition_reward_term: str | None = None,
        reposition_weight: float = 0.0,
        reposition_start_stage: int | None = None,
    ) -> dict[str, float]:
        # record outcomes only for genuinely completed episodes (skip freshly
        # initialized envs whose episode has not run yet)
        ep_len = env.episode_length_buf[env_ids]
        valid = ep_len >= self._min_ep_len
        if bool(valid.any()):
            ep_sum = env.reward_manager._episode_sums[self._success_term][env_ids]
            outcomes = (ep_sum > 0.0).float()[valid]
            n = int(outcomes.numel())
            idx = (self._ptr + torch.arange(n, device=env.device)) % self._window
            self._buf[idx] = outcomes
            self._ptr = int((self._ptr + n) % self._window)
            self._count = min(self._count + n, self._window)

        if self._count >= self._window:
            rate = float(self._buf.mean())
        elif self._count > 0:
            rate = float(self._buf[: self._count].mean())
        else:
            rate = 0.0

        # advance only after a minimum dwell at this stage AND once the window is
        # full and performance clears the threshold (dwell prevents cascading
        # through stages within a single iteration; see class docstring).
        steps_in_stage = int(env.common_step_counter) - self._stage_start_step
        if (
            self._stage < len(self._stages) - 1
            and steps_in_stage >= self._min_stage_steps
            and self._count >= self._window
            and rate >= self._threshold
        ):
            self._stage += 1
            self._stage_start_step = int(env.common_step_counter)
            self._apply_stage()
            self._buf.zero_()
            self._count = 0
            self._ptr = 0

        return {
            "stage": float(self._stage),
            "success_rate": rate,
            "x_off_hi": float(self._stages[self._stage]["x"][1]),
            "reposition_active": float(self._stage >= self._reposition_start_stage),
        }
