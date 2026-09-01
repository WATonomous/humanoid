"""Simulated perception as a mjlab command term.

Owns the batched shuttle EKF (perception_torch) and publishes two feature
blocks each control tick, both laid out as [p(3), v(3), traj(n_traj*3)]:

  student_features  from the EKF fed one noisy position measurement per tick
                    — twitchy early, converging as measurements accumulate,
                    matching the real perception model refining its fit
  teacher_features  the same layout computed from the true state (the
                    privileged trajectory prior)

The command lifecycle fits exactly: _resample_command fires on episode reset
(after the shuttle reset event wrote the new true state) and re-initializes
the filter rows; _update_command runs once per control tick after
sim.forward(), so measurements always come from the current true state and
observations read fresh features.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import torch

from mjlab.managers.command_manager import CommandTerm, CommandTermCfg

import aero
import perception_torch as pt

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


class PerceptionCommand(CommandTerm):
    cfg: "PerceptionCommandCfg"

    def __init__(self, cfg: "PerceptionCommandCfg", env: "ManagerBasedRlEnv"):
        super().__init__(cfg, env)
        params = aero.load_params()
        pp = params["perception"]
        self._k = params["shuttle"]["k"]
        self._g = params["gravity"]
        self._sigma_meas = pp["sigma_meas"]
        self._sigma_p0 = pp["sigma_p0"]
        self._sigma_v0 = pp["sigma_v0"]
        self._sigma_acc = pp["sigma_acc"]
        self._n_traj = pp["n_traj"]
        self._traj_dt = pp["traj_dt"]
        self._sub_dt = pp["traj_sub_dt"]

        self._shuttle = env.scene[cfg.entity_name]
        idx = self._shuttle.data.indexing
        self._q_adr = idx.free_joint_q_adr
        self._v_adr = idx.free_joint_v_adr

        n, dev = self.num_envs, self.device
        self._x = torch.zeros(n, 6, device=dev)
        self._P = torch.zeros(n, 6, 6, device=dev)
        self._just_reset = torch.zeros(n, dtype=torch.bool, device=dev)
        dim = 6 + 3 * self._n_traj
        self.student_features = torch.zeros(n, dim, device=dev)
        self.teacher_features = torch.zeros(n, dim, device=dev)
        # ground-truth task geometry (logged via metrics): per-episode min
        # face->p* distance and the distance at the tick nearest t*
        self._face_cfg = None
        self._min_dist = torch.full((n,), float("inf"), device=dev)
        self._dist_at_tstar = torch.zeros(n, device=dev)
        self._tstar_seen = torch.zeros(n, dtype=torch.bool, device=dev)

    @property
    def command(self) -> torch.Tensor:
        return self.student_features

    def _true_state(self) -> tuple[torch.Tensor, torch.Tensor]:
        data = self._shuttle.data.data
        return (data.qpos[:, self._q_adr[:3]].clone(),
                data.qvel[:, self._v_adr[:3]].clone())

    def _measure(self, p_true: torch.Tensor) -> torch.Tensor:
        return p_true + self._sigma_meas * torch.randn_like(p_true)

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        p_true, _ = self._true_state()
        z0 = self._measure(p_true[env_ids])
        x, P = pt.ekf_init(z0, self._sigma_p0, self._sigma_v0)
        self._x[env_ids] = x
        self._P[env_ids] = P
        self._just_reset[env_ids] = True
        self._min_dist[env_ids] = float("inf")
        self._dist_at_tstar[env_ids] = 0.0
        self._tstar_seen[env_ids] = False

    def _update_command(self, env_ids: torch.Tensor | None) -> None:
        p_true, v_true = self._true_state()
        if env_ids is None:
            # per-tick path: EKF predict+update for rows that were not reset
            # this very tick (reset rows already hold their init measurement)
            run = ~self._just_reset
            if bool(run.any()):
                ids = run.nonzero(as_tuple=False).squeeze(-1)
                x, P = pt.ekf_predict(self._x[ids], self._P[ids],
                                      self._env.step_dt, self._k, self._g,
                                      self._sigma_acc)
                x, P = pt.ekf_update(x, P, self._measure(p_true[ids]),
                                     self._sigma_meas)
                self._x[ids] = x
                self._P[ids] = P
            self._just_reset[:] = False
        self.student_features[:] = pt.feature_layout(
            self._x[:, :3], self._x[:, 3:], self._k, self._n_traj,
            self._traj_dt, self._sub_dt, self._g)
        self.teacher_features[:] = pt.feature_layout(
            p_true, v_true, self._k, self._n_traj,
            self._traj_dt, self._sub_dt, self._g)

    def _update_metrics(self) -> None:
        p_true, v_true = self._true_state()
        self.metrics["ekf_pos_err"] = (self._x[:, :3] - p_true).norm(dim=-1)
        self.metrics["ekf_vel_err"] = (self._x[:, 3:] - v_true).norm(dim=-1)

        # lazy: mdp imports this module, so import it only at call time
        from badminton_mjlab import mdp
        store = getattr(self._env, "_badminton", None)
        if store is None:
            return
        if self._face_cfg is None:
            from mjlab.managers.scene_entity_config import SceneEntityCfg
            cfg = SceneEntityCfg("robot", site_names=("face_center",))
            cfg.resolve(self._env.scene)
            self._face_cfg = cfg
        face, _ = mdp._face_pose(self._env, self._face_cfg)
        dist = (face - store["p_star"]).norm(dim=-1)
        self._min_dist = torch.minimum(self._min_dist, dist)
        t_now = self._env.episode_length_buf.float() * self._env.step_dt
        at_tstar = (t_now >= store["t_star"]) & ~self._tstar_seen
        self._dist_at_tstar[at_tstar] = dist[at_tstar]
        self._tstar_seen |= at_tstar
        # logged at episode reset (last assigned value), so these read as
        # per-episode min / at-t* distances
        self.metrics["face_pstar_min_dist"] = torch.where(
            torch.isinf(self._min_dist), dist, self._min_dist)
        self.metrics["face_pstar_dist_at_tstar"] = self._dist_at_tstar


@dataclass(kw_only=True)
class PerceptionCommandCfg(CommandTermCfg):
    entity_name: str = "shuttle"
    resampling_time_range: tuple[float, float] = field(
        default=(1.0e9, 1.0e9))  # never; reset() re-inits per episode

    def build(self, env: "ManagerBasedRlEnv") -> PerceptionCommand:
        return PerceptionCommand(self, env)
