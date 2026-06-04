from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import combine_frame_transforms, quat_from_euler_xyz

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ring_marker_utils import NUM_INTERCEPT_MARKERS

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import UniformInterceptCommandCfg


class UniformInterceptCommand(CommandTerm):
    """Command generator for a timed 3D intercept point (badminton shuttle arrival).

    Privileged timing (for the policy): intercept position, time-to-hit, ring scale,
    and a short hit-moment pulse when the shuttle would arrive.

    Debug visualization only: rings shrink during countdown, flash at min size for one
    step at contact, then hide until the next resample.
    """

    cfg: UniformInterceptCommandCfg

    def __init__(self, cfg: UniformInterceptCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]

        self.pos_command_b = torch.zeros(self.num_envs, 3, device=self.device)
        self.lead_time_left = torch.zeros(self.num_envs, device=self.device)
        self.lead_time_total = torch.ones(self.num_envs, device=self.device)
        self.hit_moment_time_left = torch.zeros(self.num_envs, device=self.device)
        self.hit_moment_active = torch.zeros(self.num_envs, device=self.device)

        self.pos_command_w = torch.zeros_like(self.pos_command_b)
        self._target_tilt_quat = quat_from_euler_xyz(
            torch.tensor([0.0], device=self.device),
            torch.tensor([self.cfg.target_tilt_pitch_rad], device=self.device),
            torch.tensor([self.cfg.target_tilt_yaw_rad], device=self.device),
        ).repeat(self.num_envs, 1)

        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["hit_in_moment"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        msg = "UniformInterceptCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tHit moment duration: {self.cfg.hit_moment_duration_s} s (0 = one env step)\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    @property
    def command(self) -> torch.Tensor:
        """Privileged intercept command. Shape is (num_envs, 5).

        - [:3] target position in robot base frame
        - [3] hit-moment pulse (1.0 on the shuttle-arrival step, else 0.0)
        - [4] seconds until shuttle arrival (0.0 once the moment has passed)

        Ring scale is debug-visualization only and is not part of the policy obs
        (keeps obs dim compatible with earlier checkpoints).
        """
        return torch.cat(
            [
                self.pos_command_b,
                self.hit_moment_active.unsqueeze(-1),
                self.lead_time_left.unsqueeze(-1),
            ],
            dim=-1,
        )

    def _ring_scale(self) -> torch.Tensor:
        """Visual-only ring scale: shrink → contact flash → hide."""
        lead_ratio = self.lead_time_left / self.lead_time_total.clamp(min=1.0e-3)
        countdown_scale = self.cfg.min_ring_scale + (1.0 - self.cfg.min_ring_scale) * lead_ratio

        in_countdown = self.lead_time_left > 0.0
        at_contact = self.hit_moment_active > 0.5
        after_contact = (~in_countdown) & (~at_contact)

        scale = torch.where(in_countdown, countdown_scale, torch.full_like(countdown_scale, self.cfg.min_ring_scale))
        scale = torch.where(at_contact, torch.full_like(scale, self.cfg.min_ring_scale), scale)
        if self.cfg.post_hit_ring_hidden:
            scale = torch.where(after_contact, torch.zeros_like(scale), scale)
        else:
            scale = torch.where(after_contact, torch.ones_like(scale), scale)
        return scale

    def _hit_moment_duration(self) -> float:
        if self.cfg.hit_moment_duration_s > 0.0:
            return self.cfg.hit_moment_duration_s
        return self._env.step_dt

    def _update_metrics(self):
        self.pos_command_w, _ = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pos_command_b,
        )

    def _resample_command(self, env_ids: Sequence[int]):
        r = torch.empty(len(env_ids), device=self.device)
        self.pos_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        self.pos_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        self.pos_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)

        sampled_lead = r.uniform_(*self.cfg.ranges.lead_time)
        self.lead_time_left[env_ids] = sampled_lead
        self.lead_time_total[env_ids] = sampled_lead
        self.hit_moment_time_left[env_ids] = 0.0
        self.hit_moment_active[env_ids] = 0.0

    def _update_command(self):
        dt = self._env.step_dt
        hit_duration = self._hit_moment_duration()

        shuttle_arrives = (self.lead_time_left > 0.0) & (self.lead_time_left - dt <= 0.0)
        self.lead_time_left = torch.clamp(self.lead_time_left - dt, min=0.0)

        self.hit_moment_time_left = torch.where(
            shuttle_arrives,
            torch.full_like(self.hit_moment_time_left, hit_duration),
            self.hit_moment_time_left,
        )
        self.hit_moment_time_left = torch.where(
            self.hit_moment_time_left > 0.0,
            torch.clamp(self.hit_moment_time_left - dt, min=0.0),
            self.hit_moment_time_left,
        )
        self.hit_moment_active = (self.hit_moment_time_left > 0.0).float()

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "target_visualizer"):
                self.target_visualizer = VisualizationMarkers(self.cfg.target_visualizer_cfg)
            self.target_visualizer.set_visibility(True)
        elif hasattr(self, "target_visualizer"):
            self.target_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return

        num_markers = self.num_envs * NUM_INTERCEPT_MARKERS
        translations = self.pos_command_w.repeat_interleave(NUM_INTERCEPT_MARKERS, dim=0)
        orientations = self._target_tilt_quat.repeat_interleave(NUM_INTERCEPT_MARKERS, dim=0)

        env_ring_scale = self._ring_scale()
        ring_scale = env_ring_scale.repeat_interleave(NUM_INTERCEPT_MARKERS)
        scales = ring_scale.unsqueeze(-1).expand(-1, 3).clone()
        center_mask = (
            torch.arange(num_markers, device=self.device) % NUM_INTERCEPT_MARKERS
        ) == (NUM_INTERCEPT_MARKERS - 1)
        center_scale = torch.where(env_ring_scale > 0.0, 0.45, 0.0).repeat_interleave(NUM_INTERCEPT_MARKERS)
        scales[center_mask] = center_scale[center_mask].unsqueeze(-1).expand(-1, 3)

        marker_indices = torch.arange(NUM_INTERCEPT_MARKERS, device=self.device).repeat(self.num_envs)
        self.target_visualizer.visualize(
            translations=translations,
            orientations=orientations,
            scales=scales,
            marker_indices=marker_indices,
        )
