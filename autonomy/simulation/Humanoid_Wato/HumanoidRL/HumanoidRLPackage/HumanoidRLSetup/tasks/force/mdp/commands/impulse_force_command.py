from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import combine_frame_transforms, quat_apply

from HumanoidRLPackage.HumanoidRLSetup.tasks.force.mdp.arrow_utils import resolve_force_to_arrow

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import UniformImpulseForceCommandCfg


class UniformImpulseForceCommand(CommandTerm):
    """Command generator for short force impulses at sampled locations in the robot base frame.

    The command tensor has shape (num_envs, 7):
        - [:3] target position in base frame
        - [3:6] desired force vector in base frame
        - [6] impulse active flag (1.0 during the impulse window, else 0.0)
    """

    cfg: UniformImpulseForceCommandCfg

    def __init__(self, cfg: UniformImpulseForceCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]

        self.pos_command_b = torch.zeros(self.num_envs, 3, device=self.device)
        self.force_command_b = torch.zeros(self.num_envs, 3, device=self.device)
        self.impulse_time_left = torch.zeros(self.num_envs, device=self.device)
        self.impulse_active = torch.zeros(self.num_envs, device=self.device)

        self.pos_command_w = torch.zeros_like(self.pos_command_b)
        self.force_command_w = torch.zeros_like(self.force_command_b)
        self.measured_force_w = torch.zeros_like(self.force_command_b)

        self.metrics["region_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["force_error"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        msg = "UniformImpulseForceCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tImpulse duration: {self.cfg.impulse_duration_s} s\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    @property
    def command(self) -> torch.Tensor:
        """Desired impulse command. Shape is (num_envs, 7)."""
        return torch.cat([self.pos_command_b, self.force_command_b, self.impulse_active.unsqueeze(-1)], dim=-1)

    def _update_metrics(self):
        self.pos_command_w, _ = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pos_command_b,
        )
        self.force_command_w = quat_apply(self.robot.data.root_quat_w, self.force_command_b)
        self.measured_force_w = self._get_best_measured_force()

        body_pos_w = self.robot.data.body_state_w[:, :, :3]
        dists = torch.norm(body_pos_w - self.pos_command_w.unsqueeze(1), dim=-1)
        self.metrics["region_error"] = dists.min(dim=1).values

        active = self.impulse_active > 0.5
        force_error = torch.norm(self.measured_force_w - self.force_command_w, dim=-1)
        self.metrics["force_error"] = torch.where(active, force_error, torch.zeros_like(force_error))

    def _get_best_measured_force(self) -> torch.Tensor:
        """Return the largest net contact force vector among all monitored links."""
        contact_sensor: ContactSensor = self._env.scene.sensors[self.cfg.contact_sensor_name]
        forces = contact_sensor.data.net_forces_w
        magnitudes = torch.norm(forces, dim=-1)
        best_idx = magnitudes.argmax(dim=1)
        env_ids = torch.arange(self.num_envs, device=self.device)
        return forces[env_ids, best_idx]

    def _resample_command(self, env_ids: Sequence[int]):
        r = torch.empty(len(env_ids), device=self.device)
        self.pos_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        self.pos_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        self.pos_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)

        self.force_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.force_x)
        self.force_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.force_y)
        self.force_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.force_z)

        self.impulse_time_left[env_ids] = self.cfg.impulse_duration_s
        self.impulse_active[env_ids] = 1.0

    def _update_command(self):
        dt = self._env.step_dt
        self.impulse_time_left = torch.clamp(self.impulse_time_left - dt, min=0.0)
        self.impulse_active = (self.impulse_time_left > 0.0).float()

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_force_visualizer"):
                self.goal_force_visualizer = VisualizationMarkers(self.cfg.goal_force_visualizer_cfg)
                self.current_force_visualizer = VisualizationMarkers(self.cfg.current_force_visualizer_cfg)
                self.target_region_visualizer = VisualizationMarkers(self.cfg.target_region_visualizer_cfg)
            self.goal_force_visualizer.set_visibility(True)
            self.current_force_visualizer.set_visibility(True)
            self.target_region_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_force_visualizer"):
                self.goal_force_visualizer.set_visibility(False)
                self.current_force_visualizer.set_visibility(False)
                self.target_region_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return

        identity_quat = torch.zeros(self.num_envs, 4, device=self.device)
        identity_quat[:, 0] = 1.0
        self.target_region_visualizer.visualize(self.pos_command_w, identity_quat)

        goal_scale, goal_quat = resolve_force_to_arrow(
            self.force_command_w,
            tuple(self.cfg.goal_force_visualizer_cfg.markers["arrow"].scale),
        )
        current_scale, current_quat = resolve_force_to_arrow(
            self.measured_force_w,
            tuple(self.cfg.current_force_visualizer_cfg.markers["arrow"].scale),
        )
        self.goal_force_visualizer.visualize(self.pos_command_w, goal_quat, goal_scale)
        self.current_force_visualizer.visualize(self.pos_command_w, current_quat, current_scale)
