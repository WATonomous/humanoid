from __future__ import annotations

from dataclasses import MISSING

from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.ring_marker_utils import (
    build_intercept_target_visualizer_cfg,
)

from .intercept_command import UniformInterceptCommand


@configclass
class UniformInterceptCommandCfg(CommandTermCfg):
    """Configuration for a timed 3D intercept (badminton hit target) command."""

    class_type: type = UniformInterceptCommand

    asset_name: str = MISSING
    """Robot asset used to transform commands into the world frame."""

    window_duration_s: float = 0.4
    """Duration of the hit window once it opens [s]."""

    min_ring_scale: float = 0.35
    """Ring scale at the moment the hit window opens (timing shrink effect)."""

    target_tilt_pitch_rad: float = 0.55
    """Pitch tilt applied to the flat target disk [rad]."""

    target_tilt_yaw_rad: float = 0.15
    """Yaw tilt applied to the flat target disk [rad]."""

    @configclass
    class Ranges:
        """Uniform distribution ranges for intercept commands."""

        pos_x: tuple[float, float] = MISSING
        pos_y: tuple[float, float] = MISSING
        pos_z: tuple[float, float] = MISSING
        lead_time: tuple[float, float] = MISSING
        """Seconds after resample before the hit window opens."""

    ranges: Ranges = MISSING

    target_visualizer_cfg = build_intercept_target_visualizer_cfg()
