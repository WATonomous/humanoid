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
    """Configuration for a timed 3D intercept (badminton shuttle arrival) command."""

    class_type: type = UniformInterceptCommand

    asset_name: str = MISSING
    """Robot asset used to transform commands into the world frame."""

    hit_moment_duration_s: float = 0.0
    """How long the hit-moment reward pulse stays active [s]. 0 = one env step (~67 ms)."""

    post_hit_ring_hidden: bool = True
    """After the contact flash, hide rings until the next resample (visualization only)."""

    min_ring_scale: float = 0.35
    """Debug-vis ring scale on the one-step shuttle-contact flash."""

    target_tilt_pitch_rad: float = 0.55
    """Pitch tilt applied to the flat target disk [rad] (visualization only)."""

    target_tilt_yaw_rad: float = 0.15
    """Yaw tilt applied to the flat target disk [rad] (visualization only)."""

    @configclass
    class Ranges:
        """Uniform distribution ranges for intercept commands."""

        pos_x: tuple[float, float] = MISSING
        pos_y: tuple[float, float] = MISSING
        pos_z: tuple[float, float] = MISSING
        lead_time: tuple[float, float] = MISSING
        """Seconds after resample until the shuttle arrives at the intercept point."""

    ranges: Ranges = MISSING

    target_visualizer_cfg = build_intercept_target_visualizer_cfg()
