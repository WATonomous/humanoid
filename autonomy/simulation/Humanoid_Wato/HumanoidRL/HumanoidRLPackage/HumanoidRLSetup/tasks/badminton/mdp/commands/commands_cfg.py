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
    """Configuration for a timed EE intercept (badminton shuttle arrival) command."""

    class_type: type = UniformInterceptCommand

    asset_name: str = MISSING
    """Robot asset used to transform commands into the world frame."""

    tracking_body_names: list[str] | str = "DIP_INDEX_v1_.*"
    """Body name(s) used for command metrics (closest link if multiple)."""

    hit_moment_duration_s: float = 0.0
    """How long the hit-moment pulse stays active [s]. 0 = one env step (~67 ms)."""

    post_hit_ring_hidden: bool = True
    """After the contact flash, hide rings until the next resample (visualization only)."""

    min_ring_scale: float = 0.35
    """Debug-vis ring scale on the one-step shuttle-contact flash."""

    @configclass
    class Ranges:
        """Uniform distribution ranges for intercept commands."""

        pos_x: tuple[float, float] = MISSING
        pos_y: tuple[float, float] = MISSING
        pos_z: tuple[float, float] = MISSING
        lead_time: tuple[float, float] = MISSING
        """Seconds after resample until the shuttle arrives at the intercept point."""
        roll: tuple[float, float] = (-0.15, 0.15)
        """Desired EE roll at impact [rad] (base frame)."""
        pitch: tuple[float, float] = (0.45, 0.65)
        """Desired EE pitch at impact [rad] (base frame, ~racket face tilt)."""
        yaw: tuple[float, float] = (-0.35, 0.35)
        """Desired EE yaw at impact [rad] (base frame)."""
        speed: tuple[float, float] = (0.4, 1.5)
        """Desired EE linear speed at impact [m/s] along base→intercept axis."""

    ranges: Ranges = MISSING

    target_visualizer_cfg = build_intercept_target_visualizer_cfg()
