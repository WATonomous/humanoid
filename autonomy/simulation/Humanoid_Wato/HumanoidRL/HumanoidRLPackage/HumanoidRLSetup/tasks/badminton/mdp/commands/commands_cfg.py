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
<<<<<<< HEAD
<<<<<<< HEAD
    """Configuration for a timed EE intercept (badminton shuttle arrival) command."""
=======
    """Configuration for a timed 3D intercept (badminton hit target) command."""
>>>>>>> 97ddcbcd (rl-badminton)
=======
    """Configuration for a timed 3D intercept (badminton shuttle arrival) command."""
>>>>>>> bf63d8b3 (rl-badminton)

    class_type: type = UniformInterceptCommand

    asset_name: str = MISSING
    """Robot asset used to transform commands into the world frame."""

<<<<<<< HEAD
<<<<<<< HEAD
    tracking_body_names: list[str] | str = "DIP_INDEX_v1_.*"
    """Body name(s) used for command metrics (closest link if multiple)."""

    hit_moment_duration_s: float = 0.0
    """How long the hit-moment pulse stays active [s]. 0 = one env step (~67 ms)."""

    post_hit_ring_hidden: bool = True
    """After the contact flash, hide rings until the next resample (visualization only)."""

    min_ring_scale: float = 0.35
    """Debug-vis ring scale on the one-step shuttle-contact flash."""
=======
    window_duration_s: float = 0.4
    """Duration of the hit window once it opens [s]."""
=======
    hit_moment_duration_s: float = 0.0
    """How long the hit-moment reward pulse stays active [s]. 0 = one env step (~67 ms)."""

    post_hit_ring_hidden: bool = True
    """After the contact flash, hide rings until the next resample (visualization only)."""
>>>>>>> bf63d8b3 (rl-badminton)

    min_ring_scale: float = 0.35
    """Debug-vis ring scale on the one-step shuttle-contact flash."""

    target_tilt_pitch_rad: float = 0.55
    """Pitch tilt applied to the flat target disk [rad] (visualization only)."""

    target_tilt_yaw_rad: float = 0.15
<<<<<<< HEAD
    """Yaw tilt applied to the flat target disk [rad]."""
>>>>>>> 97ddcbcd (rl-badminton)
=======
    """Yaw tilt applied to the flat target disk [rad] (visualization only)."""
>>>>>>> bf63d8b3 (rl-badminton)

    @configclass
    class Ranges:
        """Uniform distribution ranges for intercept commands."""

        pos_x: tuple[float, float] = MISSING
        pos_y: tuple[float, float] = MISSING
        pos_z: tuple[float, float] = MISSING
        lead_time: tuple[float, float] = MISSING
<<<<<<< HEAD
<<<<<<< HEAD
        """Seconds after resample until the shuttle arrives at the intercept point."""
        roll: tuple[float, float] = (-0.15, 0.15)
        """Desired EE roll at impact [rad] (base frame)."""
        pitch: tuple[float, float] = (0.45, 0.65)
        """Desired EE pitch at impact [rad] (base frame, ~racket face tilt)."""
        yaw: tuple[float, float] = (-0.35, 0.35)
        """Desired EE yaw at impact [rad] (base frame)."""
        speed: tuple[float, float] = (0.4, 1.5)
        """Desired EE linear speed at impact [m/s] along base→intercept axis."""
=======
        """Seconds after resample before the hit window opens."""
>>>>>>> 97ddcbcd (rl-badminton)
=======
        """Seconds after resample until the shuttle arrives at the intercept point."""
>>>>>>> bf63d8b3 (rl-badminton)

    ranges: Ranges = MISSING

    target_visualizer_cfg = build_intercept_target_visualizer_cfg()
