"""Leader-arm angle -> sim target helpers for ``pioneer_leader_arm_teleop.py``.

Joint targets are clamped to the arm's URDF limits (passed in by the caller), so the sim
arm cannot be driven past what the real arm can reach. The leader itself is an input device
with torque always off; nothing here can move a physical actuator.
"""
from __future__ import annotations

import math

# Gripper servo (G) range, in degrees. The claw starts OPEN at GRIPPER_OPEN_DEG
# (its maximum) and closes toward GRIPPER_CLOSED_DEG (its minimum). The leader
# zeroes wherever it starts, so the startup pose is taken to be 41.5 deg -- zero
# the leader with the claw fully open. See gripper_fraction().
GRIPPER_OPEN_DEG = 41.5
GRIPPER_CLOSED_DEG = 0.0


def limited_target(
    angle_rad: float,
    sign: float,
    scale: float,
    bounds_deg: tuple[float, float],
    offset_rad: float = 0.0,
) -> float:
    """Map one leader angle to a clamped sim joint target, in radians.

    ``angle_rad`` is the leader's startup-relative angle, ``sign`` flips the
    axis direction (see ``--signs``) and ``scale`` is the leader-to-sim gain.
    ``offset_rad`` is the sim joint angle the leader's zero maps to (its home
    pose); the leader's motion is added on top of it before the clamp.
    The result is hard-clamped into ``bounds_deg``; callers rely on that clamp
    rather than checking the range themselves, so it must never be bypassed.
    """
    lo_deg, hi_deg = bounds_deg
    if not (math.isfinite(lo_deg) and math.isfinite(hi_deg)) or lo_deg >= hi_deg:
        raise ValueError(f"invalid bounds {bounds_deg!r}: need finite lo < hi")
    value = offset_rad + angle_rad * sign * scale
    if not math.isfinite(value):
        # A NaN would propagate straight into a position target; hold the home pose instead.
        value = offset_rad
    return max(math.radians(lo_deg), min(math.radians(hi_deg), value))


def gripper_fraction(angle_rad: float, sign: float) -> float:
    """Map the gripper servo's reading to a closure fraction: 0 = open, 1 = closed.

    ``angle_rad`` is startup-relative, and startup is the open end, so the claw's
    angle is ``GRIPPER_OPEN_DEG + sign * angle``, clamped into
    [GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG]. With ``sign`` = +1 the servo reading
    has to DECREASE to close; if closing the leader claw does nothing in sim,
    flip the gripper's sign. A non-finite reading returns 0 (open) rather than
    propagating into a finger target.
    """
    span = GRIPPER_OPEN_DEG - GRIPPER_CLOSED_DEG
    claw_deg = GRIPPER_OPEN_DEG + sign * math.degrees(angle_rad)
    if not math.isfinite(claw_deg):
        return 0.0
    claw_deg = max(GRIPPER_CLOSED_DEG, min(GRIPPER_OPEN_DEG, claw_deg))
    return (GRIPPER_OPEN_DEG - claw_deg) / span
