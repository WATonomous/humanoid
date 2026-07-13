"""Configuration surface for the generalized pick-and-place task.

One dataclass consumed by BOTH the Isaac Lab env cfg (scene/events) and the
cuRobo expert/orchestrator (waypoints, noise, success checks), loaded from a
YAML file. See pick_place_gen/README.md for a field-by-field guide and
worked examples (plain pick-place, stacking, camera-less fast mode).

All poses/ranges are in the robot base frame (== env-local frame; the robot
sits at the env origin). Defaults come from the validated workspace in
wato_constants.py.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from typing import Optional

import yaml

import wato_constants as wc


@dataclass
class ObjectParams:
    """The object to pick."""
    size: tuple = (0.04, 0.04, 0.04)      # cuboid edge lengths [m]
    mass: float = 0.05                     # [kg]
    color: tuple = (0.8, 0.1, 0.1)         # RGB 0-1
    x_range: tuple = wc.WORKSPACE_X        # spawn range [m]
    y_range: tuple = wc.WORKSPACE_Y
    yaw_range: tuple = (-3.14159, 3.14159)


@dataclass
class PlaceParams:
    """Where to place it."""
    mode: str = "table"                    # "table": random pose on the table | "stack": on top of
                                           # place_object | "tray": centre of a fixed tray (tray.usda)
    x_range: tuple = wc.WORKSPACE_X        # target sample range (table mode)
    y_range: tuple = wc.WORKSPACE_Y
    min_separation: float = 0.12           # min XY distance object <-> target at reset [m]
    xy_tolerance: float = 0.03             # success: object center within this XY radius [m]
    z_tolerance: float = 0.02              # success: object height within this of target [m]
    # stack mode only: the base object
    stack_object_size: tuple = (0.05, 0.05, 0.05)
    stack_object_mass: float = 0.10
    stack_object_color: tuple = (0.1, 0.2, 0.8)
    # tray mode only: a fixed white tray; the cube is placed in its centre
    tray_scale: float = 0.7                # footprint scale on the tray asset (0.20x0.16 m)
    tray_height_scale: float = 1.0         # extra scale on the wall HEIGHT only (relative
                                           # to tray_scale); <1 gives lower walls so the
                                           # gripper clears the rim when grasping nearby
    tray_center: tuple = (0.33, -0.16)     # tray interior centre in the env frame [m]


@dataclass
class MotionParams:
    """Expert motion shaping (cuRobo phase targets)."""
    hover_offset: float = 0.10             # pre-grasp hover above the grasp pose [m]
    place_hover_offset: float = 0.06       # hover above the place pose — lower than the
                                           # grasp hover; poses near the workspace ceiling
                                           # (z ~0.20+) plan poorly
    grasp_tip_depth: float = 0.010         # fingertip-center below object top at grasp [m]
    lift_height: float = 0.16              # transit height above table top [m]
    place_clearance: float = 0.005         # drop gap above the support surface [m]
    yaw_candidates: int = wc.NUM_GRASP_YAWS  # top-down grasp yaw goalset size
    max_plan_attempts: int = 3
    time_dilation: float = 0.6             # slow cuRobo trajectories (1.0 = planner speed)


@dataclass
class NoiseParams:
    """Per-episode diversity + per-step actuation noise (recovery signal)."""
    enabled: bool = True
    hover_jitter: float = 0.01             # SD [m] of the off-nominal approach-hover
                                           # detour (lateral + vertical); the pick then
                                           # converges back before the clean vertical
                                           # descent (0 = straight nominal approach)
    via_point_prob: float = 0.7            # probability of a random transit via-point
    via_point_lateral: float = 0.08        # via-point lateral magnitude [m]
    via_point_vertical: float = 0.05       # via-point vertical magnitude [m]
    joint_noise_std_deg: float = 0.25      # Gaussian noise on streamed joint targets
    grasp_yaw_jitter_deg: float = 8.0      # jitter on chosen grasp yaw
    friction_range: tuple = (0.6, 1.2)     # object static/dynamic friction randomization
    restitution_range: tuple = (0.0, 0.1)


@dataclass
class SuccessParams:
    settle_steps: int = 25                 # consecutive control steps object must hold pose
    max_lin_vel: float = 0.02              # [m/s] object considered settled below this
    post_release_timeout_s: float = 3.0


@dataclass
class CameraParams:
    enabled: bool = True
    width: int = 640
    height: int = 480
    external: bool = True                  # fixed over-table view
    wrist: bool = True                     # mounted on link6l


@dataclass
class EpisodeParams:
    episode_length_s: float = 30.0
    sim_dt: float = 0.01
    decimation: int = 2                    # control rate = 1 / (sim_dt * decimation)
    record_divisor: int = 2                # record every Nth control step
    phase_timeout_s: float = 8.0           # per-phase watchdog
    # ACTUATOR EFFORT OVERRIDES (this task's env only, never the shared teleop
    # config). bimanual_arm_cfg uses RATED torques, which saturate under the
    # arm's own gravity load (measured: joint2l sags 0.21 rad at 18 Nm ->
    # ~9 cm fingertip error; joint6l sags ~0.5 rad at 0.25 Nm). Shoulder and
    # elbow use the motors' documented PEAK torques (AK10-9: 53 Nm,
    # AK80-9: 22 Nm). The wrist value EXCEEDS the GL40 peak (0.73 Nm) —
    # flagged for hardware review: this wrist motor cannot statically hold
    # the gripper in sim, and likely not on the real arm either.
    shoulder_effort_limit: float = 53.0    # [Nm] joint1L/joint2l (AK10-9 peak)
    elbow_effort_limit: float = 22.0       # [Nm] joint3l/4l/5l (AK80-9 peak)
    wrist_effort_limit: float = 5.0        # [Nm] joint6l (BEYOND GL40 spec)
    gripper_effort_limit: float = 30.0     # [N]  joint7l/joint8l (unchanged default)

    @property
    def control_hz(self) -> float:
        return 1.0 / (self.sim_dt * self.decimation)

    @property
    def record_fps(self) -> int:
        return round(self.control_hz / self.record_divisor)


@dataclass
class PickPlaceTaskParams:
    object: ObjectParams = field(default_factory=ObjectParams)
    place: PlaceParams = field(default_factory=PlaceParams)
    motion: MotionParams = field(default_factory=MotionParams)
    noise: NoiseParams = field(default_factory=NoiseParams)
    success: SuccessParams = field(default_factory=SuccessParams)
    cameras: CameraParams = field(default_factory=CameraParams)
    episode: EpisodeParams = field(default_factory=EpisodeParams)

    @staticmethod
    def from_yaml(path: Optional[str]) -> "PickPlaceTaskParams":
        params = PickPlaceTaskParams()
        if path is None:
            return params
        with open(path) as f:
            overrides = yaml.safe_load(f) or {}
        _apply_overrides(params, overrides, context=path)
        return params


def _apply_overrides(obj, overrides: dict, context: str) -> None:
    valid = {f.name: f for f in fields(obj)}
    for key, value in overrides.items():
        if key not in valid:
            raise KeyError(f"{context}: unknown field '{key}' for {type(obj).__name__} "
                           f"(valid: {sorted(valid)})")
        current = getattr(obj, key)
        if is_dataclass(current):
            if not isinstance(value, dict):
                raise TypeError(f"{context}: '{key}' expects a mapping")
            _apply_overrides(current, value, context)
        else:
            if isinstance(current, tuple) and isinstance(value, list):
                value = tuple(value)
            setattr(obj, key, value)
