"""Garment-fold env config for the WATonomous pioneer_bimanual_arm.

NEW (WATonomous) -- not vendored. Subclasses the robot-agnostic `GarmentEnvCfg`
and plugs in a single `pioneer_bimanual_arm` articulation in place of upstream's
two SO101 follower arms.
"""
from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from humanoid_garment_fold.tasks.garment_env_cfg import GarmentEnvCfg
from humanoid_garment_fold.assets.robots.bimanual_arm import (
    BIMANUAL_ARM_CFG,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    LEFT_GRIPPER_JOINTS,
    RIGHT_GRIPPER_JOINTS,
)

__all__ = [
    "GarmentPioneerEnvCfg",
    "LEFT_ARM_JOINTS", "RIGHT_ARM_JOINTS",
    "LEFT_GRIPPER_JOINTS", "RIGHT_GRIPPER_JOINTS",
]

_WRIST_OPTICS = sim_utils.PinholeCameraCfg(
    focal_length=36.5, focus_distance=400.0, horizontal_aperture=36.83,
    clipping_range=(0.01, 50.0), lock_camera=True,
)
_TOP_OPTICS = sim_utils.PinholeCameraCfg(
    focal_length=28.7, focus_distance=400.0, horizontal_aperture=38.11,
    clipping_range=(0.01, 50.0), lock_camera=True,
)


@configclass
class GarmentPioneerEnvCfg(GarmentEnvCfg):
    # 12-dim action: 6 left-arm revolute + 6 right-arm revolute joint-position
    # targets. Grippers are held open (see GarmentPioneerEnv._apply_action).
    action_space = 12
    observation_space = 12

    # one articulation, replaces upstream left_robot + right_robot
    robot: ArticulationCfg = BIMANUAL_ARM_CFG.replace(prim_path="/World/Robot")

    # The pioneer arm's REACH/FRONT axis is +X -- established by this repo's
    # tools/isaac_harness/scenes/bimanual_vial_rack.sh and pick_place_bimanual
    # (robot at identity; TABLE_X_MIN=0.18, TABLE_TOP_Z=0.05, table centre x=0.63).
    # LeHome's garment sits at world ~(0, 0, 0.63), so rotate the base +90 deg
    # about Z to face +Y toward it, sit 0.63 m behind in -Y and ~0.05 m above.
    # Measured EE at this pose: link6l (LEFT wrist) world ~(-0.24, -0.11, 0.55).
    robot_base_pos: tuple = (0.0, -0.63, 0.68)
    robot_base_rot: tuple = (0.7071068, 0.0, 0.0, 0.7071068)  # wxyz, +90 deg Z

    # Optional photoreal NuRec backdrop (visual only, no collision). Off by
    # default -- needs the NuRec renderer + manual alignment.
    backdrop_usd_path: str | None = None
    backdrop_pos: tuple = (0.0, 0.0, 0.0)
    backdrop_scale: float = 1.0

    # --- cameras: wrist cams on the pioneer wrist links, top cam world-fixed ---
    left_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/Robot/link6l/left_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04),
            rot=(-0.404379, -0.912179, -0.0451242, 0.0486914),
            convention="ros",
        ),
        data_types=["rgb"], spawn=_WRIST_OPTICS,
        width=640, height=480, update_period=1 / 30.0,
    )
    right_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/Robot/link6/right_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04),
            rot=(-0.404379, -0.912179, -0.0451242, 0.0486914),
            convention="ros",
        ),
        data_types=["rgb"], spawn=_WRIST_OPTICS,
        width=640, height=480, update_period=1 / 30.0,
    )
    top_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/TopCam",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 1.1), rot=(0.0, 1.0, 0.0, 0.0), convention="ros"
        ),
        data_types=["rgb", "depth"], spawn=_TOP_OPTICS,
        width=640, height=480,
    )
    # extra fixed camera for an external "is the arm placed right" view (dev only)
    scene_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/SceneCam",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(1.8, -2.0, 1.3), rot=(1.0, 0.0, 0.0, 0.0), convention="world",
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=20.0, focus_distance=400.0, horizontal_aperture=30.0,
            clipping_range=(0.05, 60.0),
        ),
        width=960, height=640,
    )
