"""Distillation env: privileged teacher obs + camera student obs for push-block."""

from __future__ import annotations

import math

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from . import mdp
from .joint_pos_env_cfg import SoArm101PushBlockEnvCfg, SoArm101PushBlockEnvCfg_PLAY
from .push_env_cfg import FLOOR_TARGET, FLOOR_Z, RAMP_BASE_X, RAMP_TOP_X

# External table-side camera (env frame). Looks at the ramp mouth / block workspace.
_CAM_EYE = (0.55, -0.50, 0.42)
_CAM_TARGET = (0.30, 0.0, 0.05)
_CAM_WIDTH = 64
_CAM_HEIGHT = 64


def _look_at_quat_ros(eye, target, up=(0.0, 0.0, 1.0)) -> tuple[float, float, float, float]:
    """Camera quaternion (wxyz, ROS optical: +Z forward, +Y down)."""
    eye_a, target_a = np.asarray(eye, float), np.asarray(target, float)
    z = target_a - eye_a
    z /= np.linalg.norm(z)
    up_a = np.asarray(up, float)
    x = np.cross(z, up_a)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.stack([x, y, z], axis=1)
    w = math.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2.0
    return (
        float(w),
        float((m[2, 1] - m[1, 2]) / (4 * w)),
        float((m[0, 2] - m[2, 0]) / (4 * w)),
        float((m[1, 0] - m[0, 1]) / (4 * w)),
    )


@configclass
class DistillObservationsCfg:
    """Teacher = privileged policy obs; student = proprio + RGB (no block state)."""

    @configclass
    class TeacherCfg(ObsGroup):
        """Privileged observations matching the trained PPO teacher (37-D)."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        ee_position = ObsTerm(func=mdp.ee_position_in_robot_root_frame)
        block_position = ObsTerm(func=mdp.block_center_in_robot_root_frame)
        block_orientation = ObsTerm(func=mdp.block_orientation)
        block_lin_vel = ObsTerm(func=mdp.block_lin_vel)
        ramp_geometry = ObsTerm(
            func=mdp.ramp_geometry,
            params={
                "ramp_base_x": RAMP_BASE_X,
                "ramp_top_x": RAMP_TOP_X,
                "floor_z": FLOOR_Z,
                "target": FLOOR_TARGET,
            },
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class StudentCfg(ObsGroup):
        """Proprioceptive student terms (no privileged block state)."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        ee_position = ObsTerm(func=mdp.ee_position_in_robot_root_frame)
        ramp_geometry = ObsTerm(
            func=mdp.ramp_geometry,
            params={
                "ramp_base_x": RAMP_BASE_X,
                "ramp_top_x": RAMP_TOP_X,
                "floor_z": FLOOR_Z,
                "target": FLOOR_TARGET,
            },
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class StudentRgbCfg(ObsGroup):
        """RGB image for the student vision encoder."""

        rgb = ObsTerm(
            func=mdp.image,
            params={
                "sensor_cfg": SceneEntityCfg("tiled_camera"),
                "data_type": "rgb",
                "normalize": True,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            # Keep spatial dims — do not flatten the image into a vector.
            self.concatenate_terms = False

    # Keep a `policy` alias so tools that expect it still see teacher-sized obs.
    policy: TeacherCfg = TeacherCfg()
    teacher: TeacherCfg = TeacherCfg()
    student: StudentCfg = StudentCfg()
    student_rgb: StudentRgbCfg = StudentRgbCfg()


@configclass
class SoArm101PushBlockDistillEnvCfg(SoArm101PushBlockEnvCfg):
    """Push-block env with an external camera for vision distillation."""

    observations: DistillObservationsCfg = DistillObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # Cameras are expensive; keep this small for the minimal distill loop.
        self.scene.num_envs = 64
        self.scene.env_spacing = 2.5
        self.sim.render_interval = self.decimation

        self.scene.tiled_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/tiled_camera",
            offset=TiledCameraCfg.OffsetCfg(
                pos=_CAM_EYE,
                rot=_look_at_quat_ros(_CAM_EYE, _CAM_TARGET),
                convention="ros",
            ),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18.0,
                horizontal_aperture=20.955,
                clipping_range=(0.05, 3.0),
            ),
            width=_CAM_WIDTH,
            height=_CAM_HEIGHT,
            data_types=["rgb"],
            update_period=0.0,
        )


@configclass
class SoArm101PushBlockDistillEnvCfg_PLAY(SoArm101PushBlockEnvCfg_PLAY):
    """Play/eval variant of the distillation env."""

    observations: DistillObservationsCfg = DistillObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.sim.render_interval = self.decimation
        self.scene.tiled_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/tiled_camera",
            offset=TiledCameraCfg.OffsetCfg(
                pos=_CAM_EYE,
                rot=_look_at_quat_ros(_CAM_EYE, _CAM_TARGET),
                convention="ros",
            ),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18.0,
                horizontal_aperture=20.955,
                clipping_range=(0.05, 3.0),
            ),
            width=_CAM_WIDTH,
            height=_CAM_HEIGHT,
            data_types=["rgb"],
            update_period=0.0,
        )
