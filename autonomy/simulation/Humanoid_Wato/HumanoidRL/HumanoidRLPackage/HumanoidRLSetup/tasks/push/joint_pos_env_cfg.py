"""SO101 joint-position env configs for the push-block task."""

from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import (
    FrameTransformerCfg,
    OffsetCfg,
)
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.modelCfg.so101 import SO101_FOLLOWER_CFG

from . import mdp
from .push_env_cfg import PushBlockEnvCfg

# Match lift task / so101_new_calib.urdf gripper_frame on body "gripper"
_GRIPPER_FRAME_POS = (-0.0079, -0.000218121, -0.07)
_GRIPPER_FRAME_ROT = (0.0, 0.0, 1.0, 0.0)


def _so101_ee_frame_cfg(*, debug_vis: bool) -> FrameTransformerCfg:
    marker_cfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/FrameTransformer/ee_tcp")
    marker_cfg.markers["frame"].scale = (0.03, 0.03, 0.03)
    return FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        debug_vis=debug_vis,
        visualizer_cfg=marker_cfg,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/gripper",
                name="end_effector",
                offset=OffsetCfg(pos=_GRIPPER_FRAME_POS, rot=_GRIPPER_FRAME_ROT),
            ),
        ],
    )


@configclass
class SoArm101PushBlockEnvCfg(PushBlockEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # SO101 USD is not instanceable; physics replication can corrupt GPU buffers.
        self.scene.replicate_physics = False
        self.scene.num_envs = 512
        self.scene.env_spacing = 2.5

        # Arm only: gripper holds closed default and is not actuated.
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_.*", "elbow_flex", "wrist_.*"],
            scale=0.5,
            use_default_offset=True,
        )

        self.scene.ee_frame = _so101_ee_frame_cfg(debug_vis=False)


@configclass
class SoArm101PushBlockEnvCfg_PLAY(SoArm101PushBlockEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.scene.ee_frame = _so101_ee_frame_cfg(debug_vis=True)
