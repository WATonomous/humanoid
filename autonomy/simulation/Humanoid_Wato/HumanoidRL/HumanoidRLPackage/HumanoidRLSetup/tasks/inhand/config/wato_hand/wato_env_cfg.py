from isaaclab.utils import configclass

import HumanoidRLPackage.HumanoidRLSetup.tasks.inhand.inhand_env_cfg as inhand_env_cfg
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_hand import (
    INHAND_WATO_HAND_CFG,
    INHAND_CUBE_POS,
)


@configclass
class WatoHandCubeEnvCfg(inhand_env_cfg.InHandObjectEnvCfg):
    """In-hand cube reorientation for the 20-DOF Wato hand."""

    def __post_init__(self):
        super().__post_init__()

        # Wato hand USD is not instanceable; each env clones the full hand mesh.
        self.scene.replicate_physics = False
        # 2048 envs OOM-kills on ~32 GB RAM; override with --num_envs if you have headroom.
        self.scene.num_envs = 256

        self.scene.robot = INHAND_WATO_HAND_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # Paired with _INHAND_PALM_UP_ROT in modelCfg/wato_hand.py.
        self.scene.object.spawn.scale = (0.6, 0.6, 0.6)
        self.scene.object.init_state.pos = INHAND_CUBE_POS
        self.scene.object.init_state.rot = (1.0, 0.0, 0.0, 0.0)

        # Keep joint_names=[".*"] (all 20 joints) — a partial joint subset triggers a
        # shape mismatch in EMAJointPositionToLimitsAction.reset() on this Isaac Lab version.
        # Replace ".*" position_range: overlapping patterns duplicate joint ids (20 -> 24) and crash reset.
        _grasp_scale = [0.2, 0.2]
        # Compensates for use_default_offset (+0.075 rad): [1.5, 0.5] maps
        # soft limits [-0.15, +0.15] to sample range [-0.15, +0.15] after offset,
        # giving uniform coverage of the full abduction range at reset.
        _splay_scale = [1.5, 0.5]
        self.events.reset_robot_joints.params["position_range"] = {
            "circumduction": _grasp_scale,
            "MCP_A_thumb": _grasp_scale,
            "PIP_thumb": _grasp_scale,
            "DIP_thumb": _grasp_scale,
            "MCP_A_1": _splay_scale,
            "MCP_1": _grasp_scale,
            "PIP_1": _grasp_scale,
            "DIP_1": _grasp_scale,
            "MCP_A_2": _splay_scale,
            "MCP_2": _grasp_scale,
            "PIP_2": _grasp_scale,
            "DIP_2": _grasp_scale,
            "MCP_A_3": _splay_scale,
            "MCP_3": _grasp_scale,
            "PIP_3": _grasp_scale,
            "DIP_3": _grasp_scale,
            "MCP_A_4": _splay_scale,
            "MCP_4": _grasp_scale,
            "PIP_4": _grasp_scale,
            "DIP_4": _grasp_scale,
        }


@configclass
class WatoHandCubeEnvCfg_PLAY(WatoHandCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.terminations.time_out = None
        self.commands.object_pose.debug_vis = True
        # Goal-orientation marker offset (green DexCube) — not the physical object pose.
        self.commands.object_pose.marker_pos_offset = (-0.10, 0.0, 0.12)


@configclass
class WatoHandCubeNoVelObsEnvCfg(WatoHandCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.observations.policy = inhand_env_cfg.ObservationsCfg.NoVelocityKinematicObsGroupCfg()


@configclass
class WatoHandCubeNoVelObsEnvCfg_PLAY(WatoHandCubeNoVelObsEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        self.terminations.time_out = None
        self.commands.object_pose.debug_vis = True
        # Goal-orientation marker offset (green DexCube) — not the physical object pose.
        self.commands.object_pose.marker_pos_offset = (-0.10, 0.0, 0.12)
