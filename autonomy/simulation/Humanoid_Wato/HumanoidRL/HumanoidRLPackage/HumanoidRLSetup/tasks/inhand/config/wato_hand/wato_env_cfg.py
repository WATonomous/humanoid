from isaaclab.managers import EventTermCfg as EventTerm, RewardTermCfg as RewTerm, SceneEntityCfg
from isaaclab.utils import configclass

import HumanoidRLPackage.HumanoidRLSetup.tasks.inhand.inhand_env_cfg as inhand_env_cfg
import HumanoidRLPackage.HumanoidRLSetup.tasks.inhand.mdp as inhand_mdp
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_hand import (
    INHAND_WATO_HAND_CFG,
    INHAND_CUBE_POS,
    INHAND_SPREAD_RAD,
)


@configclass
class WatoHandCubeEnvCfg(inhand_env_cfg.InHandObjectEnvCfg):
    """In-hand cube reorientation for the 20-DOF Wato hand."""

    def __post_init__(self):
        super().__post_init__()

        # Share physics across envs — lower RAM per env, scales to more parallel rollouts.
        self.scene.replicate_physics = True
        self.scene.num_envs = 1024

        self.scene.robot = INHAND_WATO_HAND_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # Push expanded MCP_A limits (±27 deg) into PhysX, overriding the ±8.6 deg baked in the USD.
        self.events.expand_abduction_limits = EventTerm(
            func=inhand_mdp.apply_wato_hand_joint_limits,
            mode="startup",
        )

        # change the cube on plam's property
        self.scene.object.spawn.scale = (0.8, 0.8, 0.8)
        self.scene.object.init_state.pos = INHAND_CUBE_POS
        self.scene.object.init_state.rot = (1.0, 0.0, 0.0, 0.0)

        _grasp_scale = [0.2, 0.2]
        _splay_scale = [1.0, 1.0]
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

        # Curriculum: start with rotation about z-axis (palm normal) only.
        # Full 3D random orientation is too hard to explore from scratch; z-axis
        # rotation (spinning in the palm plane) is the most natural motion for this hand.
        # Once orientation_error shows a downward trend, expand back to ["x", "y"].
        self.commands.object_pose.rotation_axes = ["z"]

        # Small bonus for MCP_A velocity + spread deflection.
        self.rewards.spread_activity = RewTerm(
            func=inhand_mdp.mcp_a_spread_activity,
            weight=0.03,
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "spread_limit": INHAND_SPREAD_RAD * 2.0,
            },
        )


@configclass
class WatoHandCubeEnvCfg_PLAY(WatoHandCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.observations.policy.enable_corruption = False
        # Keep time_out so play episodes reset instead of clamping forever.
        self.episode_length_s = 10.0
        # Nudge goal marker aside so it does not cover the physical cube.
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
        # Keep time_out so play episodes reset instead of clamping forever.
        self.episode_length_s = 10.0
        # Nudge goal marker aside so it does not cover the physical cube.
        self.commands.object_pose.marker_pos_offset = (-0.10, 0.0, 0.12)
