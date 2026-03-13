from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.cabinet import mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.cabinet.cabinet_env_cfg import (
    CabinetEnvCfg,
    FRAME_MARKER_SMALL_CFG,
)
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG


@configclass
class HumanoidArmCabinetEnvCfg(CabinetEnvCfg):
    """Cabinet open-drawer task for the humanoid arm (mu robot)."""

    def __post_init__(self):
        super().__post_init__()

        # Robot: humanoid arm (same as manipulation/reach)
        self.scene.robot = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Arm action: all joints (arm + hand), same as manipulation
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            scale=0.5,
            use_default_offset=True,
        )
        # Gripper: treat MCP finger joints as open/close for grasp reward
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["mcp_.*"],
            open_command_expr={"mcp_.*": 0.5},
            close_command_expr={"mcp_.*": 0.0},
        )

        # End-effector frame: TCP + two fingertips for approach/grasp rewards
        # Uses same body naming as manipulation (DIP_INDEX, IP_THUMB)
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(
                prim_path="/Visuals/EndEffectorFrameTransformer"
            ),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/wrist_extension",
                    name="ee_tcp",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.0)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/DIP_INDEX_v1_0",
                    name="tool_leftfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.0)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/IP_THUMB_v1_0",
                    name="tool_rightfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.0)),
                ),
            ],
        )

        # Cabinet pose for humanoid arm workspace (similar to openarm)
        self.scene.cabinet.spawn.scale = (0.75, 0.75, 0.75)
        self.scene.cabinet.init_state.pos = (0.7, 0.0, 0.3)

        # Reward params: gripper offset and grasp open position (MCP open = 0.5)
        self.rewards.approach_gripper_handle.params["offset"] = 0.04
        self.rewards.grasp_handle.params["open_joint_pos"] = 0.5
        self.rewards.grasp_handle.params["asset_cfg"].joint_names = ["mcp_.*"]


@configclass
class HumanoidArmCabinetEnvCfg_PLAY(HumanoidArmCabinetEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
