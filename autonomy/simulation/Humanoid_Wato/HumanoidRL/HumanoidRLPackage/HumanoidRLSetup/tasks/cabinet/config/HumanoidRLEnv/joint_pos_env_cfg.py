from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.cabinet import mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.cabinet.cabinet_env_cfg import CabinetEnvCfg
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG


@configclass
class HumanoidArmCabinetEnvCfg(CabinetEnvCfg):
    """Cabinet open-drawer task for the humanoid arm (mu robot)."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.pos = (-0.1, 0.0, 0.4)

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


        # Cabinet scale 1.15 so handle gap is large enough for humanoid fingers.
        self.scene.cabinet.spawn.scale = (1.15, 1.15, 1.15)
        self.scene.cabinet.init_state.pos = (0.75, 0.0, 0.35)

        # Reward params: gripper offset and grasp open position (MCP open = 0.5).
        # Grasp reward uses only the four fingers (index, middle, ring, pinky), not thumb.
        self.rewards.approach_gripper_handle.params["offset"] = 0.04
        self.rewards.grasp_handle.params["open_joint_pos"] = 0.5
        self.rewards.grasp_handle.params["asset_cfg"].joint_names = [
            "mcp_index",
            "mcp_middle",
            "mcp_ring",
            "mcp_pinky",
        ]


@configclass
class HumanoidArmCabinetEnvCfg_PLAY(HumanoidArmCabinetEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
