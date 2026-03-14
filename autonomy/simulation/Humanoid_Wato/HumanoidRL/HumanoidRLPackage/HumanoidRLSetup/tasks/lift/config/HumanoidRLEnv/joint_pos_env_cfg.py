import math

from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.lift import mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.lift.lift_env_cfg import LiftEnvCfg


@configclass
class HumanoidArmLiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the humanoid arm (mu robot)."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot.init_state.pos = (0.0, 0.0, 0.1)

        # Arm action: all joints (arm + hand), same as manipulation
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            scale=0.5,
            use_default_offset=True,
        )
        # Gripper: MCP finger joints for open/close
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["mcp_.*"],
            open_command_expr={"mcp_.*": 0.5},
            close_command_expr={"mcp_.*": 0.0},
        )

        # Command generator: target pose relative to end-effector body (DIP_INDEX = index fingertip)
        self.commands.object_pose.body_name = "DIP_INDEX_v1_.*"
        self.commands.object_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # ee_frame removed: object_ee_distance uses robot body (DIP_INDEX_v1_.*) when no ee_frame in scene


@configclass
class HumanoidArmLiftEnvCfg_PLAY(HumanoidArmLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
