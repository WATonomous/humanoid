import math

from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from HumanoidRLPackage.HumanoidRLSetup.tasks.lift import mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.lift.lift_env_cfg import LiftEnvCfg
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG

from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip


@configclass
class HumanoidArmLiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the humanoid arm (mu robot)."""

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
        # Gripper: MCP finger joints for open/close
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["mcp_.*"],
            open_command_expr={"mcp_.*": 0.5},
            close_command_expr={"mcp_.*": 0.0},
        )

        # Command generator: target pose relative to end-effector body
        self.commands.object_pose.body_name = "DIP_INDEX_v1_.*"
        self.commands.object_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # Cube object (workspace similar to openarm)
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # End-effector frame for object_ee_distance and object_goal_distance (single frame)
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/DIP_INDEX_v1_0",
                    name="end_effector",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.0)),
                ),
            ],
        )


@configclass
class HumanoidArmLiftEnvCfg_PLAY(HumanoidArmLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
