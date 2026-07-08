import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.markers.config import CUBOID_MARKER_CFG, FRAME_MARKER_CFG
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.so101 import SO101_FOLLOWER_CFG

from . import mdp

# DexCube in scene (keep marker edge length in sync with spawn scale)
_DEX_CUBE_USD = f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd"
_OBJECT_CUBE_SCALE = (0.5, 0.5, 0.5)
# Nucleus dex_cube_instanceable.usd edge length at unit scale [m]
_DEX_CUBE_UNIT_EDGE_M = 0.05
_OBJECT_CUBE_EDGE_M = _DEX_CUBE_UNIT_EDGE_M * _OBJECT_CUBE_SCALE[0]

# Target object pose (world frame via command manager debug vis)
_GOAL_POSE_MARKER_CFG = CUBOID_MARKER_CFG.replace(prim_path="/Visuals/Command/goal_pose")
_GOAL_POSE_MARKER_CFG.markers["cuboid"].size = (
    _OBJECT_CUBE_EDGE_M,
    _OBJECT_CUBE_EDGE_M,
    _OBJECT_CUBE_EDGE_M,
)
_GOAL_POSE_MARKER_CFG.markers["cuboid"].visual_material = sim_utils.PreviewSurfaceCfg(
    diffuse_color=(0.0, 1.0, 0.0)
)
# Current gripper body pose (optional reference)
_CURRENT_POSE_MARKER_CFG = FRAME_MARKER_CFG.replace(prim_path="/Visuals/Command/body_pose")
_CURRENT_POSE_MARKER_CFG.markers["frame"].scale = (0.05, 0.05, 0.05)


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    robot = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    ee_frame: FrameTransformerCfg = MISSING

    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, 0, 0.035], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=_DEX_CUBE_USD,
            scale=_OBJECT_CUBE_SCALE,
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

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class CommandsCfg:
    object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="gripper",
        # One goal per episode (matches episode_length_s); marker only changes on env reset.
        resampling_time_range=(5.0, 5.0),
        debug_vis=False,
        goal_pose_visualizer_cfg=_GOAL_POSE_MARKER_CFG,
        current_pose_visualizer_cfg=_CURRENT_POSE_MARKER_CFG,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(-0.4, -0.2),  # target cube position (min, max) in robot base frame [m]
            pos_y=(-0.15, 0.15),
            pos_z=(0.1, 0.2),  # lifted goal height, above table grasp (~0.03–0.05 m)
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
    )


@configclass
class ActionsCfg:
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["shoulder_.*", "elbow_flex", "wrist_.*"],
        scale=0.5,
        use_default_offset=True,
    )
    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["gripper"],
        open_command_expr={"gripper": 0.5},
        close_command_expr={"gripper": 0.0},
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={
                                         "command_name": "object_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_cube_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("cube", body_names="Object"),
        },
    )


@configclass
class RewardsCfg:
    reaching_object = RewTerm(
        func=mdp.object_ee_distance,
        params={"std": 0.3},
        weight=1.0,
    )

    lifting_object = RewTerm(func=mdp.object_is_lifted, params={
                             "minimal_height": 0.08}, weight=18.0)

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.03, "command_name": "object_pose"},
        weight=20.0,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.05, "minimal_height": 0.03, "command_name": "object_pose"},
        weight=5.0,
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-5)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-5,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={
            "minimum_height": -0.05, "asset_cfg": SceneEntityCfg("cube")}
    )


@configclass
class CurriculumCfg:
    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={
            "term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={
            "term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )


@configclass
class LiftEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(
        num_envs=512, env_spacing=2.5, replicate_physics=False
    )
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 2**25
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 2**23
        self.sim.physx.gpu_heap_capacity = 2**26
        self.sim.physx.friction_correlation_distance = 0.00625
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.0)

# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Lift-Cube-SO101-v0 --headless

# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Lift-Cube-SO101-Play-v0 --num_envs=1
