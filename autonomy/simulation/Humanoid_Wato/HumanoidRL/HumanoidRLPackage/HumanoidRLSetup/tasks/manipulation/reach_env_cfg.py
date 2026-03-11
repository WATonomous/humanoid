from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import HumanoidRLPackage.HumanoidRLSetup.tasks.manipulation.mdp as mdp

@configclass
class ReachSceneCfg(InteractiveSceneCfg):
    """Configuration for the scene with a robotic arm."""

    # world
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # robots
    robot: ArticulationCfg = MISSING

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="DIP_INDEX_v1_.*",
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(-0.5, 0.5),
            pos_y=(0.62, 0.66),
            pos_z=(0.3, 0.7),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
    )

    # ee_pose_2 = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="DIP_MIDDLE_v1_.*",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         pos_x=(-0.20, -0.14),
    #         pos_y=(0.64, 0.68),
    #         pos_z=(0.38, 0.42),
    #         roll=(0.0, 0.0),
    #         pitch=(0.0, 0.0),
    #         yaw=(0.0, 0.0),
    #     ),
    # )

    # ee_pose_3 = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="DIP_RING_v1_.*",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         pos_x=(-0.22, -0.16),
    #         pos_y=(0.62, 0.66),
    #         pos_z=(0.38, 0.42),
    #         roll=(0.0, 0.0),
    #         pitch=(0.0, 0.0),
    #         yaw=(0.0, 0.0),
    #     ),
    # )

    # ee_pose_4 = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="DIP_PINKY_v1_.*",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         pos_x=(-0.24, -0.18),
    #         pos_y=(0.60, 0.64),
    #         pos_z=(0.38, 0.42),
    #         roll=(0.0, 0.0),
    #         pitch=(0.0, 0.0),
    #         yaw=(0.0, 0.0),
    #     ),
    # )

    # ee_pose_5 = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="IP_THUMB_v1_.*",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         pos_x=(-0.16, -0.10),
    #         pos_y=(0.52, 0.58),
    #         pos_z=(0.44, 0.50),
    #         roll=(0.0, 0.0),
    #         pitch=(0.0, 0.0),
    #         yaw=(0.0, 0.0),
    #     ),
    # )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = MISSING
    gripper_action: ActionTerm | None = None


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
        # pose_command_2 = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose_2"})
        # pose_command_3 = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose_3"})
        # pose_command_4 = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose_4"})
        # pose_command_5 = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose_5"})

        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # task terms
    end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_INDEX_v1_.*"), "command_name": "ee_pose"},
    )
    # end_effector_2_position_tracking = RewTerm(
    #     func=mdp.position_command_error,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_MIDDLE_v1_.*"), "command_name": "ee_pose_2"},
    # )
    # end_effector_3_position_tracking = RewTerm(
    #     func=mdp.position_command_error,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_RING_v1_.*"), "command_name": "ee_pose_3"},
    # )
    # end_effector_4_position_tracking = RewTerm(
    #     func=mdp.position_command_error,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_PINKY_v1_.*"), "command_name": "ee_pose_4"},
    # )
    # end_effector_5_position_tracking = RewTerm(
    #     func=mdp.position_command_error,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="IP_THUMB_v1_.*"), "command_name": "ee_pose_5"},
    # )

    end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.2,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_INDEX_v1_.*"), "std": 0.1, "command_name": "ee_pose"},
    )
    # end_effector_2_position_tracking_fine_grained = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_MIDDLE_v1_.*"), "std": 0.1, "command_name": "ee_pose_2"},
    # )
    # end_effector_3_position_tracking_fine_grained = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_RING_v1_.*"), "std": 0.1, "command_name": "ee_pose_3"},
    # )
    # end_effector_4_position_tracking_fine_grained = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="DIP_PINKY_v1_.*"), "std": 0.1, "command_name": "ee_pose_4"},
    # )
    # end_effector_5_position_tracking_fine_grained = RewTerm(
    #     func=mdp.position_command_error_tanh,
    #     weight=0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="IP_THUMB_v1_.*"), "std": 0.1, "command_name": "ee_pose_5"},
    # )

    # end_effector_orientation_tracking = RewTerm(
    #     func=mdp.orientation_command_error,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="TIP_B_1"), "command_name": "ee_pose"},
    # )
    # end_effector_2_orientation_tracking = RewTerm(
    #     func=mdp.orientation_command_error,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="TIP_B_2"), "command_name": "ee_pose_2"},
    # )
    # end_effector_3_orientation_tracking = RewTerm(
    #     func=mdp.orientation_command_error,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="TIP_B_3"), "command_name": "ee_pose_3"},
    # )
    # end_effector_4_orientation_tracking = RewTerm(
    #     func=mdp.orientation_command_error,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="TIP_B_4"), "command_name": "ee_pose_4"},
    # )
    # end_effector_5_orientation_tracking = RewTerm(
    #     func=mdp.orientation_command_error,
    #     weight=-0.1,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="TIP_B_5"), "command_name": "ee_pose_5"},
    # )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.00001)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0000001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -0.0001, "num_steps": 4500}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 4500}
    )


##
# Environment configuration
##


@configclass
class ReachEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the reach end-effector pose tracking environment."""

    # Scene settings
    scene: ReachSceneCfg = ReachSceneCfg(num_envs=4096, env_spacing=2.5)
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
        self.decimation = 4  # policy every 4 physics steps (was 2) → smoother target updates
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0