import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp as mdp
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG
from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.events import ARM_JOINT_NAMES
from HumanoidRLPackage.HumanoidRLSetup.tasks.badminton.mdp.rewards import DEFAULT_RACKET_BODY_NAMES


@configclass
class BadmintonSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    robot = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


@configclass
class CommandsCfg:
    intercept = mdp.UniformInterceptCommandCfg(
        asset_name="robot",
        resampling_time_range=(5.0, 5.0),
        debug_vis=True,
        window_duration_s=0.4,
        ranges=mdp.UniformInterceptCommandCfg.Ranges(
            pos_x=(-0.55, -0.15),
            pos_y=(-0.45, 0.45),
            pos_z=(0.15, 0.75),
            lead_time=(1.5, 3.5),
        ),
    )


@configclass
class ActionsCfg:
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=ARM_JOINT_NAMES,
        scale=0.5,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        intercept_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "intercept"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_robot_joints = EventTerm(
        func=mdp.reset_arm_and_racket_grip,
        mode="reset",
        params={
            "position_range": (0.75, 1.25),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    # Phase 1: always-on spatial tracking toward the intercept point.
    intercept_proximity = RewTerm(
        func=mdp.intercept_proximity_tanh,
        weight=0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "std": 0.15,
            "command_name": "intercept",
        },
    )

    # Phase 2: timed swing — reward being at the target during the hit window.
    timed_intercept = RewTerm(
        func=mdp.timed_intercept_proximity,
        weight=1.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "std": 0.10,
            "command_name": "intercept",
        },
    )
    timed_hit = RewTerm(
        func=mdp.timed_hit_bonus,
        weight=5.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "hit_radius": 0.08,
            "command_name": "intercept",
        },
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.03)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES)},
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.05, "num_steps": 15000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.15, "num_steps": 15000},
    )


@configclass
class BadmintonEnvCfg(ManagerBasedRLEnvCfg):
    scene: BadmintonSceneCfg = BadmintonSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.viewer.eye = (2.5, 2.5, 2.0)
        self.sim.dt = 1.0 / 60.0
