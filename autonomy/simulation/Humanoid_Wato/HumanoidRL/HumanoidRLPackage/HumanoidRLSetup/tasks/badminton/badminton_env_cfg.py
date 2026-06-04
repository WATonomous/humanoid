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
        hit_moment_duration_s=0.13,
        debug_vis=True,
        ranges=mdp.UniformInterceptCommandCfg.Ranges(
            pos_x=(-0.55, -0.15),
            pos_y=(-0.45, 0.45),
            pos_z=(0.15, 0.75),
            lead_time=(1.5, 3.5),
            speed=(0.4, 1.5),
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
    # Phase A — timing + position (urgency-gated so arriving early pays less).
    intercept_proximity = RewTerm(
        func=mdp.intercept_proximity_timed_tanh,
        weight=2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "std": 0.15,
            "command_name": "intercept",
            "urgency_time_constant": 0.8,
            "prep_floor": 0.15,
        },
    )
    ee_position_approach = RewTerm(
        func=mdp.ee_position_approach_exp,
        weight=1.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "pos_std": 0.12,
            "urgency_time_constant": 0.8,
        },
    )
    early_at_target = RewTerm(
        func=mdp.early_at_target_penalty,
        weight=-0.4,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "zone_radius": 0.13,
            "min_lead_time_remaining": 0.35,
        },
    )
    # Phase B — impact (position on hit pulse; vel/ori ramp in via curriculum).
    ee_impact_position = RewTerm(
        func=mdp.ee_impact_position_hit_exp,
        weight=8.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "pos_std": 0.10,
        },
    )
    ee_impact_velocity = RewTerm(
        func=mdp.ee_impact_velocity_hit_exp,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "vel_std": 0.5,
            "hit_radius": 0.13,
        },
    )
    ee_impact_orientation = RewTerm(
        func=mdp.ee_impact_orientation_hit_exp,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "ori_std": 0.8,
            "hit_radius": 0.13,
        },
    )
    racket_speed_far = RewTerm(
        func=mdp.racket_speed_penalty_far_from_target,
        weight=-0.15,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "distance_threshold": 0.13,
        },
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
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
    # After ~12 m mean position error, ramp swing speed matching at impact.
    # common_step_counter += 1 per sim step (~24 per PPO iter → 300 iter ≈ 7200 steps).
    ee_impact_velocity = CurrTerm(
        func=mdp.ramp_reward_weight,
        params={
            "term_name": "ee_impact_velocity",
            "start_weight": 0.0,
            "end_weight": 10.0,
            "start_step": 800,
            "end_step": 4500,
        },
    )
    ee_impact_orientation = CurrTerm(
        func=mdp.ramp_reward_weight,
        params={
            "term_name": "ee_impact_orientation",
            "start_weight": 0.0,
            "end_weight": 4.0,
            "start_step": 2500,
            "end_step": 6000,
        },
    )
    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.08, "num_steps": 15000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.02, "num_steps": 15000},
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
