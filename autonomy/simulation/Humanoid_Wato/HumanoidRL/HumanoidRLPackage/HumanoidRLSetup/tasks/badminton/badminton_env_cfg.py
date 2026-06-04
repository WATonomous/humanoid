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
<<<<<<< HEAD
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid_arm_hand import ARM_CFG
=======
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG
>>>>>>> 97ddcbcd (rl-badminton)
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
<<<<<<< HEAD
        # Ignored by UniformInterceptCommand: resample is cycle-aligned (lead_time + hit window).
        resampling_time_range=(5.0, 5.0),
        hit_moment_duration_s=0.20,
        debug_vis=True,
=======
        resampling_time_range=(5.0, 5.0),
        hit_moment_duration_s=0.13,
        debug_vis=True,
<<<<<<< HEAD
        window_duration_s=0.4,
>>>>>>> 97ddcbcd (rl-badminton)
=======
>>>>>>> bf63d8b3 (rl-badminton)
        ranges=mdp.UniformInterceptCommandCfg.Ranges(
            pos_x=(-0.55, -0.15),
            pos_y=(-0.45, 0.45),
            pos_z=(0.15, 0.75),
            lead_time=(1.5, 3.5),
<<<<<<< HEAD
<<<<<<< HEAD
            speed=(0.4, 1.5),
=======
>>>>>>> 97ddcbcd (rl-badminton)
=======
            speed=(0.4, 1.5),
>>>>>>> bfee0731 (improve-badminton-rl)
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    # Only penalty that references pre-impact position: don't camp at the intercept.
    early_at_target = RewTerm(
        func=mdp.early_at_target_penalty,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "zone_radius": 0.13,
            "min_lead_time_remaining": 0.25,
        },
    )
    # End-state tracking × urgency(t): strong near impact, ~0 if far or early (not proximity shaping).
    ee_state_tracking = RewTerm(
        func=mdp.ee_state_tracking_timed_exp,
        weight=12.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "pos_std": 0.10,
            "vel_std": 0.6,
            "ori_std": 0.8,
            "timing_std": 0.45,
            "hit_bonus": 2.0,
        },
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
=======
    # Phase 1: always-on spatial tracking toward the intercept point.
=======
    # Prep: move toward intercept before shuttle arrives (low weight — timing matters more).
>>>>>>> bf63d8b3 (rl-badminton)
=======
    # Phase A — timing + position (urgency-gated so arriving early pays less).
>>>>>>> bfee0731 (improve-badminton-rl)
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
=======
    # Ready pose elsewhere is fine; penalize camping at the intercept long before impact.
>>>>>>> 00aee69e (improve-badminton-rl)
    early_at_target = RewTerm(
        func=mdp.early_at_target_penalty,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "zone_radius": 0.13,
            "min_lead_time_remaining": 0.25,
        },
    )
    # Outside launch window: weak aim cue only (ready pose, know where to strike).
    coarse_aim = RewTerm(
        func=mdp.coarse_aim_toward_intercept_tanh,
        weight=0.6,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "std": 0.45,
            "command_name": "intercept",
            "approach_window_s": 0.55,
        },
    )
    # Launch window: strike-speed approach, gated by distance (must actually close in).
    timed_swing_approach = RewTerm(
        func=mdp.timed_swing_approach_exp,
        weight=4.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "approach_window_s": 0.55,
            "speed_std": 0.6,
            "hit_radius": 0.13,
            "range_std": 0.40,
        },
    )
    # Impact instant: position at the point.
    ee_impact_position = RewTerm(
        func=mdp.ee_impact_position_hit_exp,
        weight=10.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "pos_std": 0.10,
        },
    )
    # Impact instant: pass through with commanded strike speed (curriculum-ramped).
    ee_impact_swing_through = RewTerm(
        func=mdp.ee_impact_swing_through_hit_exp,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "pos_std": 0.10,
            "speed_std": 0.6,
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
    racket_speed_idle = RewTerm(
        func=mdp.racket_speed_penalty_outside_swing_window,
        weight=-0.08,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "command_name": "intercept",
            "distance_threshold": 0.20,
            "approach_window_s": 0.55,
        },
    )

<<<<<<< HEAD
    # Contact instant: reward only on the shuttle-arrival pulse (rings at min size).
    timed_intercept = RewTerm(
        func=mdp.timed_intercept_proximity,
        weight=2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "std": 0.10,
            "command_name": "intercept",
        },
    )
    timed_hit = RewTerm(
        func=mdp.timed_hit_bonus,
        weight=6.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "hit_radius": 0.08,
            "command_name": "intercept",
        },
    )
    # Swing at contact — from articulation body velocity (no contact/force sensor).
    timed_swing_speed = RewTerm(
        func=mdp.timed_swing_speed,
        weight=3.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "speed_std": 1.5,
            "command_name": "intercept",
        },
    )
    timed_swing_through = RewTerm(
        func=mdp.timed_swing_through_target,
        weight=4.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=DEFAULT_RACKET_BODY_NAMES),
            "speed_std": 1.5,
            "hit_radius": 0.08,
            "command_name": "intercept",
        },
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.03)
>>>>>>> 97ddcbcd (rl-badminton)
=======
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
>>>>>>> bfee0731 (improve-badminton-rl)
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
    ee_impact_swing_through = CurrTerm(
        func=mdp.ramp_reward_weight,
        params={
            "term_name": "ee_impact_swing_through",
            "start_weight": 0.0,
            "end_weight": 12.0,
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
<<<<<<< HEAD
<<<<<<< HEAD
        params={"term_name": "action_rate", "weight": -0.08, "num_steps": 25000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.02, "num_steps": 25000},
=======
        params={"term_name": "action_rate", "weight": -0.05, "num_steps": 15000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.15, "num_steps": 15000},
>>>>>>> 97ddcbcd (rl-badminton)
=======
        params={"term_name": "action_rate", "weight": -0.08, "num_steps": 15000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.02, "num_steps": 15000},
>>>>>>> bfee0731 (improve-badminton-rl)
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
