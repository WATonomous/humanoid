from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from ... import mdp
from ...locomotion_env_cfg import TerminationsCfg
from .rough_env_cfg import WATO_FOOT_BODIES, WATO_LEG_JOINTS, WatoHumanoidRewards, WatoHumanoidRoughEnvCfg


@configclass
class WatoHumanoidFlatRewards(WatoHumanoidRewards):
    """Kept close to official Isaac Lab G1 flat: no gait-phase clock, no swing-direction
    shaping, no body-spacing constraints. Those custom terms let the policy satisfy reward
    with a stepping *pattern* that produced no net forward displacement (verified: episodes
    with strong phase_* rewards but forward_velocity stuck at exactly 0.0 for 400+ iterations).
    G1's reward set relies on track_lin_vel_xy_exp as the dominant, ungated signal instead."""

    # G1's actual config uses only feet_air_time_positive_biped (confirmed by reading
    # Isaac Lab's source directly) and apparently escapes double-support via exploration
    # alone -- no equivalent term exists there. But ankle_60nm_extended (990/1000 episode
    # length, healthy training) settled into a static double-support lunge stance with
    # feet_air_time reward stuck at exactly 0 for the entire run: feet_air_time_positive_biped
    # only pays out once single_stance is already true, so it can't provide any gradient
    # toward the first exploratory foot-lift out of permanent double support. This is a
    # small, targeted addition (not a return to the earlier gait-phase system) just to
    # break that specific, repeatedly-observed local optimum.
    #
    # Starts at weight 0.0 -- double_support_fresh_001 (this penalty active from
    # iteration 0, weight -0.3) plateaued hard at episode length ~48-51 for 500+
    # iterations, well behind the no-penalty run's pace at the same point (60-75).
    # Fighting "learn to balance at all" and "stop double-supporting" simultaneously
    # from a random init was too much at once. A curriculum term (below) activates the
    # penalty only once basic stability is established.
    double_support_penalty = RewTerm(
        func=mdp.double_support_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=WATO_FOOT_BODIES),
        },
    )
    # -0.3 (curriculum target below) was too weak: double_support_curriculum_001 finished
    # at episode length 932.8, track_lin_vel_xy_exp 0.8865, but double_support_penalty
    # only -0.2072 (~69% double-support time) and feet_air_time still ~0.0002. Once
    # tracking reward approaches its ~0.9-1.0 ceiling, a max -0.3 penalty is negligible
    # by comparison (weight=1.0 on tracking vs an effective ceiling of 0.3 on this) --
    # raised substantially so it can actually compete.


@configclass
class WatoHumanoidFlatTerminations(TerminationsCfg):
    # Reverted: LEGR-matched base_tilt/low_base thresholds caused a consistent
    # collapse to ~17-18 step episodes across 3 different action-scale variants,
    # ruling out action scale as the cause and implicating these thresholds instead.
    base_tilt = None
    low_base = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.30})


@configclass
class WatoHumanoidFlatEnvCfg(WatoHumanoidRoughEnvCfg):
    rewards: WatoHumanoidFlatRewards = WatoHumanoidFlatRewards()
    terminations: WatoHumanoidFlatTerminations = WatoHumanoidFlatTerminations()

    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None

        self.actions.joint_pos.scale = {
            # Reverted Hip_F 0.40 -> 0.25: raising it in isolation did not change the
            # failure mode either (video confirmed still a rigid forward topple, legs
            # locked together, no stepping) -- ruled out action scale as the bottleneck.
            "Hip_F_.*": 0.25,
            "Knee_.*": 0.30,
            "Hip_A_.*": 0.06,
            "Hip_R_.*": 0.06,
            "Ankle_R_.*": 0.08,
            "Ankle_P_.*": 0.12,
        }

        # Reverted: tightening reset randomization (base velocity +-0.1, joint
        # position_range 0.9-1.1) converged to the identical ~40-42 step ceiling as
        # full-strength randomization, and the rollout video showed the same rigid
        # forward-topple failure mode -- ruled out reset perturbation as the bottleneck.

        # joint_vel observation noise is inherited from the base ObservationsCfg at a
        # flat +-1.5 rad/s (copied from Isaac Lab's G1-scale default). Hip_F/Knee here
        # have velocity_limit_sim=3.6652 rad/s (see wato_humanoid_v1.py) -- that noise
        # is ~41% of their full range, versus a small fraction for G1's much
        # faster-rated joints. diagnose_spawn.py proved zero-action survives 60+ steps
        # under full physical randomization while every trained policy (even near-zero
        # noise/entropy) still falls by ~35-40 steps -- the fall is coming from the
        # policy's closed-loop reaction to something, and corrupted joint velocity
        # proprioception on the slowest, most balance-critical joints is the leading
        # remaining candidate. Scaled down to a value proportionate to Wato's actual
        # actuator speeds instead of G1's.
        self.observations.policy.joint_vel.noise = Unoise(n_min=-0.3, n_max=0.3)

        # Matched to official Isaac Lab G1 flat: track_lin_vel_xy_exp (inherited from rough,
        # weight=1.0, std=0.5) stays the dominant, ungated reward -- only retune a handful of
        # weights + command range, same pattern as G1FlatEnvCfg.
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.lin_vel_z_l2.weight = -0.2
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.0e-7
        # Raised from 0.75, threshold lowered from 0.4: ankle_stiffness_extended_2000's
        # healthy checkpoints (iter 1000-1500) reached full 1000-step episodes but with
        # feet_air_time reward stuck at exactly 0.0 -- the robot found a static brace
        # pose that already scores ~0.92/1.0 on track_lin_vel_xy_exp (std=0.5 is
        # forgiving enough that standing-still error looks "close enough"), so there
        # was no marginal incentive to risk stepping. Lower threshold makes even a
        # brief foot-lift count; higher weight makes it worth the risk.
        self.rewards.feet_air_time.weight = 2.0
        self.rewards.feet_air_time.params["threshold"] = 0.2
        self.rewards.dof_torques_l2.weight = -2.0e-6
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=WATO_LEG_JOINTS)
        self.rewards.termination_penalty.weight = -100.0
        # Reverted to the original std=0.5: both 0.25 and 0.4 overcorrected and
        # produced too weak a gradient (stepping_fix_fresh_002/003 both plateaued
        # around episode length 45-47, nowhere near the 990/1000 full-episode
        # stability the original std=0.5 config actually reached by iteration 1000).
        # The real fix for "stuck standing" is entropy_coef/empirical_normalization
        # (prevents the divergence that corrupted that run) plus the higher
        # feet_air_time weight below -- not a tighter tracking tolerance.

        # Reverted from LEGR-matched (0.12, 0.22) forced-forward range with
        # rel_standing_envs=0.0: every rollout so far (3 different reward/action/reset
        # configs, all visually confirmed) shows the robot toppling forward rigidly
        # within ~0.8s regardless of tuning. A command range with no zero/standing
        # option means every env's reward gradient always pushes for forward motion
        # from step 0, before the policy has any chance to learn balance -- G1 never
        # does this (rel_standing_envs=0.02, lin_vel_x includes 0.0). Testing whether
        # letting some envs command "stand still" removes the incentive to lean
        # forward before the policy can support it.
        # Reverted lin_vel_x to (0.0, 0.22): narrowing it to (0.15, 0.28) was part of
        # the same overcorrection as the tighter std -- combined with std=0.25/0.4 it
        # produced a stalled gradient (episode length plateaued ~45-47, nowhere near
        # the 990/1000 stability the original range actually reached). rel_standing_envs
        # stays at 0.1 (up from 0.0) since that part was a legitimate fix on its own.
        # widen_velocity_001 confirmed walking held up (even improved -- feet_air_time
        # nearly doubled) after widening lin_vel_x to (0.0, 0.45). Now: (1) made
        # symmetric so it also learns walking backward, not just forward, and (2)
        # added lateral (lin_vel_y) and turning (ang_vel_z) for the first time --
        # previously zeroed all session as a deliberate scope reduction while chasing
        # basic forward stepping. Kept lateral/turning ranges modest since they're
        # brand new to the policy; can widen further once confirmed stable.
        # Jumped straight to G1's own flat-config scale (lin_vel_x (0.0,1.0), lin_vel_y
        # (-0.5,0.5), ang_vel_z (-1.0,1.0)) rather than incrementally widening again --
        # higher_velocity_001's live-observed checkpoint confirmed walking held up fine
        # at the previous (-0.6,0.6)/(-0.25,0.25)/(-0.5,0.5) range, so no reason to be
        # more conservative than the reference config at this point.
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.1
        self.commands.base_velocity.rel_heading_envs = 0.0
        self.commands.base_velocity.ranges.lin_vel_x = (-1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = None
        self.events.reset_base.params["pose_range"] = {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (0.0, 0.0)}

        # Activates double_support_penalty (weight 0.0 -> -2.0) once basic stability is
        # established, instead of fighting balance-learning and anti-double-support
        # pressure simultaneously from a random init. ankle_60nm_extended reached full
        # 990/1000 episode-length stability around PPO iteration ~2500-3000; at
        # num_steps_per_env=24, that's roughly 2500*24=60000 env steps. Picking 55000
        # (~iteration 2290) to activate slightly before that point.
        # -0.3 was tried first and was too weak (double_support_curriculum_001: finished
        # at ~69% double-support time, feet_air_time still ~0.0002) -- once tracking
        # reward nears its ~0.9-1.0 ceiling, a max -0.3 penalty is negligible by
        # comparison. Raised to -2.0 so it can meaningfully compete.
        self.curriculum.double_support_penalty_activate = CurrTerm(
            func=mdp.modify_reward_weight,
            params={"term_name": "double_support_penalty", "weight": -2.0, "num_steps": 55000},
        )


@configclass
class WatoHumanoidFlatEnvCfg_PLAY(WatoHumanoidFlatEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.0
        self.commands.base_velocity.rel_heading_envs = 0.0
        self.commands.base_velocity.ranges.lin_vel_x = (-1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
