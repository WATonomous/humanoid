from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from ... import mdp
from ...locomotion_env_cfg import TerminationsCfg
from .rough_env_cfg import LEGR_FOOT_BODIES, LEGR_LEG_JOINTS, LEGRRewards, LEGRRoughEnvCfg


LEGR_GAIT_PERIOD = 0.9


@configclass
class LEGRFlatRewards(LEGRRewards):
    upright_track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_upright_exp,
        weight=2.0,
        params={"command_name": "base_velocity", "std": 0.10, "tilt_std": 0.22},
    )
    forward_velocity = RewTerm(func=mdp.forward_velocity_reward, weight=2.0, params={"command_name": "base_velocity"})
    world_forward_velocity = RewTerm(
        func=mdp.world_forward_velocity_reward, weight=2.0, params={"command_name": "base_velocity"}
    )
    lateral_velocity_l2 = RewTerm(func=mdp.lateral_velocity_l2, weight=-9.0)
    yaw_rate_l2 = RewTerm(func=mdp.yaw_rate_l2, weight=-1.35)
    heading_yaw_l2 = RewTerm(func=mdp.heading_yaw_l2, weight=-0.75)
    phase_alternating_gait = RewTerm(
        func=mdp.phase_alternating_gait,
        weight=2.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
        },
    )
    phase_contact_mismatch = RewTerm(
        func=mdp.phase_contact_mismatch_penalty,
        weight=-1.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
        },
    )
    phase_swing_foot_forward = RewTerm(
        func=mdp.phase_swing_foot_forward,
        weight=3.3,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "target_step_length": 0.070,
        },
    )
    phase_swing_foot_forward_velocity = RewTerm(
        func=mdp.phase_swing_foot_forward_velocity,
        weight=1.3,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "target_velocity": 0.22,
        },
    )
    phase_swing_foot_behind = RewTerm(
        func=mdp.phase_swing_foot_behind_l2,
        weight=-1.2,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "min_step_length": 0.030,
        },
    )
    phase_swing_foot_inward = RewTerm(
        func=mdp.phase_swing_foot_inward_l2,
        weight=-1.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "min_side_y": 0.09,
        },
    )
    phase_swing_lateral_velocity = RewTerm(
        func=mdp.phase_swing_lateral_velocity_l2,
        weight=-1.2,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "free_velocity": 0.08,
            "target_velocity": 0.25,
        },
    )
    phase_feet_clearance = RewTerm(
        func=mdp.phase_feet_clearance_biped,
        weight=2.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "period": LEGR_GAIT_PERIOD,
            "target_height": 0.035,
        },
    )
    phase_swing_knee_bend = RewTerm(
        func=mdp.phase_swing_knee_bend,
        weight=2.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", joint_names=["Knee_L", "Knee_R"]),
            "period": LEGR_GAIT_PERIOD,
            "target_bend": 0.35,
        },
    )
    foot_lateral_placement = RewTerm(
        func=mdp.foot_lateral_placement_l2,
        weight=-8.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES), "target_width": 0.24},
    )
    feet_too_close = RewTerm(
        func=mdp.feet_too_close_l2,
        weight=-20.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES), "min_width": 0.20},
    )
    thighs_too_close = RewTerm(
        func=mdp.bodies_too_close_l2,
        weight=-8.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["Rotation_L", "Rotation_R"]), "min_width": 0.16},
    )
    thigh_lateral_placement = RewTerm(
        func=mdp.paired_body_lateral_placement_l2,
        weight=-4.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["Rotation_L", "Rotation_R"]), "target_width": 0.18},
    )
    calves_too_close = RewTerm(
        func=mdp.bodies_too_close_l2,
        weight=-12.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["Calf_L", "Calf_R"]), "min_width": 0.16},
    )
    calf_lateral_placement = RewTerm(
        func=mdp.paired_body_lateral_placement_l2,
        weight=-6.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["Calf_L", "Calf_R"]), "target_width": 0.18},
    )
    hip_rotation_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["R_.*"])},
    )
    hip_abduction_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["A_.*"])},
    )
    ankle_roll_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["U_R_.*"])},
    )
    ankle_roll_deviation = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.04,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["U_R_.*"])},
    )
    left_ankle_roll_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.006,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["U_R_L"])},
    )
    left_ankle_roll_deviation = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.015,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["U_R_L"])},
    )
    ankle_pitch_deviation = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["U_P_.*"])},
    )
    feet_clearance = RewTerm(
        func=mdp.feet_clearance_biped,
        weight=1.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=LEGR_FOOT_BODIES),
            "target_height": 0.04,
        },
    )
    double_support = RewTerm(
        func=mdp.double_support_penalty,
        weight=-2.0,
        params={"command_name": "base_velocity", "sensor_cfg": SceneEntityCfg("contact_forces", body_names=LEGR_FOOT_BODIES)},
    )
    base_height_l2 = RewTerm(func=mdp.base_height_l2, weight=-2.0, params={"target_height": 0.80})


@configclass
class LEGRFlatTerminations(TerminationsCfg):
    base_tilt = DoneTerm(func=mdp.base_tilt_over_limit, params={"limit": 0.28})
    low_base = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.62})


@configclass
class LEGRFlatEnvCfg(LEGRRoughEnvCfg):
    rewards: LEGRFlatRewards = LEGRFlatRewards()
    terminations: LEGRFlatTerminations = LEGRFlatTerminations()

    def __post_init__(self):
        super().__post_init__()

        # Change terrain to flat.
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.policy.gait_phase = ObsTerm(func=mdp.gait_phase, params={"period": LEGR_GAIT_PERIOD})
        self.curriculum.terrain_levels = None

        # Actions
        self.actions.joint_pos.scale = {
            "F_.*": 0.35,
            "Knee_.*": 0.40,
            "A_.*": 0.09,
            "R_.*": 0.08,
            "U_R_.*": 0.09,
            "U_P_.*": 0.20,
        }

        # Rewards
        self.rewards.track_lin_vel_xy_exp.weight = 0.0
        self.rewards.track_lin_vel_xy_exp.params["std"] = 0.35
        self.rewards.upright_track_lin_vel_xy_exp.params["std"] = 0.10
        self.rewards.forward_velocity.weight = 2.0
        self.rewards.world_forward_velocity.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.1
        self.rewards.track_ang_vel_z_exp.params["std"] = 0.22
        self.rewards.lateral_velocity_l2.weight = -9.0
        self.rewards.yaw_rate_l2.weight = -1.35
        self.rewards.heading_yaw_l2.weight = -0.75
        self.rewards.lin_vel_z_l2.weight = -0.2
        self.rewards.ang_vel_xy_l2.weight = -0.3
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.0e-7
        self.rewards.feet_air_time.weight = 3.0
        self.rewards.feet_air_time.params["threshold"] = 0.08
        self.rewards.flat_orientation_l2.weight = -5.0
        self.rewards.dof_torques_l2.weight = -1.0e-6
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=LEGR_LEG_JOINTS)
        self.rewards.joint_deviation_hip.weight = -0.40
        self.rewards.feet_slide.weight = -0.5
        self.rewards.double_support.weight = -2.0
        self.rewards.termination_penalty.weight = -100.0

        # Commands
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.0
        self.commands.base_velocity.rel_heading_envs = 0.0
        self.commands.base_velocity.ranges.lin_vel_x = (0.12, 0.22)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = None
        self.events.reset_base.params["pose_range"] = {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (0.0, 0.0)}


@configclass
class LEGRFlatEnvCfg_PLAY(LEGRFlatEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.0
        self.commands.base_velocity.rel_heading_envs = 0.0
        self.commands.base_velocity.ranges.lin_vel_x = (0.16, 0.16)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
