from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_humanoid_v1 import WATO_HUMANOID_V1_CFG

from ... import mdp
from ...locomotion_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg


WATO_BASE_BODY = "base"
WATO_FOOT_BODIES = ["Foot_L", "Foot_R"]
WATO_LEG_JOINTS = ["Hip_.*", "Knee_.*", "Ankle_.*"]
WATO_ANKLE_JOINTS = ["Ankle_R_.*", "Ankle_P_.*"]


@configclass
class WatoHumanoidRewards(RewardsCfg):
    """Reward terms for the Wato Humanoid V1 locomotion MDP."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=2.0, params={"command_name": "base_velocity", "std": 0.5}
    )
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=WATO_FOOT_BODIES),
            "threshold": 0.4,
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=WATO_FOOT_BODIES),
            "asset_cfg": SceneEntityCfg("robot", body_names=WATO_FOOT_BODIES),
        },
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WATO_ANKLE_JOINTS)},
    )
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["Hip_A_.*", "Hip_R_.*"])},
    )


@configclass
class WatoHumanoidRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: WatoHumanoidRewards = WatoHumanoidRewards()

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = WATO_HUMANOID_V1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = f"{{ENV_REGEX_NS}}/Robot/{WATO_BASE_BODY}"

        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = [WATO_BASE_BODY]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        self.rewards.lin_vel_z_l2.weight = 0.0
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.dof_acc_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=WATO_LEG_JOINTS)
        self.rewards.dof_torques_l2.weight = -1.5e-7
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=WATO_LEG_JOINTS)

        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)

        self.terminations.base_contact.params["sensor_cfg"].body_names = [WATO_BASE_BODY]


@configclass
class WatoHumanoidRoughEnvCfg_PLAY(WatoHumanoidRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
