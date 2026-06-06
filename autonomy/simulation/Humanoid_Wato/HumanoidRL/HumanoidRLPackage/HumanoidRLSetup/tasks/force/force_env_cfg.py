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
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import HumanoidRLPackage.HumanoidRLSetup.tasks.force.mdp as mdp
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid_arm_hand import ARM_FORCE_CFG

_CONTACT_BODIES = mdp.DEFAULT_CONTACT_BODY_NAMES


@configclass
class ForceSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    robot = ARM_FORCE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    push_wall = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PushWall",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.58, 0.0, 0.15)),
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 1.4, 1.2),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.65, 0.65, 0.7)),
        ),
    )

    # arm.usd nests all links under Robot/arm_assembly/* (depth 2). The default
    # Robot/.* pattern only matches one level and finds arm_assembly (no reporter).
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*/.*",
        force_threshold=0.1,
        history_length=0,
        debug_vis=False,
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


@configclass
class CommandsCfg:
    ee_impulse = mdp.UniformImpulseForceCommandCfg(
        asset_name="robot",
        contact_sensor_name="contact_forces",
        resampling_time_range=(4.0, 4.0),
        impulse_duration_s=0.5,
        debug_vis=True,
        ranges=mdp.UniformImpulseForceCommandCfg.Ranges(
            pos_x=(-0.72, -0.62),
            pos_y=(-0.35, 0.35),
            pos_z=(0.0, 0.35),
            force_x=(1.0, 5.0),
            force_y=(-1.5, 1.5),
            force_z=(-1.5, 1.5),
        ),
    )


@configclass
class ActionsCfg:
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        impulse_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_impulse"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.75, 1.25),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    region_proximity = RewTerm(
        func=mdp.region_proximity_tanh,
        weight=0.6,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=_CONTACT_BODIES),
            "std": 0.12,
            "command_name": "ee_impulse",
        },
    )
    contact_force_tracking = RewTerm(
        func=mdp.multi_link_contact_force_tracking,
        weight=2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=_CONTACT_BODIES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=_CONTACT_BODIES),
            "std": 3.0,
            "command_name": "ee_impulse",
            "region_radius": 0.12,
            "contact_threshold": 0.5,
        },
    )
    contact_force_penalty = RewTerm(
        func=mdp.multi_link_contact_force_error,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=_CONTACT_BODIES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=_CONTACT_BODIES),
            "command_name": "ee_impulse",
            "region_radius": 0.12,
            "contact_threshold": 0.5,
        },
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.03)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot")},
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
class ForceEnvCfg(ManagerBasedRLEnvCfg):
    scene: ForceSceneCfg = ForceSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        # Ensure spawn flag survives Hydra cfg round-trips.
        self.scene.robot.spawn.activate_contact_sensors = True

        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        self.sim.dt = 1.0 / 60.0
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625
        self.scene.contact_forces.update_period = self.sim.dt
