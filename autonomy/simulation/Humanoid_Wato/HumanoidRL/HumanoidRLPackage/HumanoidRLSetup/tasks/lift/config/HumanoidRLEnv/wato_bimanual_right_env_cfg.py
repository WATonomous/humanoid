import math

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm, RewardTermCfg as RewTerm, SceneEntityCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_bimanual import (
    BIMANUAL_ARM_CFG,
    RIGHT_ARM_JOINTS,
    RIGHT_CONTROLLED_JOINTS,
    RIGHT_EE_BODY,
    RIGHT_FINGER_DISTAL_TIP_LOCAL,
    RIGHT_FINGER_TIP_BODIES,
    RIGHT_GRIPPER_CLOSED,
    RIGHT_GRIPPER_JOINTS,
    RIGHT_GRIPPER_OPEN,
)
from HumanoidRLPackage.HumanoidRLSetup.tasks.lift import mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.lift.lift_env_cfg import LiftEnvCfg

_OBJECT_CUBE_SIZE = (0.045, 0.045, 0.045)
_OBJECT_START_POS = (0.27, 0.22, 0.05)
_GRASP_TARGET_WIDTH = 0.045
_GRASP_WIDTH_STD = 0.020

# Approximate TCP at the midpoint between the two right gripper joint origins in link6 frame.
_RIGHT_TCP_OFFSET_POS = (0.0345, 0.10361, 0.004349)
_RIGHT_TCP_OFFSET_ROT = (1.0, 0.0, 0.0, 0.0)


def _right_ee_frame_cfg(*, debug_vis: bool) -> FrameTransformerCfg:
    marker_cfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/FrameTransformer/right_ee_tcp")
    marker_cfg.markers["frame"].scale = (0.05, 0.05, 0.05)
    return FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        debug_vis=debug_vis,
        visualizer_cfg=marker_cfg,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/link6",
                name="right_ee_tcp",
                offset=OffsetCfg(pos=_RIGHT_TCP_OFFSET_POS, rot=_RIGHT_TCP_OFFSET_ROT),
            ),
        ],
    )


@configclass
class WatoBimanualRightLiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the WATO bimanual robot, controlling only the right arm."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.replicate_physics = False
        self.scene.robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.0)
        self.scene.robot.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        self.scene.robot.spawn.activate_contact_sensors = True

        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=_OBJECT_START_POS, rot=[1, 0, 0, 0]),
            spawn=sim_utils.CuboidCfg(
                size=_OBJECT_CUBE_SIZE,
                mass_props=sim_utils.MassPropertiesCfg(mass=0.025),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0025, rest_offset=0.0),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=4.0,
                    dynamic_friction=3.0,
                    restitution=0.0,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.82, 0.1)),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=24,
                    solver_velocity_iteration_count=4,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=1.5,
                    disable_gravity=False,
                ),
            ),
        )

        self.scene.ee_frame = _right_ee_frame_cfg(debug_vis=False)
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*",
            force_threshold=0.05,
            history_length=0,
            debug_vis=False,
        )

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=RIGHT_ARM_JOINTS,
            scale=0.30,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=RIGHT_GRIPPER_JOINTS,
            open_command_expr=RIGHT_GRIPPER_OPEN,
            close_command_expr=RIGHT_GRIPPER_CLOSED,
        )

        self.commands.object_pose.body_name = RIGHT_EE_BODY
        self.commands.object_pose.ranges.pos_x = (0.24, 0.36)
        self.commands.object_pose.ranges.pos_y = (0.18, 0.28)
        self.commands.object_pose.ranges.pos_z = (0.10, 0.18)
        self.commands.object_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        right_joint_cfg = SceneEntityCfg("robot", joint_names=RIGHT_CONTROLLED_JOINTS)
        gripper_joint_cfg = SceneEntityCfg("robot", joint_names=RIGHT_GRIPPER_JOINTS)

        fingertip_cfg = SceneEntityCfg("robot", body_names=list(RIGHT_FINGER_TIP_BODIES))
        contact_sensor_cfg = SceneEntityCfg("contact_forces", body_names=list(RIGHT_FINGER_TIP_BODIES))
        fingertip_offsets = [RIGHT_FINGER_DISTAL_TIP_LOCAL[name] for name in RIGHT_FINGER_TIP_BODIES]

        self.observations.policy.joint_pos.params = {"asset_cfg": right_joint_cfg}
        self.observations.policy.joint_vel.params = {"asset_cfg": right_joint_cfg}
        self.observations.policy.object_position = ObsTerm(
            func=mdp.fingertip_center_to_object,
            params={"fingertip_cfg": fingertip_cfg, "tip_local_offsets": fingertip_offsets},
        )
        self.observations.policy.actions = ObsTerm(
            func=mdp.grasp_geometry,
            params={
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
                "gripper_cfg": gripper_joint_cfg,
                "open_width": 0.10,
            },
        )
        self.rewards.joint_vel.params["asset_cfg"] = right_joint_cfg

        self.rewards.lifting_object.weight = 40.0
        self.rewards.lifting_object.params["minimal_height"] = 0.058
        self.rewards.object_height = RewTerm(
            func=mdp.object_height_above_initial,
            params={"height_start": 0.05, "height_target": 0.10},
            weight=0.0,
        )
        self.rewards.object_goal_tracking.weight = 0.0
        self.rewards.object_goal_tracking.params["minimal_height"] = 0.065
        self.rewards.object_goal_tracking_fine_grained.weight = 0.0
        self.rewards.object_goal_tracking_fine_grained.params["minimal_height"] = 0.065
        self.rewards.gripper_close_near_object = RewTerm(
            func=mdp.gripper_closed_when_near_object,
            params={"threshold": 0.13, "open_width": 0.10, "asset_cfg": gripper_joint_cfg},
            weight=2.0,
        )
        self.rewards.close_action_near_object = RewTerm(
            func=mdp.close_action_near_object,
            params={"threshold": 0.09, "action_name": "gripper_action"},
            weight=2.0,
        )
        self.rewards.object_between_fingertips = RewTerm(
            func=mdp.object_between_fingertips,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=8.0,
        )
        self.rewards.ee_above_object = RewTerm(
            func=mdp.ee_above_object,
            params={"xy_std": 0.08, "z_offset": 0.035, "z_std": 0.035},
            weight=2.0,
        )
        self.rewards.fingertip_center_on_object = RewTerm(
            func=mdp.fingertip_center_on_object,
            params={
                "xy_std": 0.11,
                "z_offset": 0.0,
                "z_std": 0.08,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=10.0,
        )
        self.rewards.close_action_when_centered = RewTerm(
            func=mdp.close_action_when_centered,
            params={
                "centered_threshold": 0.02,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "action_name": "gripper_action",
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=12.0,
        )
        self.rewards.close_action_when_grasp_ready = RewTerm(
            func=mdp.close_action_when_pinch_ready,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "xy_std": 0.11,
                "z_offset": 0.0,
                "z_std": 0.08,
                "action_name": "gripper_action",
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=35.0,
        )
        self.rewards.open_action_when_grasp_ready = RewTerm(
            func=mdp.open_action_when_pinch_ready,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "xy_std": 0.11,
                "z_offset": 0.0,
                "z_std": 0.08,
                "action_name": "gripper_action",
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=-25.0,
        )
        self.rewards.close_action_when_not_grasp_ready = RewTerm(
            func=mdp.close_action_when_not_pinch_ready,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "xy_std": 0.11,
                "z_offset": 0.0,
                "z_std": 0.08,
                "action_name": "gripper_action",
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=-20.0,
        )
        self.rewards.gripper_close_on_object = RewTerm(
            func=mdp.gripper_closed_on_object,
            params={
                "center_std": 0.065,
                "perpendicular_std": 0.045,
                "open_width": 0.10,
                "target_width": _GRASP_TARGET_WIDTH,
                "width_std": _GRASP_WIDTH_STD,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=90.0,
        )
        self.rewards.gripper_width_on_object = RewTerm(
            func=mdp.gripper_width_on_object,
            params={
                "target_width": _GRASP_TARGET_WIDTH,
                "width_std": _GRASP_WIDTH_STD,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=250.0,
        )
        self.rewards.two_finger_contact_on_object = RewTerm(
            func=mdp.two_finger_contact_on_object,
            params={
                "contact_force_std": 0.25,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "sensor_cfg": contact_sensor_cfg,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=1000.0,
        )
        self.rewards.gripper_open_when_not_centered = RewTerm(
            func=mdp.gripper_open_when_not_centered,
            params={
                "centered_threshold": 0.35,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=2.0,
        )
        self.rewards.object_height_when_grasped = RewTerm(
            func=mdp.object_height_when_grasped,
            params={
                "height_start": 0.05,
                "height_target": 0.10,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=300.0,
        )
        self.rewards.object_height_when_contact_grasped = RewTerm(
            func=mdp.object_height_when_contact_grasped,
            params={
                "height_start": 0.05,
                "height_target": 0.10,
                "contact_force_std": 0.25,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "sensor_cfg": contact_sensor_cfg,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=12000.0,
        )
        self.rewards.cube_follows_gripper_when_grasped = RewTerm(
            func=mdp.cube_follows_gripper_when_grasped,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "height_start": 0.05,
                "height_target": 0.10,
                "vertical_offset": 0.04,
                "vertical_std": 0.04,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=300.0,
        )
        self.rewards.object_height_without_grasp = RewTerm(
            func=mdp.object_height_without_grasp,
            params={
                "height_start": 0.05,
                "height_target": 0.10,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=-250.0,
        )
        self.rewards.object_xy_without_grasp = RewTerm(
            func=mdp.object_xy_displacement_without_grasp,
            params={
                "start_pos": (_OBJECT_START_POS[0], _OBJECT_START_POS[1]),
                "max_distance": 0.06,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=-50.0,
        )
        self.rewards.object_velocity_without_grasp = RewTerm(
            func=mdp.object_velocity_without_grasp,
            params={
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=-2.0,
        )
        self.rewards.object_goal_when_grasped = RewTerm(
            func=mdp.object_goal_distance_when_grasped,
            params={
                "std": 0.16,
                "minimal_height": 0.065,
                "command_name": "object_pose",
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=0.0,
        )
        self.rewards.gripper_open_far_object = RewTerm(
            func=mdp.gripper_open_when_far_object,
            params={"threshold": 0.10, "open_width": 0.10, "asset_cfg": gripper_joint_cfg},
            weight=2.0,
        )
        self.rewards.ee_lift_closed_near_object = RewTerm(
            func=mdp.fingertip_up_velocity_when_contact_grasped,
            params={
                "speed_target": 0.08,
                "contact_force_std": 0.25,
                "center_std": 0.075,
                "perpendicular_std": 0.05,
                "open_width": 0.10,
                "sensor_cfg": contact_sensor_cfg,
                "gripper_cfg": gripper_joint_cfg,
                "fingertip_cfg": fingertip_cfg,
                "tip_local_offsets": fingertip_offsets,
            },
            weight=800.0,
        )
        self.rewards.action_rate.weight = -1e-5
        self.rewards.joint_vel.weight = -1e-5
        self.curriculum.action_rate.params["weight"] = -1e-4
        self.curriculum.joint_vel.params["weight"] = -1e-4
        self.scene.contact_forces.update_period = self.sim.dt

        self.events.reset_cube_position.params["pose_range"] = {
            "x": (-0.005, 0.005),
            "y": (-0.005, 0.005),
            "z": (0.0, 0.0),
        }
        self.terminations.object_xy_out_of_bounds = DoneTerm(
            func=mdp.object_xy_out_of_bounds,
            params={
                "x_bounds": (-0.10, 0.70),
                "y_bounds": (-0.20, 0.65),
                "object_cfg": SceneEntityCfg("cube"),
            },
        )
        self.events.object_physics_material = EventTerm(
            func=mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("cube"),
                "static_friction_range": (4.0, 4.0),
                "dynamic_friction_range": (3.0, 3.0),
                "restitution_range": (0.0, 0.0),
                "num_buckets": 1,
            },
        )
        self.events.robot_physics_material = EventTerm(
            func=mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "static_friction_range": (4.0, 4.0),
                "dynamic_friction_range": (3.0, 3.0),
                "restitution_range": (0.0, 0.0),
                "num_buckets": 1,
            },
        )


@configclass
class WatoBimanualRightLiftEnvCfg_PLAY(WatoBimanualRightLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.commands.object_pose.debug_vis = True
        self.scene.ee_frame = _right_ee_frame_cfg(debug_vis=True)
