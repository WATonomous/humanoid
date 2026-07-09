"""Isaac Lab env for generalized pick-and-place with the wato_bimanual_arm left arm.

Joint-space action env (cuRobo expert streams joint targets); the right arm
has no action term and holds its default pose via the implicit actuators.
Scene geometry (table slab, workspace) matches the cuRobo world model in
pick_place_gen/ exactly.

Configure via pick_place_gen/task_params.py: build with
    env_cfg = make_env_cfg(PickPlaceTaskParams.from_yaml(path))
    env = gym.make("Isaac-PickPlace-BimanualLeft-v0", cfg=env_cfg)
"""
import math

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from . import mdp
from .robot_cfg_shim import (
    BIMANUAL_ARM_CFG,
    LEFT_JOINTS_ALL,
    PickPlaceTaskParams,
    wato_constants as wc,
)

_TABLE_CENTER = (
    wc.TABLE_X_MIN + wc.TABLE_DIMS[0] / 2,
    -0.10,
    wc.TABLE_TOP_Z - wc.TABLE_DIMS[2] / 2,
)


def _cuboid_object_cfg(prim_name: str, size, mass, color, init_pos) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/" + prim_name,
        init_state=RigidObjectCfg.InitialStateCfg(pos=init_pos),
        spawn=sim_utils.CuboidCfg(
            size=tuple(size),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=mass),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.9, dynamic_friction=0.8, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color)),
        ),
    )


def _look_at_quat_ros(eye, target, up=(0.0, 0.0, 1.0)) -> tuple:
    """Camera quaternion (wxyz, ROS optical convention: +Z forward, +Y down)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    up = np.asarray(up, float)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.stack([x, y, z], axis=1)
    w = math.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2.0
    return (
        w,
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
    )


@configclass
class PickPlaceSceneCfg(InteractiveSceneCfg):
    # NOTE: all fields annotated so @configclass keeps declaration order —
    # InteractiveScene creates entities in this order and the cameras must
    # spawn after the robot (the wrist cam nests under Robot/link6l).
    robot: ArticulationCfg = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_CENTER),
        spawn=sim_utils.CuboidCfg(
            size=wc.TABLE_DIMS,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.9, dynamic_friction=0.8, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.35, 0.25)),
        ),
    )

    plane: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
        spawn=sim_utils.GroundPlaneCfg(),
    )

    light: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # Filled by make_env_cfg from PickPlaceTaskParams:
    object: RigidObjectCfg = None
    place_object: RigidObjectCfg = None
    camera_external: TiledCameraCfg = None
    camera_wrist: TiledCameraCfg = None


@configclass
class ActionsCfg:
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=LEFT_JOINTS_ALL,
        scale=1.0,
        use_default_offset=False,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEFT_JOINTS_ALL)},
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEFT_JOINTS_ALL)},
        )
        object_pose = ObsTerm(func=mdp.object_pose_in_robot_root_frame)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    # params filled by make_env_cfg (object list depends on place mode)
    reset_objects = EventTerm(func=mdp.reset_objects_min_separation, mode="reset", params={})
    object_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("object", body_names=".*"),
            "static_friction_range": (0.6, 1.2),
            "dynamic_friction_range": (0.5, 1.1),
            "restitution_range": (0.0, 0.1),
            "num_buckets": 32,
        },
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={"minimum_height": wc.TABLE_TOP_Z - 0.10, "asset_cfg": SceneEntityCfg("object")},
    )


@configclass
class RewardsCfg:
    """IL data generation only — no reward shaping."""
    pass


@configclass
class PickPlaceBimanualEnvCfg(ManagerBasedRLEnvCfg):
    # replicate_physics=True: with False, child prims of the referenced robot
    # USD are not yet composed when nested sensors (wrist cam) spawn.
    scene: PickPlaceSceneCfg = PickPlaceSceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        apply_task_params(self, PickPlaceTaskParams())


def apply_task_params(cfg: "PickPlaceBimanualEnvCfg", params: PickPlaceTaskParams) -> None:
    """Project a PickPlaceTaskParams onto the env cfg (idempotent)."""
    obj = params.object
    obj_rest_z = wc.TABLE_TOP_Z + obj.size[2] / 2 + 0.002
    cfg.scene.object = _cuboid_object_cfg(
        "Object", obj.size, obj.mass, obj.color,
        ((obj.x_range[0] + obj.x_range[1]) / 2, (obj.y_range[0] + obj.y_range[1]) / 2, obj_rest_z),
    )

    reset_assets = [SceneEntityCfg("object")]
    z_values = [obj_rest_z]
    if params.place.mode == "stack":
        pl = params.place
        stack_rest_z = wc.TABLE_TOP_Z + pl.stack_object_size[2] / 2 + 0.002
        cfg.scene.place_object = _cuboid_object_cfg(
            "PlaceObject", pl.stack_object_size, pl.stack_object_mass, pl.stack_object_color,
            (0.36, -0.14, stack_rest_z),
        )
        reset_assets.append(SceneEntityCfg("place_object"))
        z_values.append(stack_rest_z)
    else:
        cfg.scene.place_object = None

    cfg.events.reset_objects.params = {
        "x_range": obj.x_range,
        "y_range": obj.y_range,
        "yaw_range": obj.yaw_range,
        "z_values": z_values,
        "min_separation": params.place.min_separation,
        "asset_cfgs": reset_assets,
    }
    if params.noise.enabled:
        fr = params.noise.friction_range
        cfg.events.object_material.params.update({
            "static_friction_range": fr,
            "dynamic_friction_range": (max(fr[0] - 0.1, 0.05), max(fr[1] - 0.1, 0.1)),
            "restitution_range": params.noise.restitution_range,
        })
    else:
        cfg.events.object_material = None

    cam = params.cameras
    if cam.enabled and cam.external:
        eye = (1.05, -0.85, 0.75)
        target = ((obj.x_range[0] + obj.x_range[1]) / 2, (obj.y_range[0] + obj.y_range[1]) / 2, wc.TABLE_TOP_Z)
        cfg.scene.camera_external = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/external_cam",
            offset=TiledCameraCfg.OffsetCfg(pos=eye, rot=_look_at_quat_ros(eye, target), convention="ros"),
            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 6.0)),
            width=cam.width, height=cam.height,
            data_types=["rgb"],
            update_period=0.0,
        )
    else:
        cfg.scene.camera_external = None
    if cam.enabled and cam.wrist:
        # perched laterally off the gripper rail (wrist +Z), looking at the
        # fingertip center; clears the rail (|z|<0.05) and forearm (+Y side)
        cam_pos = (0.0345, -0.06, -0.22)
        tip_ref = np.array(wc.FINGERTIP_OFFSET_IN_WRIST)
        cfg.scene.camera_wrist = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/link6l/wrist_cam",
            offset=TiledCameraCfg.OffsetCfg(
                pos=cam_pos,
                # up hint = wrist +Y (world-up during top-down grasps)
                rot=_look_at_quat_ros(cam_pos, tip_ref, up=(0.0, -1.0, 0.0)),
                convention="ros",
            ),
            spawn=sim_utils.PinholeCameraCfg(focal_length=16.0, clipping_range=(0.02, 4.0)),
            width=cam.width, height=cam.height,
            data_types=["rgb"],
            update_period=0.0,
        )
    else:
        cfg.scene.camera_wrist = None

    # Grippers default to OPEN=(0,0): the shared cfg's (-0.05, +0.05) default
    # CROSSES the finger meshes (STL-verified; labels inverted upstream).
    for j in ("joint7", "joint8", "joint7l", "joint8l"):
        cfg.scene.robot.init_state.joint_pos[j] = 0.0

    # PhysX self-collisions OFF for this task: the exported finger-rail
    # collision meshes interpenetrate by design (even fully open), producing
    # constant internal contact forces that saturate the shoulder (measured
    # 53 Nm applied in free space). Path-level self-collision safety is
    # guaranteed by the cuRobo planner instead.
    cfg.scene.robot.spawn.articulation_props.enabled_self_collisions = False

    # Actuator effort overrides (see EpisodeParams for rationale/flags).
    ep_lim = params.episode
    cfg.scene.robot.actuators["left_shoulder"].effort_limit_sim = ep_lim.shoulder_effort_limit
    cfg.scene.robot.actuators["left_elbow"].effort_limit_sim = ep_lim.elbow_effort_limit
    cfg.scene.robot.actuators["left_wrist"].effort_limit_sim = ep_lim.wrist_effort_limit
    cfg.scene.robot.actuators["left_gripper"].effort_limit_sim = ep_lim.gripper_effort_limit

    ep = params.episode
    cfg.decimation = ep.decimation
    cfg.episode_length_s = ep.episode_length_s
    cfg.sim.dt = ep.sim_dt
    cfg.sim.render_interval = ep.decimation
    cfg.sim.render.rendering_mode = "quality"
    cfg.sim.physx.bounce_threshold_velocity = 0.2
    cfg.sim.physx.friction_correlation_distance = 0.00625


def make_env_cfg(params: PickPlaceTaskParams) -> PickPlaceBimanualEnvCfg:
    cfg = PickPlaceBimanualEnvCfg()
    apply_task_params(cfg, params)
    return cfg
