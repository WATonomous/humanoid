"""SO101 follower articulation and scene config for Isaac Lab teleop scripts."""

import os

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from isaacsim.core.utils.rotations import euler_angles_to_quat

import vial_task_assets as vta

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", ".."))
_SO101_FOLLOWER_USD = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101", "so101_follower_good.usd")
_SO101_ARM_CAMERA_USD = os.path.join(_REPO_ROOT, "assets", "lerobot", "so101", "so101_arm_camera.usd")

# LeRobot / IL schema joint order (so101_follower_good.usd)
SO101_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

SO101_ARM_JOINT_NAMES = SO101_JOINT_NAMES[:5]
SO101_EE_BODY = "gripper"

# Workshop SO-ARM101-USD.usd joint names (so101_arm_camera.usd)
WORKSHOP_USD_JOINT_NAMES = [
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
]

ROBOT_KINDS = ("follower", "arm_camera")


def robot_joint_names(robot_kind: str) -> list[str]:
    if robot_kind == "arm_camera":
        return list(WORKSHOP_USD_JOINT_NAMES)
    return list(SO101_JOINT_NAMES)


def robot_arm_joint_names(robot_kind: str) -> list[str]:
    return robot_joint_names(robot_kind)[:5]


def robot_gripper_joint_name(robot_kind: str) -> str:
    return "Jaw" if robot_kind == "arm_camera" else "gripper"

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

# Gripper revolute limits from assets/lerobot/so101_new_calib.urdf
GRIPPER_OPEN = -0.174533
GRIPPER_CLOSED = 0.9

SO101_FOLLOWER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_SO101_FOLLOWER_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        },
    ),
    actuators={
        "sts3215-gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
        "sts3215-arm": ImplicitActuatorCfg(
            joint_names_expr=[
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ],
            effort_limit_sim=10,
            velocity_limit_sim=10,
            stiffness=17.8,
            damping=0.60,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


SO101_ARM_CAMERA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_SO101_ARM_CAMERA_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(-0.05, 0.0, 0.0),
        rot=euler_angles_to_quat(np.array([0, 0, 90]), degrees=True),
        joint_pos={
            "Rotation": -0.2736,
            "Pitch": -0.6109,
            "Elbow": -0.0745,
            "Wrist_Pitch": 1.5148,
            "Wrist_Roll": -1.6034,
            "Jaw": -0.1465,
        },
    ),
    actuators={
        "rotation": ImplicitActuatorCfg(
            joint_names_expr=["Rotation"], effort_limit_sim=30, stiffness=55, damping=0.7
        ),
        "pitch": ImplicitActuatorCfg(
            joint_names_expr=["Pitch"], effort_limit_sim=30, stiffness=30, damping=0.8
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["Elbow"], effort_limit_sim=30, stiffness=25, damping=0.7
        ),
        "wrist_pitch": ImplicitActuatorCfg(
            joint_names_expr=["Wrist_Pitch"], effort_limit_sim=30, stiffness=12, damping=0.5
        ),
        "wrist_roll": ImplicitActuatorCfg(
            joint_names_expr=["Wrist_Roll"], effort_limit_sim=30, stiffness=7, damping=0.5
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["Jaw"], effort_limit_sim=30, stiffness=4, damping=0.3
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


def configure_scene_robot(scene_cfg, robot_kind: str):
    """Swap robot articulation (LeRobot follower vs workshop arm w/ visible camera mesh)."""
    if robot_kind == "arm_camera":
        vta.assert_arm_camera_usd()
        scene_cfg.robot = SO101_ARM_CAMERA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    return scene_cfg


def configure_vision_cameras(scene_cfg, robot_kind: str):
    """Ego camera: spawned on gripper (follower) vs embedded gripper_cam mesh (arm_camera)."""
    if not hasattr(scene_cfg, "camera_ego"):
        return scene_cfg
    if robot_kind == "arm_camera":
        scene_cfg.camera_ego.prim_path = "{ENV_REGEX_NS}/Robot/gripper/gripper_cam"
        scene_cfg.camera_ego.spawn = None
    else:
        scene_cfg.camera_ego.prim_path = "{ENV_REGEX_NS}/Robot/gripper"
        scene_cfg.camera_ego.spawn = sim_utils.PinholeCameraCfg(
            projection_type="pinhole",
            f_stop=100.0,
            focal_length=13.5,
            focus_distance=0.05,
        )
    scene_cfg.camera_ego.offset.pos = (-0.005, 0.06, -0.062)
    scene_cfg.camera_ego.offset.rot = euler_angles_to_quat(np.array([-45, 0, 0]), degrees=True)
    return scene_cfg


@configclass
class SO101SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def _vial_spawn_cfg() -> sim_utils.UsdFileCfg:
    return sim_utils.UsdFileCfg(
        usd_path=vta.VIAL_OPAQUE_USDA,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(angular_damping=100.0),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
    )


@configclass
class SO101VialTaskSceneCfg(SO101SceneCfg):
    """SO101 + NVIDIA workshop vial-to-rack props (lightbox, mat, vials, rack)."""

    lightstudio = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/LightStudio",
        spawn=sim_utils.UsdFileCfg(usd_path=vta.LIGHTBOX_USD),
        init_state=AssetBaseCfg.InitialStateCfg(pos=vta.LIGHTBOX_SPAWN_POS),
    )

    mat = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Mat",
        spawn=sim_utils.UsdFileCfg(usd_path=vta.MAT_USDA),
        init_state=AssetBaseCfg.InitialStateCfg(pos=vta.MAT_SPAWN_POS, rot=vta.MAT_QUAT),
    )

    vial_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial_1",
        spawn=_vial_spawn_cfg(),
        init_state=RigidObjectCfg.InitialStateCfg(pos=vta.VIAL_SPAWNS[0], rot=vta.VIAL_QUAT),
    )
    vial_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial_2",
        spawn=_vial_spawn_cfg(),
        init_state=RigidObjectCfg.InitialStateCfg(pos=vta.VIAL_SPAWNS[1], rot=vta.VIAL_QUAT),
    )
    vial_3 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial_3",
        spawn=_vial_spawn_cfg(),
        init_state=RigidObjectCfg.InitialStateCfg(pos=vta.VIAL_SPAWNS[2], rot=vta.VIAL_QUAT),
    )

    rack_left = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Rack_Left",
        spawn=sim_utils.UsdFileCfg(
            usd_path=vta.VIAL_RACK_USDA,
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=vta.RACK_SPAWN),
    )

    tray = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Tray",
        spawn=sim_utils.UsdFileCfg(usd_path=vta.TRAY_USDA),
        init_state=RigidObjectCfg.InitialStateCfg(pos=vta.TRAY_SPAWN),
    )


def _so101_camera_cfg() -> TiledCameraCfg:
    return TiledCameraCfg(
        prim_path="",
        update_period=0.0,
        height=CAMERA_HEIGHT,
        width=CAMERA_WIDTH,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            projection_type="pinhole",
            f_stop=100.0,
            focal_length=13.5,
            focus_distance=0.05,
        ),
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="opengl",
        ),
    )


@configclass
class SO101VialTaskVisionSceneCfg(SO101VialTaskSceneCfg):
    """Vial task + workshop cameras (ego gripper + external D455 on lightbox)."""

    lightbox_light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/LightStudio/LightBox/RectLight",
    )

    camera_ego = _so101_camera_cfg().replace()
    camera_ego.prim_path = "{ENV_REGEX_NS}/Robot/gripper"
    camera_ego.offset.pos = (-0.005, 0.06, -0.062)
    camera_ego.offset.rot = euler_angles_to_quat(np.array([-45, 0, 0]), degrees=True)

    camera_external_D455 = _so101_camera_cfg().replace()
    camera_external_D455.prim_path = (
        "{ENV_REGEX_NS}/LightStudio/LightBox/camera_mount/rsd455/RSD455/Camera_OmniVision_OV9782_Right"
    )
    camera_external_D455.spawn = None


# Workshop vials_to_rack_env_cfg RTX settings (translucent vials)
SO101_VIAL_TASK_RTX_CARB_SETTINGS: dict[str, bool | float] = {
    "rtx.reflections.enabled": True,
    "rtx.translucency.reflectAtAllBounce": True,
    "rtx.translucency.sampleRoughness": True,
    "rtx.translucency.reflectionThroughputThreshold": 0.05,
    "rtx.translucency.maxRefractionBounces": 5,
    "rtx.raytracing.fractionalCutoutOpacity": True,
}


def vial_task_render_cfg(*, domain_rand: bool = False):
    """Render settings for vial task (translucency; full RTX when ``--domain_rand``)."""
    import isaaclab.sim as sim_utils

    if domain_rand:
        return sim_utils.RenderCfg(
            enable_translucency=True,
            enable_reflections=True,
            rendering_mode="quality",
            carb_settings=SO101_VIAL_TASK_RTX_CARB_SETTINGS,
        )
    return sim_utils.RenderCfg(enable_translucency=True)


def _sky_light_spawn_cfg() -> sim_utils.DomeLightCfg:
    kwargs: dict = {
        "intensity": 1000.0,
        "visible_in_primary_ray": False,
        "enable_color_temperature": True,
        "color_temperature": 6500.0,
    }
    if os.path.isfile(vta.HDRI_MOON_LAB):
        kwargs["texture_file"] = vta.HDRI_MOON_LAB
    return sim_utils.DomeLightCfg(**kwargs)


@configclass
class SO101VialTaskDRSceneCfg(SO101VialTaskSceneCfg):
    """Vial task + HDRI sky dome (workshop ``VialsToRackDRSceneCfg``)."""

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=500.0, color=(0.75, 0.75, 0.75)),
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/sky_light",
        spawn=_sky_light_spawn_cfg(),
    )


@configclass
class SO101VialTaskDRVisionSceneCfg(SO101VialTaskVisionSceneCfg):
    """Full DR vial task with ego + external cameras (workshop ``VialsToRackDRSceneCfg``)."""

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=500.0, color=(0.75, 0.75, 0.75)),
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/sky_light",
        spawn=_sky_light_spawn_cfg(),
    )
