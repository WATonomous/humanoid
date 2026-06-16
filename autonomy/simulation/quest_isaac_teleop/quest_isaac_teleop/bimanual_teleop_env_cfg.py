"""CloudXR + Pink IK teleop env for the bimanual arm."""

import importlib.util
import math
import sys
from pathlib import Path

_SIMULATION_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SIMULATION_DIR / "Humanoid_Wato"))

from isaaclab_teleop import IsaacTeleopCfg, XrCfg

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.common import ViewerCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from quest_isaac_teleop.bimanual_pink_controller_cfg import (
    GRIPPER_CLOSED,
    GRIPPER_JOINTS,
    GRIPPER_OPEN,
    WATO_BIMANUAL_IK_ACTION_CFG,
)

# The keyboard-teleop config lives in a directory with a space, so load by path.
_BIMANUAL_CFG_PATH = (
    _SIMULATION_DIR / "Teleop" / "keyboard-based teleoperation" / "bimanual_arm_cfg.py"
)
_spec = importlib.util.spec_from_file_location("bimanual_arm_cfg", str(_BIMANUAL_CFG_PATH))
_bimanual_arm_cfg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bimanual_arm_cfg)
BIMANUAL_ARM_CFG = _bimanual_arm_cfg.BIMANUAL_ARM_CFG

_BIMANUAL_ROOT = _SIMULATION_DIR / "Humanoid_Wato" / "wato_bimanual_arm"
_URDF_PATH = str(_BIMANUAL_ROOT / "urdf" / "armDouble.SLDASM.urdf")
_MESH_DIR = str(_BIMANUAL_ROOT / "meshes")

_ROBOT_USD_PATH = str(
    _BIMANUAL_ROOT / "urdf" / "armDouble.SLDASM" / "armDouble.SLDASM_limits.usda"
)

_ROBOT_POS = (0.0, 0.0, 0.5)
_ROBOT_ROT = (0.0, 0.0, 1.0, 0.0)  # xyzw, 180 deg about Z

_VIEWER_EYE = (-0.198, -0.016, 1.329)
_VIEWER_LOOKAT = (0.667, -0.016, 0.828)

# Flip the z sign if left/right or forward is mirrored in the headset.
_XR_ANCHOR_POS = (0.0, 0.0, 0.0)
_XR_ANCHOR_ROT = (0.0, 0.0, -0.70710678, 0.70710678)


def _make_bimanual_gripper_retargeter_cls():
    """Build the retargeter class lazily (isaacteleop only imports under Isaac)."""
    import numpy as np

    from isaacteleop.retargeting_engine.interface import BaseRetargeter, RetargeterIOType
    from isaacteleop.retargeting_engine.interface.retargeter_core_types import RetargeterIO
    from isaacteleop.retargeting_engine.interface.tensor_group_type import (
        OptionalType,
        TensorGroupType,
    )
    from isaacteleop.retargeting_engine.tensor_types import (
        FloatType,
        HandInput,
        HandInputIndex,
        HandJointIndex,
    )

    class BimanualGripperRetargeter(BaseRetargeter):
        """Map thumb/index pinch to the four gripper joints."""

        def __init__(self, name: str, close_m: float = 0.03, open_m: float = 0.05) -> None:
            super().__init__(name=name)
            self._close_m = close_m
            self._open_m = open_m
            self._closed = {"left": False, "right": False}

        def input_spec(self) -> RetargeterIOType:
            return {
                "hand_right": OptionalType(HandInput()),
                "hand_left": OptionalType(HandInput()),
            }

        def output_spec(self) -> RetargeterIOType:
            return {
                "gripper_joints": TensorGroupType(
                    "gripper_joints", [FloatType(j) for j in GRIPPER_JOINTS]
                )
            }

        def _update_side(self, side: str, hand_group) -> bool:
            if hand_group.is_none:
                return self._closed[side]

            joint_positions = np.from_dlpack(hand_group[HandInputIndex.JOINT_POSITIONS])
            joint_valid = np.from_dlpack(hand_group[HandInputIndex.JOINT_VALID])

            if not (joint_valid[HandJointIndex.THUMB_TIP] and joint_valid[HandJointIndex.INDEX_TIP]):
                return self._closed[side]

            thumb = joint_positions[HandJointIndex.THUMB_TIP]
            index = joint_positions[HandJointIndex.INDEX_TIP]
            distance = float(np.linalg.norm(thumb - index))

            if distance < self._close_m:
                self._closed[side] = True
            elif distance > self._open_m:
                self._closed[side] = False
            return self._closed[side]

        def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
            if context.execution_events.reset:
                self._closed = {"left": False, "right": False}

            out = outputs["gripper_joints"]
            right_targets = GRIPPER_CLOSED if self._update_side("right", inputs["hand_right"]) else GRIPPER_OPEN
            left_targets = GRIPPER_CLOSED if self._update_side("left", inputs["hand_left"]) else GRIPPER_OPEN

            out[0] = right_targets["joint7"]
            out[1] = right_targets["joint8"]
            out[2] = left_targets["joint7l"]
            out[3] = left_targets["joint8l"]

    return BimanualGripperRetargeter


def _build_bimanual_pipeline():
    """Build the 18D teleop action pipeline."""
    from isaacteleop.retargeters import (
        Se3AbsRetargeter,
        Se3RetargeterConfig,
        TensorReorderer,
    )
    from isaacteleop.retargeting_engine.deviceio_source_nodes import HandsSource
    from isaacteleop.retargeting_engine.interface import OutputCombiner, ValueInput
    from isaacteleop.retargeting_engine.tensor_types import TransformMatrix

    BimanualGripperRetargeter = _make_bimanual_gripper_retargeter_cls()

    hands = HandsSource(name="hands")
    transform_input = ValueInput("world_T_anchor", TransformMatrix())
    transformed_hands = hands.transformed(transform_input.output(ValueInput.VALUE))

    right_se3 = Se3AbsRetargeter(
        Se3RetargeterConfig(
            input_device=HandsSource.RIGHT,
            zero_out_xy_rotation=False,
            use_wrist_rotation=True,
            use_wrist_position=True,
            target_offset_roll=0.0,
            target_offset_pitch=0.0,
            target_offset_yaw=0.0,
        ),
        name="right_ee_pose",
    )
    connected_right_se3 = right_se3.connect(
        {HandsSource.RIGHT: transformed_hands.output(HandsSource.RIGHT)}
    )

    left_se3 = Se3AbsRetargeter(
        Se3RetargeterConfig(
            input_device=HandsSource.LEFT,
            zero_out_xy_rotation=False,
            use_wrist_rotation=True,
            use_wrist_position=True,
            target_offset_roll=0.0,
            target_offset_pitch=0.0,
            target_offset_yaw=0.0,
        ),
        name="left_ee_pose",
    )
    connected_left_se3 = left_se3.connect(
        {HandsSource.LEFT: transformed_hands.output(HandsSource.LEFT)}
    )

    gripper = BimanualGripperRetargeter(name="gripper")
    connected_gripper = gripper.connect(
        {
            "hand_right": transformed_hands.output(HandsSource.RIGHT),
            "hand_left": transformed_hands.output(HandsSource.LEFT),
        }
    )

    right_ee_elements = ["r_pos_x", "r_pos_y", "r_pos_z", "r_quat_x", "r_quat_y", "r_quat_z", "r_quat_w"]
    left_ee_elements = ["l_pos_x", "l_pos_y", "l_pos_z", "l_quat_x", "l_quat_y", "l_quat_z", "l_quat_w"]
    gripper_elements = list(GRIPPER_JOINTS)

    reorderer = TensorReorderer(
        input_config={
            "right_ee_pose": right_ee_elements,
            "left_ee_pose": left_ee_elements,
            "gripper_joints": gripper_elements,
        },
        output_order=right_ee_elements + left_ee_elements + gripper_elements,
        name="action_reorderer",
        input_types={
            "right_ee_pose": "array",
            "left_ee_pose": "array",
            "gripper_joints": "scalar",
        },
    )
    connected_reorderer = reorderer.connect(
        {
            "right_ee_pose": connected_right_se3.output("ee_pose"),
            "left_ee_pose": connected_left_se3.output("ee_pose"),
            "gripper_joints": connected_gripper.output("gripper_joints"),
        }
    )

    pipeline = OutputCombiner({"action": connected_reorderer.output("output")})
    return pipeline, [right_se3, left_se3]


def _high_pd_actuators():
    """Stiffen the arm joints so they track IK targets closely."""
    actuators = dict(BIMANUAL_ARM_CFG.actuators)
    for name in ("left_shoulder", "left_elbow", "left_wrist", "right_arm"):
        actuators[name] = actuators[name].replace(stiffness=4400.0, damping=40.0)
    return actuators


@configclass
class BimanualTeleopSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot: ArticulationCfg = BIMANUAL_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        # The USD overlay already includes the joint limit fixes.
        spawn=sim_utils.UsdFileCfg(
            usd_path=_ROBOT_USD_PATH,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            activate_contact_sensors=False,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
            ),
        ),
        actuators=_high_pd_actuators(),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=_ROBOT_POS,
            rot=_ROBOT_ROT,
            joint_pos=BIMANUAL_ARM_CFG.init_state.joint_pos,
        ),
    )


@configclass
class ActionsCfg:
    bimanual_ik = WATO_BIMANUAL_IK_ACTION_CFG


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)


@configclass
class WatoBimanualTeleopEnvCfg(ManagerBasedRLEnvCfg):
    scene: BimanualTeleopSceneCfg = BimanualTeleopSceneCfg(num_envs=1, env_spacing=2.5)
    viewer: ViewerCfg = ViewerCfg(eye=_VIEWER_EYE, lookat=_VIEWER_LOOKAT, origin_type="world")
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    commands = None
    rewards = None
    curriculum = None

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 3600.0
        self.sim.dt = 1 / 200
        self.sim.render_interval = 2

        self.actions.bimanual_ik.controller.usd_path = None
        self.actions.bimanual_ik.controller.urdf_path = _URDF_PATH
        self.actions.bimanual_ik.controller.mesh_path = _MESH_DIR

        self.xr = XrCfg(
            anchor_pos=_XR_ANCHOR_POS,
            anchor_rot=_XR_ANCHOR_ROT,
            near_plane=0.1,
        )
        self.isaac_teleop = IsaacTeleopCfg(
            pipeline_builder=lambda: _build_bimanual_pipeline()[0],
            sim_device=self.sim.device,
            xr_cfg=self.xr,
        )
