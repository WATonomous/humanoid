"""Quest bimanual arm teleoperation via OpenXR/CloudXR — full immersive VR,
not just hand tracking. Runs inside the simulation_isaac container.

Unlike run_quest_bimanual_teleop.py (WebXR browser page -> ROS 2 -> this
script, hand-tracking input ONLY, Isaac Sim renders to the host monitor),
this script drives BOTH the sim's stereo rendering AND hand tracking through
Isaac Lab's native isaaclab.devices.openxr.OpenXRDevice, connected to the
NVIDIA CloudXR Runtime container (see
modules/docker-compose.simulation_isaac_cloudxr.patch.yaml). The Quest runs
the NVIDIA CloudXR Streaming Client app (not the Meta Browser) and connects
directly over Wi-Fi/LAN -- no adb reverse tunnels, no webxr_server.py, no
quest_teleop_node ROS bridge. None of that pipeline is used here.

Isaac Lab's OpenXRDevice is a standalone isaaclab.devices.DeviceBase --
no ManagerBasedRLEnvCfg/Gym task registration is required to use it (verified
via a headless smoke test: isaaclab.devices.openxr imports and the XR
session negotiates with the CloudXR runtime with just AppLauncher(xr=True)).
So this script reuses the exact same raw InteractiveScene + DLS fingertip-IK
loop as run_quest_bimanual_teleop.py (_ArmDlsController et al., copied
verbatim from there / bimanual_arm_cfg.py) and only replaces the INPUT layer:
QuestRosReceiver (ROS 2 /quest_teleop) -> OpenXRDevice.advance().

Coordinate mapping
------------------
OpenXR hand/head poses come from XRInputDevice.get_all_virtual_world_poses(),
i.e. already expressed in the USD stage's WORLD frame (Isaac Sim's XR
extension handles the physical-headset -> stage-world anchor transform
internally, per XrCfg.anchor_pos/anchor_rot below) -- unlike the WebXR
browser path, there is NO separate _QUEST_TO_WORLD remap matrix here; wrist
world-frame displacement is converted directly to the robot's base frame via
quat_apply_inverse(root_quat_w, ...), same last step as the other script.

No RealSense POV camera/viewport in this script -- Isaac Lab's own
remove_camera_configs() docstring warns extra cameras can cause rendering
conflicts with an active XR session, and it's moot here anyway: the headset
IS the camera.

Usage
-----
Inside the simulation_isaac container (cloudxr-runtime container must also
be up -- see the patch compose file):
    ISAAC_LAB=/workspace/isaaclab \\
    PYTHONPATH=/workspace/humanoid/autonomy/simulation/quest_isaac_teleop:$PYTHONPATH \\
    /workspace/isaaclab/isaaclab.sh -p \\
        /workspace/humanoid/autonomy/simulation/quest_isaac_teleop/run_quest_openxr_bimanual_teleop.py \\
        --device cpu
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

# ── path setup (must be before AppLauncher so PYTHONPATH is correct) ─────────
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parent
_KEYBOARD_TELEOP_DIR = _SIM_DIR / "Teleop" / "keyboard_based_teleoperation"
sys.path.insert(0, str(_KEYBOARD_TELEOP_DIR))

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Quest bimanual arm teleop, full immersive VR via OpenXR/CloudXR")
parser.add_argument("--gain", type=float, default=1.0,
                    help="Motion gain: metres of EE motion per metre of real wrist motion")
parser.add_argument("--anchor-x", type=float, default=0.9,
                     help="XR anchor position X (m) -- where the sim origin appears relative to your standing "
                          "position. Untested first guess: stand ~0.9m back from the robot's base so the arms "
                          "reach out toward you at a comfortable distance -- tune live.")
parser.add_argument("--anchor-y", type=float, default=0.0, help="XR anchor position Y (m).")
parser.add_argument("--anchor-z", type=float, default=0.0, help="XR anchor position Z (m).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.xr = True  # forces AppLauncher to load the isaaclab.python.xr.openxr.kit experience

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── post-launch imports (require Omniverse runtime) ───────────────────────────
import torch  # noqa: E402

import carb  # noqa: E402
import omni.appwindow  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.devices.device_base import DeviceBase  # noqa: E402
from isaaclab.devices.openxr import OpenXRDevice, OpenXRDeviceCfg, XrCfg  # noqa: E402
from isaaclab.devices.openxr.common import HAND_JOINT_NAMES  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_inv, quat_mul, subtract_frame_transforms  # noqa: E402

from bimanual_arm_cfg import (  # noqa: E402
    BIMANUAL_ARM_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_FINGER_TIP_BODIES,
    LEFT_FINGER_DISTAL_TIP_LOCAL,
    LEFT_GRIPPER_JOINTS,
    RIGHT_EE_BODY,
    RIGHT_FINGER_TIP_BODIES,
    RIGHT_FINGER_DISTAL_TIP_LOCAL,
    apply_joint_limits,
    compute_gripper_tip_pose_b,
    compute_tip_ik_jacobian,
    resolve_body_ids,
    resolve_joint_name,
)

assert "wrist" in HAND_JOINT_NAMES and "thumb_tip" in HAND_JOINT_NAMES and "index_tip" in HAND_JOINT_NAMES

# ── constants ─────────────────────────────────────────────────────────────────

_RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
_RIGHT_GRIPPER_OPEN = {"joint7": -0.05, "joint8": 0.05}
_RIGHT_GRIPPER_CLOSED = {"joint7": 0.0, "joint8": 0.0}

_GAIN_FAR = 1.0
_GAIN_RAMP_START_M = 0.15
_GAIN_RAMP_END_M = 0.35
_MAX_REACH_M = 0.28

# Untested starting point (identity = no correction) -- OpenXR's world-frame
# convention may still need a per-arm rotation offset the same way the WebXR
# path did (_WRIST_ORIENT_OFFSET_LEFT/RIGHT in run_quest_bimanual_teleop.py),
# but the *reason* for that offset there was a WebXR-hand-frame axis-
# convention mismatch specific to that browser's hand-tracking API -- OpenXR
# reports poses in a standardized frame, so the mismatch may not recur here.
# Retune live (same knob, same table in that script's README section) if
# rotating your wrist drives the gripper along the wrong axis.
_WRIST_ORIENT_OFFSET_LEFT = torch.tensor([1.0, 0.0, 0.0, 0.0])
_WRIST_ORIENT_OFFSET_RIGHT = torch.tensor([1.0, 0.0, 0.0, 0.0])

_PINCH_CLOSE_M = 0.030
_PINCH_OPEN_M = 0.050

_DLS_LAMBDA = 0.2
_DLS_LAMBDA_RIGHT = 0.35

_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))
_ENCLOSURE_X_SHIFT = -0.2
_ENCLOSURE_BACK_X = -0.25 + _ENCLOSURE_X_SHIFT
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
_ENCLOSURE_TOP_Z = 0.9
_ENCLOSURE_FRONT_X = 0.323 + _ENCLOSURE_X_SHIFT
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_FRONT_X - _ENCLOSURE_BACK_X
_ENCLOSURE_X_CTR = (_ENCLOSURE_BACK_X + _ENCLOSURE_FRONT_X) / 2
_TABLE_X_MIN, _TABLE_X_MAX = _ENCLOSURE_BACK_X, 0.6
_TABLE_Y_MIN, _TABLE_Y_MAX = _ENCLOSURE_Y_MIN, _ENCLOSURE_Y_MAX
_TABLE_THICKNESS = 0.05


# ── scene ─────────────────────────────────────────────────────────────────────

@configclass
class BimanualSceneCfg(InteractiveSceneCfg):
    """Bare arm stand + the white lightbox enclosure walls/table -- same
    geometry as run_quest_bimanual_teleop.py's BimanualSceneCfg. No external
    camera here (see module docstring: XR + extra cameras don't mix)."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    enclosure_back: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureBack",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_BACK_X, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(0.003, _ENCLOSURE_Y_SPAN, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_left: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureLeft",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MIN, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_right: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureRight",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MAX, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_top: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureTop",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, _ENCLOSURE_Y_SPAN, 0.003), visual_material=_ENCLOSURE_MATERIAL),
    )
    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=((_TABLE_X_MIN + _TABLE_X_MAX) / 2, (_TABLE_Y_MIN + _TABLE_Y_MAX) / 2, -_TABLE_THICKNESS / 2)
        ),
        spawn=sim_utils.CuboidCfg(
            size=(_TABLE_X_MAX - _TABLE_X_MIN, _TABLE_Y_MAX - _TABLE_Y_MIN, _TABLE_THICKNESS),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.3, 0.25)),
        ),
    )


# ── helpers ───────────────────────────────────────────────────────────────────

def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {n: i for i, n in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, n)] for n in names]


def _make_entity_cfg(scene, joint_names: list[str], ee_body: str) -> SceneEntityCfg:
    cfg = SceneEntityCfg("robot", joint_names=joint_names, body_names=[ee_body])
    cfg.resolve(scene)
    return cfg


def _ramped_gain(disp_m: torch.Tensor, gain_near: float) -> torch.Tensor:
    t = ((disp_m - _GAIN_RAMP_START_M) / (_GAIN_RAMP_END_M - _GAIN_RAMP_START_M)).clamp(0.0, 1.0)
    t = t * t * (3.0 - 2.0 * t)
    return gain_near + (_GAIN_FAR - gain_near) * t


# OpenXRDevice's default/untracked joint pose sentinel (see DeviceBase.reset()
# and openxr_device.py's `default_pose = np.array([0, 0, 0, 1, 0, 0, 0])`) --
# same "all zero position, identity quat" sentinel pattern as the WebXR path's
# _is_tracked, just matching THIS device's actual default value.
def _is_tracked(pos: torch.Tensor, quat_wxyz: torch.Tensor, eps: float = 1e-6) -> bool:
    return not (
        torch.allclose(pos, torch.zeros_like(pos), atol=eps)
        and torch.allclose(quat_wxyz, torch.tensor([1.0, 0.0, 0.0, 0.0], device=quat_wxyz.device), atol=eps)
    )


def _joint_pos_quat(hand_data: dict, joint_name: str, device) -> tuple[torch.Tensor, torch.Tensor]:
    """hand_data[joint_name] is a 7-elem numpy array [x,y,z,qw,qx,qy,qz] (see
    OpenXRDevice._get_raw_data's docstring). Returns (pos_xyz, quat_wxyz)."""
    pose = hand_data[joint_name]
    pos = torch.tensor(pose[0:3], dtype=torch.float32, device=device)
    quat = torch.tensor(pose[3:7], dtype=torch.float32, device=device)
    return pos, quat


def _pinch_dist(hand_data: dict) -> float:
    thumb, _ = _joint_pos_quat(hand_data, "thumb_tip", "cpu")
    index, _ = _joint_pos_quat(hand_data, "index_tip", "cpu")
    return (thumb - index).norm().item()


def _ee_pose_in_base(robot, body_id: int):
    root_pose_w = robot.data.root_state_w[:, 0:7]
    ee_pose_w = robot.data.body_state_w[:, body_id, 0:7]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )


class _ArmDlsController:
    """Copied verbatim from run_quest_bimanual_teleop.py -- see that file for
    the full rationale on lambda_val, fingertip-tip referencing, and the
    snap-to-DLS-solution actuator-lag workaround."""

    def __init__(self, scene, robot, device, arm_joint_names, ee_body, finger_tip_bodies, finger_distal_local,
                 lambda_val=_DLS_LAMBDA):
        self.finger_tip_bodies = finger_tip_bodies
        self.finger_distal_local = finger_distal_local

        entity_cfg = _make_entity_cfg(scene, arm_joint_names, ee_body)
        self.body_id = entity_cfg.body_ids[0]
        self.arm_ids = entity_cfg.joint_ids
        self.ee_jacobi_idx = self.body_id - 1 if robot.is_fixed_base else self.body_id
        self.finger_body_ids = resolve_body_ids(robot, finger_tip_bodies)

        cfg = DifferentialIKControllerCfg(
            command_type="pose", use_relative_mode=False, ik_method="dls",
            ik_params={"lambda_val": lambda_val},
        )
        self.controller = DifferentialIKController(cfg, num_envs=scene.num_envs, device=device)
        self.controller.reset(env_ids=torch.arange(scene.num_envs, device=device))

        self.home_xyz: torch.Tensor | None = None
        self.home_quat: torch.Tensor | None = None
        self.home_tip_pos_b: torch.Tensor | None = None
        self.home_tip_quat_b: torch.Tensor | None = None
        self.wrist_orient_offset: torch.Tensor | None = None

    def tip_pose_b(self, robot, root_pose_w):
        return compute_gripper_tip_pose_b(
            robot, root_pose_w, self.body_id, self.finger_body_ids,
            self.finger_tip_bodies, self.finger_distal_local,
        )

    def solve_and_apply(self, robot, device, target_pos_b, target_quat_b):
        root_pose_w = robot.data.root_state_w[:, 0:7]
        wrist_pos_b, _ = _ee_pose_in_base(robot, self.body_id)
        tip_pos_b, tip_quat_b = self.tip_pose_b(robot, root_pose_w)
        jacobian_w = robot.root_physx_view.get_jacobians()[:, self.ee_jacobi_idx, :, self.arm_ids]
        jacobian_b = compute_tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b)

        self.controller.set_command(torch.cat([target_pos_b, target_quat_b], dim=1))
        joint_pos = robot.data.joint_pos[:, self.arm_ids]
        joint_pos_des = self.controller.compute(tip_pos_b, tip_quat_b, jacobian_b, joint_pos)

        robot.set_joint_position_target(joint_pos_des, joint_ids=self.arm_ids)
        robot.write_joint_state_to_sim(
            joint_pos_des,
            torch.zeros((1, len(self.arm_ids)), device=device),
            joint_ids=self.arm_ids,
        )
        return tip_pos_b


# ── main simulation loop ──────────────────────────────────────────────────────

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene) -> None:
    robot = scene["robot"]
    device = sim.device
    sim_dt = sim.get_physics_dt()
    gain = args_cli.gain

    scene.update(sim_dt)
    apply_joint_limits(robot)

    left_arm_names = [resolve_joint_name(robot, n) for n in LEFT_ARM_JOINTS]
    right_arm_names = [resolve_joint_name(robot, n) for n in _RIGHT_ARM_JOINTS]

    left_arm = _ArmDlsController(
        scene, robot, device, left_arm_names, LEFT_EE_BODY,
        LEFT_FINGER_TIP_BODIES, LEFT_FINGER_DISTAL_TIP_LOCAL,
    )
    right_arm = _ArmDlsController(
        scene, robot, device, right_arm_names, RIGHT_EE_BODY,
        RIGHT_FINGER_TIP_BODIES, RIGHT_FINGER_DISTAL_TIP_LOCAL,
        lambda_val=_DLS_LAMBDA_RIGHT,
    )
    left_arm.wrist_orient_offset = _WRIST_ORIENT_OFFSET_LEFT.to(device).unsqueeze(0)
    right_arm.wrist_orient_offset = _WRIST_ORIENT_OFFSET_RIGHT.to(device).unsqueeze(0)

    left_gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    right_gripper_ids = _joint_ids(robot, _RIGHT_GRIPPER_JOINTS)

    left_g_open = torch.tensor([[GRIPPER_OPEN["joint7l"], GRIPPER_OPEN["joint8l"]]], device=device)
    left_g_closed = torch.tensor([[GRIPPER_CLOSED["joint7l"], GRIPPER_CLOSED["joint8l"]]], device=device)
    right_g_open = torch.tensor([[_RIGHT_GRIPPER_OPEN["joint7"], _RIGHT_GRIPPER_OPEN["joint8"]]], device=device)
    right_g_closed = torch.tensor([[_RIGHT_GRIPPER_CLOSED["joint7"], _RIGHT_GRIPPER_CLOSED["joint8"]]], device=device)

    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    root_pose_w0 = robot.data.root_state_w[:, 0:7]
    init_tip_pos_b_l, init_tip_quat_b_l = left_arm.tip_pose_b(robot, root_pose_w0)
    init_tip_pos_b_r, init_tip_quat_b_r = right_arm.tip_pose_b(robot, root_pose_w0)
    print(f"[Quest-XR][diag] rest EE pos (base frame) L(tip)={init_tip_pos_b_l[0].tolist()} "
          f"R(tip)={init_tip_pos_b_r[0].tolist()}", flush=True)

    # XR anchor: where the sim's world origin appears relative to your
    # physical standing position when you put the headset on. Static (no
    # anchor_prim_path) since the robot is fixed-base -- untested first
    # values, tune live with --anchor-x/-y/-z.
    xr_cfg = XrCfg(anchor_pos=(args_cli.anchor_x, args_cli.anchor_y, args_cli.anchor_z))
    device_cfg = OpenXRDeviceCfg(xr_cfg=xr_cfg, sim_device=str(device))
    xr_device = OpenXRDevice(device_cfg, retargeters=[])
    xr_device.reset()
    print(f"[Quest-XR] {xr_device}", flush=True)

    # Targets applied every frame (unconditionally), updated only while tracked.
    target_pos_b_left = init_tip_pos_b_l.clone()
    target_quat_b_left = init_tip_quat_b_l.clone()
    target_pos_b_right = init_tip_pos_b_r.clone()
    target_quat_b_right = init_tip_quat_b_r.clone()

    left_closed = False
    right_closed = False

    def _sphere_cfg(color, radius, opacity=1.0):
        return sim_utils.SphereCfg(
            radius=radius,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, opacity=opacity),
        )

    left_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/left_ik_target",
        markers={"sphere": _sphere_cfg((0.2, 0.5, 1.0), 0.05, opacity=0.3)},
    ))
    right_target_vis = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/right_ik_target",
        markers={"sphere": _sphere_cfg((1.0, 0.3, 0.1), 0.05, opacity=0.3)},
    ))

    def _recalibrate() -> None:
        left_arm.home_xyz = None
        right_arm.home_xyz = None
        print("[Quest-XR] Recalibrating — hold wrists in a comfortable pose.", flush=True)

    def _on_keyboard_event(event, *args, **kwargs) -> None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name == "R":
            _recalibrate()

    _keyboard_iface = carb.input.acquire_input_interface()
    _keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    _keyboard_sub = _keyboard_iface.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

    print("[Quest-XR] Ready. Waiting for OpenXR hand-tracking data.", flush=True)
    print("[Quest-XR] Both arms: Differential IK (DLS), fingertip-tip target.", flush=True)
    print("[Quest-XR] Connect via the NVIDIA CloudXR Streaming Client app on the Quest.", flush=True)
    print("[Quest-XR] Press R (Isaac Sim window focused) to recalibrate to your current pose.", flush=True)

    diag_frame = 0
    while simulation_app.is_running():
        raw = xr_device.advance()  # no retargeters -> raw dict, see device_base.py
        left_hand = raw.get(DeviceBase.TrackingTarget.HAND_LEFT)
        right_hand = raw.get(DeviceBase.TrackingTarget.HAND_RIGHT)

        if left_hand is not None and right_hand is not None:
            left_xyz_q, left_quat = _joint_pos_quat(left_hand, "wrist", device)
            right_xyz_q, right_quat = _joint_pos_quat(right_hand, "wrist", device)

            left_tracked = _is_tracked(left_xyz_q, left_quat)
            right_tracked = _is_tracked(right_xyz_q, right_quat)

            root_quat_w = robot.data.root_state_w[:, 3:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]

            # ── Homing (fingertip-tip anchored) -- world-frame directly, no
            # _QUEST_TO_WORLD remap: OpenXR poses are already USD stage-world
            # (see module docstring). ──────────────────────────────────────
            if left_arm.home_xyz is None and left_tracked:
                left_arm.home_xyz = left_xyz_q.clone()
                left_arm.home_quat = left_quat.clone()
                left_arm.home_tip_pos_b = init_tip_pos_b_l.clone()
                left_arm.home_tip_quat_b = init_tip_quat_b_l.clone()
                target_pos_b_left = left_arm.home_tip_pos_b.clone()
                target_quat_b_left = left_arm.home_tip_quat_b.clone()
                print(f"[Quest-XR] Left wrist tracked — homed at {left_xyz_q.tolist()}", flush=True)

            if right_arm.home_xyz is None and right_tracked:
                right_arm.home_xyz = right_xyz_q.clone()
                right_arm.home_quat = right_quat.clone()
                right_arm.home_tip_pos_b = init_tip_pos_b_r.clone()
                right_arm.home_tip_quat_b = init_tip_quat_b_r.clone()
                target_pos_b_right = right_arm.home_tip_pos_b.clone()
                target_quat_b_right = right_arm.home_tip_quat_b.clone()
                print(f"[Quest-XR] Right wrist tracked — homed at {right_xyz_q.tolist()}", flush=True)

            # ── Target (fingertip-tip, base frame) ──────────────────────────
            if left_arm.home_xyz is not None and left_tracked:
                left_disp_w = left_xyz_q - left_arm.home_xyz
                left_gain = _ramped_gain(left_disp_w.norm(), gain)
                left_disp_w = left_disp_w * left_gain
                left_disp_w = left_disp_w * min(1.0, _MAX_REACH_M / max(left_disp_w.norm().item(), 1e-6))
                target_pos_b_left = (
                    left_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, left_disp_w.unsqueeze(0))
                )
                dq_left_world = quat_mul(left_quat.unsqueeze(0), quat_inv(left_arm.home_quat.unsqueeze(0)))
                dq_left_world = quat_mul(quat_mul(left_arm.wrist_orient_offset, dq_left_world),
                                         quat_inv(left_arm.wrist_orient_offset))
                dq_left_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_left_world), root_quat_w)
                target_quat_b_left = quat_mul(dq_left_base, left_arm.home_tip_quat_b)

            if right_arm.home_xyz is not None and right_tracked:
                right_disp_w = right_xyz_q - right_arm.home_xyz
                right_gain = _ramped_gain(right_disp_w.norm(), gain)
                right_disp_w = right_disp_w * right_gain
                right_disp_w = right_disp_w * min(1.0, _MAX_REACH_M / max(right_disp_w.norm().item(), 1e-6))
                target_pos_b_right = (
                    right_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, right_disp_w.unsqueeze(0))
                )
                dq_right_world = quat_mul(right_quat.unsqueeze(0), quat_inv(right_arm.home_quat.unsqueeze(0)))
                dq_right_world = quat_mul(quat_mul(right_arm.wrist_orient_offset, dq_right_world),
                                          quat_inv(right_arm.wrist_orient_offset))
                dq_right_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_right_world), root_quat_w)
                target_quat_b_right = quat_mul(dq_right_base, right_arm.home_tip_quat_b)

            diag_frame += 1
            if diag_frame % 100 == 0:
                tip_pos_b_l, _ = left_arm.tip_pose_b(robot, root_pose_w)
                tip_pos_b_r, _ = right_arm.tip_pose_b(robot, root_pose_w)
                if left_arm.home_xyz is not None and left_tracked:
                    err_l = (target_pos_b_left - tip_pos_b_l).norm().item()
                    print(f"[Quest-XR][diag] L(tip) target_err={err_l:.3f}m", flush=True)
                if right_arm.home_xyz is not None and right_tracked:
                    err_r = (target_pos_b_right - tip_pos_b_r).norm().item()
                    print(f"[Quest-XR][diag] R(tip) target_err={err_r:.3f}m", flush=True)

            _rp = robot.data.root_state_w[:, :3]
            if left_arm.home_xyz is not None:
                left_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_left))
            if right_arm.home_xyz is not None:
                right_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_right))

            if left_tracked:
                ld = _pinch_dist(left_hand)
                if ld < _PINCH_CLOSE_M:
                    left_closed = False
                elif ld > _PINCH_OPEN_M:
                    left_closed = True

            if right_tracked:
                rd = _pinch_dist(right_hand)
                if rd < _PINCH_CLOSE_M:
                    right_closed = False
                elif rd > _PINCH_OPEN_M:
                    right_closed = True

        # ── Differential IK (DLS) solve, both arms, every frame ─────────────────
        left_arm.solve_and_apply(robot, device, target_pos_b_left, target_quat_b_left)
        right_arm.solve_and_apply(robot, device, target_pos_b_right, target_quat_b_right)

        # ── Gripper targets ────────────────────────────────────────────────────
        robot.set_joint_position_target(left_g_closed if left_closed else left_g_open, joint_ids=left_gripper_ids)
        robot.set_joint_position_target(right_g_closed if right_closed else right_g_open, joint_ids=right_gripper_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(left_gripper_ids), device=device), joint_ids=left_gripper_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(right_gripper_ids), device=device), joint_ids=right_gripper_ids)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene = InteractiveScene(BimanualSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    print("[Quest-XR] Simulation ready (lightbox enclosure walls added).", flush=True)
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
