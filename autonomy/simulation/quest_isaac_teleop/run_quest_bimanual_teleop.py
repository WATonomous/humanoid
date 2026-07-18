"""Quest bimanual arm teleoperation — runs inside the simulation_isaac
container.

Both arms are controlled via Quest hand tracking. Hand pose data arrives via
ROS 2 /quest_teleop topic published by quest_teleop_node (also in this
container). The left Quest wrist drives the left arm, the right Quest wrist
drives the right arm. Pinching thumb + index closes the corresponding
gripper.

IK solver: isaaclab.controllers.DifferentialIKController (ik_method="dls",
damped least squares), tracking each gripper's fingertip-tip-center frame —
the same controller/target used by
Task_space_controller/robot_arm_controllers/task_space_test.py, just fed
from the Quest wrists instead of a draggable cube.

Each arm is fingertip-tip referenced (LEFT_/RIGHT_FINGER_TIP_BODIES in
bimanual_arm_cfg.py: midpoint of each gripper's two finger distal tips), not
the raw wrist link — see compute_gripper_tip_pose_b / compute_tip_ik_jacobian
there. RIGHT_FINGER_DISTAL_TIP_LOCAL was measured directly from the link7/
link8 STL vertex data (mesh outward-X extreme, Y/Z bounding-box center) using
the same method as the pre-existing LEFT_FINGER_DISTAL_TIP_LOCAL constants —
link7/link8 are NOT a simple mirror of link7l/link8l (separate STL files,
swapped X-extent — see bimanual_arm_cfg.py for the measurement).

Orientation of each tip frame is defined as its own wrist link's orientation
(see compute_gripper_tip_pose_w); only the tracked *position* is offset to
each fingertip midpoint instead of the wrist origin.

lambda_val=0.2 (both arms) was tuned live on the left arm: at the DLS
default (0.01), a ~0.1m commanded tip displacement pushed the Jacobian
condition number from ~60 to 4000+ and some axes moved AWAY from target
instead of converging. 0.2 keeps the condition number bounded (~100) and
converges to a stable ~0.05m residual instead. Not yet validated on the
right arm specifically (mechanically mirrors the left, so 0.2 is a
reasonable starting point) — retune per-arm live if needed.

Coordinate mapping
------------------
WebXR uses a Y-up frame (X-right, Y-up, -Z-forward). The robot base is in
a Z-up world frame and is rotated 180° about Z, so the mapping is applied in
two stages (see _QUEST_TO_WORLD and the per-arm delta computation below).

Usage
-----
Inside the simulation_isaac container:
    ISAAC_LAB=/workspace/isaaclab \\
    PYTHONPATH=/workspace/humanoid/autonomy/simulation/quest_isaac_teleop:$PYTHONPATH \\
    /workspace/isaaclab/isaaclab.sh -p \\
        /workspace/humanoid/autonomy/simulation/quest_isaac_teleop/run_quest_bimanual_teleop.py \\
        --device cpu

Or via the helper script:
    ./run_quest_bimanual_teleop.sh
"""

import argparse
import sys
import threading
from pathlib import Path

from isaaclab.app import AppLauncher

# ── path setup (must be before AppLauncher so PYTHONPATH is correct) ─────────
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parent
_KEYBOARD_TELEOP_DIR = _SIM_DIR / "Teleop" / "keyboard_based_teleoperation"
sys.path.insert(0, str(_KEYBOARD_TELEOP_DIR))

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Quest bimanual arm teleop (both arms: DLS fingertip IK)")
parser.add_argument("--gain", type=float, default=1.0,
                    help="Motion gain: metres of EE motion per metre of real wrist motion")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── post-launch imports (require Omniverse runtime) ───────────────────────────
import rclpy  # noqa: E402
import torch  # noqa: E402
from rclpy.node import Node  # noqa: E402
from common_msgs.msg import QuestHandPose  # noqa: E402

import carb  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
    quat_apply,
    quat_apply_inverse,
    quat_from_matrix,
    quat_inv,
    quat_mul,
    subtract_frame_transforms,
)

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

# ── constants ─────────────────────────────────────────────────────────────────

_RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
_RIGHT_GRIPPER_OPEN = {"joint7": -0.05, "joint8": 0.05}
_RIGHT_GRIPPER_CLOSED = {"joint7": 0.0, "joint8": 0.0}

# WebXR (Y-up: X-right, Y-up, -Z-forward) -> simulation world frame (Z-up).
# Identical mapping to run_quest_bimanual_teleop.py — see that file for the
# derivation/rationale of each constant below.
# X/Z swapped (2026-07-18) -- live testing showed physical forward/back hand
# motion driving world Y (lateral) and physical left/right hand motion
# driving world X (forward-reach), for BOTH arms -- i.e. this headset/
# webxr_server's local reference space has quest X and Z swapped relative to
# the assumed X-right/-Z-forward convention (its local-floor "forward" axis
# was anchored 90 deg off from the physical direction the operator treats as
# forward, not a per-arm issue). Fix: route qx into the forward-reach row and
# qz into the lateral row (previously the reverse), keeping the same
# coefficient signs each row had before so the already-tuned axis_sign_left/
# right lateral-direction flip (see below) still lands on the correct
# (now qz-sourced) row unchanged. Still a proper rotation (det +1, verified) --
# do not further negate a row, that corrupts orientation via quat_from_matrix.
_QUEST_TO_WORLD = torch.tensor(
    [[-1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, 1.0, 0.0]],
    dtype=torch.float32,
)
_AXIS_SIGN_LEFT = torch.tensor([1.0, -1.0, 1.0])
_AXIS_SIGN_RIGHT = torch.tensor([1.0, -1.0, 1.0])

_GAIN_FAR = 1.0
_GAIN_RAMP_START_M = 0.15
_GAIN_RAMP_END_M = 0.35

_MAX_REACH_M = 0.28

# Base-yaw trial (2026-07-18) tried and REJECTED -- user confirmed it swung
# the gripper to the wrong side of the mount/stand bar. Reverting to the
# shared _DEFAULT_JOINT_POS (bimanual_arm_cfg.py) unmodified; the fix instead
# moves the lightbox enclosure back to give the arm's original rest pose room
# (see _ENCLOSURE_X_SHIFT below), not the arm's joints.

_WRIST_ORIENT_OFFSET_LEFT = torch.tensor([1.0, 0.0, 0.0, 0.0])
_WRIST_ORIENT_OFFSET_RIGHT = torch.tensor([0.0, 0.0, 0.0, 1.0])
# Trying 180deg about world Z first (2026-07-18) -- user reported the right
# arm's orientation feels "backwards" during live tracking. Auto-cal removal
# reverted wrist_orient_offset to identity (no correction) for both arms, and
# the README already documents the raw uncorrected per-arm axis-convention
# mismatch as asymmetric (previously ~30 right / ~90 left) -- but if a 180deg
# offset doesn't fix a genuine "rotate one way, EE goes the opposite way"
# feel, this isn't a fixed-angle misalignment at all, it's a chirality/mirror
# issue (WebXR often reports left/right hand joint orientations in mirrored
# local conventions, since a hand skeleton itself is chiral) -- that can't be
# fixed with any rotation-offset quaternion here, it needs a sign flip on the
# raw quest wrist quaternion components instead. Retune/replace per the
# README's "Wrist orientation alignment" table based on what the live test
# shows.

_THUMB_TIP_IDX = 4
_INDEX_TIP_IDX = 9
_PINCH_CLOSE_M = 0.030
_PINCH_OPEN_M = 0.050

_DLS_LAMBDA = 0.2
# Right arm gets extra damping (not a smaller _MAX_REACH_M) to deal with the
# instability observed near full extension: live-session diagnostics
# (2026-07-18) showed it freezing mid-reach for ~55s with target_err stuck
# above even the 0.28m reach clamp -- a real DLS solve normally converges on
# a static target within ~1s, so that plateau meant it hit an unstable/
# singular patch of its workspace it couldn't recover from without hitting R
# to recalibrate. The two arms' default joint poses were measured
# independently (not mirrored), so the right arm's rest pose isn't
# necessarily as well-conditioned for full-extension reaches as the left's.
# Shrinking _MAX_REACH_M would have sidestepped that region entirely but
# also given up real workspace; heavier damping keeps the full 0.28m reach
# and instead makes the solver more conservative (bounded correction per
# step) exactly where the Jacobian conditioning gets bad, at the cost of
# slower convergence there. Retune live if it now feels sluggish well before
# full extension, or still locks up at the extreme.
_DLS_LAMBDA_RIGHT = 0.35

_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))
# Shifted back 0.2m (2026-07-18) -- after the base-yaw joint trial (moving the
# arm instead) was rejected as the wrong direction, the fix is to move the box
# instead of the arm: same depth (X_SPAN unchanged), whole box translated
# toward -X to clear the mount/stand structural bar the back wall was
# crowding. Trial magnitude -- check live whether 0.2m is enough clearance or
# needs more.
_ENCLOSURE_X_SHIFT = -0.2
_ENCLOSURE_BACK_X = -0.25 + _ENCLOSURE_X_SHIFT
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
_ENCLOSURE_TOP_Z = 0.9
_ENCLOSURE_FRONT_X = 0.323 + _ENCLOSURE_X_SHIFT  # open front (arm reach direction), shifted with the rest of the box
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_FRONT_X - _ENCLOSURE_BACK_X
_ENCLOSURE_X_CTR = (_ENCLOSURE_BACK_X + _ENCLOSURE_FRONT_X) / 2


_TABLE_X_MIN, _TABLE_X_MAX = _ENCLOSURE_BACK_X, 0.6
_TABLE_Y_MIN, _TABLE_Y_MAX = _ENCLOSURE_Y_MIN, _ENCLOSURE_Y_MAX
_TABLE_THICKNESS = 0.05

# External RealSense D455, mounted on top of the base_link brace (the shared
# mount structure between the two arms) so a single feed shows BOTH grippers
# at once -- an operator overview camera, same idea as Ramy's PerceptionCamera
# in the old bimanual_arm_lightbox.usd (a plain Camera, not an rsd455 asset)
# and the SO101 vial task's camera_external_D455 (an rsd455 mounted on the
# lightbox, not the arm itself). Mounted as a child of base_link (not the env
# root) so it's expressed in the robot's own base frame -- rest tip base-frame
# X is ~0.42-0.43 (both grippers reach in +X, see the "[Quest][diag] rest EE
# pos" log line), so identity orientation (rsd455's local +X = lens boresight)
# already faces the grippers without a yaw flip. Height/offset above the brace
# is an untested first guess -- tune live.
_RSD455_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
_EXTERNAL_CAMERA_MOUNT = (
    (0.0, 0.0, 0.75),                              # translate xyz, relative to base_link
    (0.9238795325112867, 0.0, 0.3826834323650898, 0.0),  # orient wxyz -- 45deg downward pitch about Y (was identity/level)
)
# Sub-path to the actual renderable Camera prim inside the rsd455 payload --
# same as the SO101 vial task's camera_external_D455 (task_env_cfg.py).
_RSD455_CAMERA_SUBPATH = "rsd455/RSD455/Camera_OmniVision_OV9782_Right"


def _attach_rsd455_camera(parent_prim_path: str, translate: tuple, orient_wxyz: tuple) -> str:
    """Attach an rsd455 payload as a `camera_mount` child Xform of any prim
    (a robot link for a wrist camera, or the env root for a fixed external
    one). Returns the camera_mount prim path."""
    stage = omni.usd.get_context().get_stage()
    mount_path = f"{parent_prim_path}/camera_mount"
    mount_xform = UsdGeom.Xform.Define(stage, mount_path)
    xformable = UsdGeom.Xformable(mount_xform)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translate))
    w, x, y, z = orient_wxyz
    xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    rsd_prim = stage.DefinePrim(f"{mount_path}/rsd455")
    rsd_prim.GetPayloads().AddPayload(_RSD455_USD_URL)
    return mount_path


def _open_pov_viewport(camera_prim_path: str, window_name: str):
    """Best-effort: open a Kit viewport window locked to a camera prim, for a
    live wrist-POV feed on the host monitor. Returns the viewport_api (so the
    caller can periodically capture frames from it -- see
    _POV_FRAME_PATH/capture_viewport_to_file in run_simulator) or None if the
    viewport extension isn't available in this Kit experience -- this is a
    convenience feature, not required for teleop to function."""
    try:
        from omni.kit.viewport.utility import create_viewport_window

        viewport_window = create_viewport_window(window_name, width=640, height=480)
        viewport_window.viewport_api.camera_path = camera_prim_path
        return viewport_window.viewport_api
    except Exception as exc:  # noqa: BLE001 -- best-effort convenience feature
        print(f"[Quest] Could not open POV viewport '{window_name}': {exc}", flush=True)
        return None


# Frame the POV viewport gets captured to, for the Quest browser to fetch.
# Lives in the WebXR static dir (autonomy/teleop/quest_teleop/static/) so
# webxr_server.py's existing SimpleHTTPRequestHandler serves it as a plain
# static file -- no server code changes needed. Captured every
# _POV_CAPTURE_EVERY_N_STEPS sim steps (100Hz sim loop / 10 = ~10Hz feed).
_POV_FRAME_PATH = _SIM_DIR.parent / "teleop" / "quest_teleop" / "static" / "pov.png"
_POV_CAPTURE_EVERY_N_STEPS = 10


# ── helpers ───────────────────────────────────────────────────────────────────

@configclass
class BimanualSceneCfg(InteractiveSceneCfg):
    """Bare arm stand + the white lightbox enclosure walls/table (visual only, no collision on the
    walls) -- same geometry as BimanualPushBlockSceneCfg in
    HumanoidRLSetup/tasks/push/bimanual_env_cfg.py. Always on (no CLI flag)."""

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


class QuestRosReceiver(Node):
    """ROS 2 subscriber that caches the latest /quest_teleop message."""

    def __init__(self) -> None:
        super().__init__("quest_ik_listener")
        self._latest: QuestHandPose | None = None
        self._lock = threading.Lock()
        self.create_subscription(QuestHandPose, "/quest_teleop", self._cb, 1)
        self.get_logger().info("Subscribed to /quest_teleop")

    def _cb(self, msg: QuestHandPose) -> None:
        with self._lock:
            self._latest = msg

    def poll(self) -> QuestHandPose | None:
        with self._lock:
            return self._latest


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {n: i for i, n in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, n)] for n in names]


def _make_entity_cfg(scene, joint_names: list[str], ee_body: str) -> SceneEntityCfg:
    cfg = SceneEntityCfg("robot", joint_names=joint_names, body_names=[ee_body])
    cfg.resolve(scene)
    return cfg


def _wrist_xyz(wrist) -> torch.Tensor:
    p = wrist.position
    return torch.tensor([p.x, p.y, p.z], dtype=torch.float32)


def _wrist_quat_wxyz(wrist) -> torch.Tensor:
    o = wrist.orientation
    return torch.tensor([o.w, o.x, o.y, o.z], dtype=torch.float32)


def _ramped_gain(disp_m: torch.Tensor, gain_near: float) -> torch.Tensor:
    t = ((disp_m - _GAIN_RAMP_START_M) / (_GAIN_RAMP_END_M - _GAIN_RAMP_START_M)).clamp(0.0, 1.0)
    t = t * t * (3.0 - 2.0 * t)
    return gain_near + (_GAIN_FAR - gain_near) * t


def _is_tracked(pos: torch.Tensor, quat_wxyz: torch.Tensor, eps: float = 1e-6) -> bool:
    """True unless this is the WebXR "untracked hand" sentinel (see
    run_quest_bimanual_teleop.py for the full rationale)."""
    return not (
        torch.allclose(pos, torch.zeros_like(pos), atol=eps)
        and torch.allclose(quat_wxyz, torch.tensor([1.0, 0.0, 0.0, 0.0], device=quat_wxyz.device), atol=eps)
    )


def _pinch_dist(hand_joints: list) -> float:
    if len(hand_joints) != 75:
        return float("inf")
    ti, ii = _THUMB_TIP_IDX * 3, _INDEX_TIP_IDX * 3
    thumb = torch.tensor(hand_joints[ti:ti + 3])
    index = torch.tensor(hand_joints[ii:ii + 3])
    return (thumb - index).norm().item()


def _ee_pose_in_base(robot, body_id: int):
    root_pose_w = robot.data.root_state_w[:, 0:7]
    ee_pose_w = robot.data.body_state_w[:, body_id, 0:7]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )


class _ArmDlsController:
    """Bundles a DifferentialIKController with the per-arm state needed to
    drive it from Quest wrist data: entity/jacobian indices, fingertip
    geometry, homing state, and the current target. One instance per arm —
    kept separate (rather than sharing one controller with num_envs=2) so
    neither arm's homing/recalibration state can leak into the other's.
    """

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

        self.quest_home_xyz: torch.Tensor | None = None
        self.quest_home_quat: torch.Tensor | None = None
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

        # Snap directly to the DLS solution instead of waiting on the PD
        # actuator to catch up — see module docstring for why (live testing
        # found ~0.1 rad of actuator lag on the shoulder joint alone).
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

    _base_link_prim_path = "/World/envs/env_0/Robot/base_link"  # num_envs is always 1 in this script
    _base_link_prim = omni.usd.get_context().get_stage().GetPrimAtPath(_base_link_prim_path)
    print(f"[Quest] base_link prim valid={_base_link_prim.IsValid()} "
          f"authored={_base_link_prim.HasAuthoredReferences() or _base_link_prim.GetTypeName() != ''} "
          f"body_names={list(robot.data.body_names)}", flush=True)
    external_cam_mount = _attach_rsd455_camera(_base_link_prim_path, *_EXTERNAL_CAMERA_MOUNT)
    print(f"[Quest] External RealSense D455 (both-arms overview, on base_link brace) attached: {external_cam_mount}", flush=True)
    pov_viewport_api = _open_pov_viewport(f"{external_cam_mount}/{_RSD455_CAMERA_SUBPATH}", "Both-Arms POV")
    if pov_viewport_api is not None:
        _POV_FRAME_PATH.parent.mkdir(parents=True, exist_ok=True)
        print(f"[Quest] POV feed will be captured to {_POV_FRAME_PATH} "
              f"(served by webxr_server.py's static handler at /pov.png)", flush=True)

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

    quest_to_world = _QUEST_TO_WORLD.to(device)
    quest_to_world_quat = quat_from_matrix(quest_to_world.unsqueeze(0))
    axis_sign_left = _AXIS_SIGN_LEFT.to(device)
    axis_sign_right = _AXIS_SIGN_RIGHT.to(device)
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
    print(f"[Quest][diag] rest EE pos (base frame) L(tip)={init_tip_pos_b_l[0].tolist()} "
          f"R(tip)={init_tip_pos_b_r[0].tolist()}", flush=True)

    rclpy.init()
    receiver = QuestRosReceiver()
    threading.Thread(target=rclpy.spin, args=(receiver,), daemon=True).start()

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
        left_arm.quest_home_xyz = None
        right_arm.quest_home_xyz = None
        print("[Quest] Recalibrating — hold wrists in a comfortable pose.", flush=True)

    def _on_keyboard_event(event, *args, **kwargs) -> None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name == "R":
            _recalibrate()

    _keyboard_iface = carb.input.acquire_input_interface()
    _keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    _keyboard_sub = _keyboard_iface.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

    print("[Quest] Ready. Waiting for /quest_teleop messages.", flush=True)
    print("[Quest] Both arms: Differential IK (DLS), fingertip-tip target.", flush=True)
    print("[Quest] Connect the Quest browser to start streaming hand data.", flush=True)
    print("[Quest] Press R (Isaac Sim window focused) to recalibrate to your current pose.", flush=True)

    diag_frame = 0
    pov_capture_frame = 0
    while simulation_app.is_running():
        msg = receiver.poll()

        if msg is not None:
            left_xyz_q = _wrist_xyz(msg.left_wrist).to(device)
            right_xyz_q = _wrist_xyz(msg.right_wrist).to(device)
            left_quat = _wrist_quat_wxyz(msg.left_wrist).to(device)
            right_quat = _wrist_quat_wxyz(msg.right_wrist).to(device)
            left_joints = list(msg.left_hand_joints)
            right_joints = list(msg.right_hand_joints)

            left_tracked = _is_tracked(left_xyz_q, left_quat)
            right_tracked = _is_tracked(right_xyz_q, right_quat)

            root_quat_w = robot.data.root_state_w[:, 3:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]
            tip_pos_b_l, _ = left_arm.tip_pose_b(robot, root_pose_w)
            tip_pos_b_r, _ = right_arm.tip_pose_b(robot, root_pose_w)

            # ── Homing (fingertip-tip anchored) ─────────────────────────────────
            if left_arm.quest_home_xyz is None and left_tracked:
                left_arm.quest_home_xyz = left_xyz_q.clone()
                left_arm.quest_home_quat = left_quat.clone()
                # Anchor to the fixed launch-time rest tip pose, not wherever
                # the tip currently sits — see run_quest_bimanual_teleop.py's
                # equivalent comment for why (avoids baking in IK lag / making
                # recalibration a no-op).
                left_arm.home_tip_pos_b = init_tip_pos_b_l.clone()
                left_arm.home_tip_quat_b = init_tip_quat_b_l.clone()
                target_pos_b_left = left_arm.home_tip_pos_b.clone()
                target_quat_b_left = left_arm.home_tip_quat_b.clone()

                # NOTE: previously recomputed wrist_orient_offset here from
                # ee_quat_w_l / q_home_w_l ("auto-calibration"). Removed —
                # that formula (offset = ee_home * quest_home^-1, applied as
                # a conjugation on every subsequent delta) is mathematically
                # invalid: it relates two orientations with no physical
                # relationship (arbitrary robot rest pose vs. whatever way
                # the user's wrist happened to be pointing at homing) and a
                # single home sample can't determine the actual sensor/body
                # axis-convention correction (that needs multiple independent
                # rotation samples). Verified in sim: it converges to a
                # stable but ~50-60deg WRONG orientation (not a convergence
                # speed issue), and destabilizes tip position while doing so
                # — worse on the right arm (~0.11m residual vs ~0.06m left
                # for the same commanded rotation). wrist_orient_offset now
                # stays at the static _WRIST_ORIENT_OFFSET_LEFT/RIGHT
                # constant set below main(); re-tune those manually per the
                # README's "Wrist orientation alignment" table if rotation
                # axes still feel misaligned.
                print(f"[Quest] Left wrist tracked — homed at {left_xyz_q.tolist()}", flush=True)

            if right_arm.quest_home_xyz is None and right_tracked:
                right_arm.quest_home_xyz = right_xyz_q.clone()
                right_arm.quest_home_quat = right_quat.clone()
                right_arm.home_tip_pos_b = init_tip_pos_b_r.clone()
                right_arm.home_tip_quat_b = init_tip_quat_b_r.clone()
                target_pos_b_right = right_arm.home_tip_pos_b.clone()
                target_quat_b_right = right_arm.home_tip_quat_b.clone()

                # See matching note in the left-wrist homing block above —
                # auto-calibration removed, wrist_orient_offset stays static.
                print(f"[Quest] Right wrist tracked — homed at {right_xyz_q.tolist()}", flush=True)

            # ── Target (fingertip-tip, base frame) ──────────────────────────────
            if left_arm.quest_home_xyz is not None and left_tracked:
                left_disp_raw = left_xyz_q - left_arm.quest_home_xyz
                left_gain = _ramped_gain(left_disp_raw.norm(), gain)
                left_delta_w = (quest_to_world @ left_disp_raw) * left_gain * axis_sign_left
                left_delta_w = left_delta_w * min(1.0, _MAX_REACH_M / max(left_delta_w.norm().item(), 1e-6))
                target_pos_b_left_dbg = (
                    left_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, left_delta_w.unsqueeze(0))
                )
                if left_disp_raw.norm().item() > 0.05:
                    print(f"[Quest][axisdbg] L quest_disp(x,y,z)={left_disp_raw.tolist()}  "
                          f"world_delta={left_delta_w.tolist()}  "
                          f"base_delta_from_home={(target_pos_b_left_dbg - left_arm.home_tip_pos_b)[0].tolist()}",
                          flush=True)
                target_pos_b_left = target_pos_b_left_dbg
                dq_left = quat_mul(left_quat.unsqueeze(0), quat_inv(left_arm.quest_home_quat.unsqueeze(0)))
                dq_left_world = quat_mul(quat_mul(quest_to_world_quat, dq_left), quat_inv(quest_to_world_quat))
                dq_left_world = quat_mul(quat_mul(left_arm.wrist_orient_offset, dq_left_world),
                                         quat_inv(left_arm.wrist_orient_offset))
                dq_left_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_left_world), root_quat_w)
                target_quat_b_left = quat_mul(dq_left_base, left_arm.home_tip_quat_b)

            if right_arm.quest_home_xyz is not None and right_tracked:
                right_disp_raw = right_xyz_q - right_arm.quest_home_xyz
                right_gain = _ramped_gain(right_disp_raw.norm(), gain)
                right_delta_w = (quest_to_world @ right_disp_raw) * right_gain * axis_sign_right
                right_delta_w = right_delta_w * min(1.0, _MAX_REACH_M / max(right_delta_w.norm().item(), 1e-6))
                target_pos_b_right_dbg = (
                    right_arm.home_tip_pos_b + quat_apply_inverse(root_quat_w, right_delta_w.unsqueeze(0))
                )
                if right_disp_raw.norm().item() > 0.05:
                    print(f"[Quest][axisdbg] R quest_disp(x,y,z)={right_disp_raw.tolist()}  "
                          f"world_delta={right_delta_w.tolist()}  "
                          f"base_delta_from_home={(target_pos_b_right_dbg - right_arm.home_tip_pos_b)[0].tolist()}",
                          flush=True)
                target_pos_b_right = target_pos_b_right_dbg
                dq_right = quat_mul(right_quat.unsqueeze(0), quat_inv(right_arm.quest_home_quat.unsqueeze(0)))
                dq_right_world = quat_mul(quat_mul(quest_to_world_quat, dq_right), quat_inv(quest_to_world_quat))
                dq_right_world = quat_mul(quat_mul(right_arm.wrist_orient_offset, dq_right_world),
                                          quat_inv(right_arm.wrist_orient_offset))
                dq_right_base = quat_mul(quat_mul(quat_inv(root_quat_w), dq_right_world), root_quat_w)
                target_quat_b_right = quat_mul(dq_right_base, right_arm.home_tip_quat_b)

            diag_frame += 1
            if diag_frame % 100 == 0:
                if left_arm.quest_home_xyz is not None and left_tracked:
                    err_l = (target_pos_b_left - tip_pos_b_l).norm().item()
                    print(f"[Quest][diag] L(tip) gain={left_gain.item():.2f} target_err={err_l:.3f}m", flush=True)
                if right_arm.quest_home_xyz is not None and right_tracked:
                    err_r = (target_pos_b_right - tip_pos_b_r).norm().item()
                    print(f"[Quest][diag] R(tip) gain={right_gain.item():.2f} target_err={err_r:.3f}m", flush=True)

            _rp = robot.data.root_state_w[:, :3]
            if left_arm.quest_home_xyz is not None:
                left_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_left))
            if right_arm.quest_home_xyz is not None:
                right_target_vis.visualize(translations=_rp + quat_apply(root_quat_w, target_pos_b_right))

            if left_tracked:
                ld = _pinch_dist(left_joints)
                if ld < _PINCH_CLOSE_M:
                    left_closed = False
                elif ld > _PINCH_OPEN_M:
                    left_closed = True

            if right_tracked:
                rd = _pinch_dist(right_joints)
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

        pov_capture_frame += 1
        if pov_viewport_api is not None and pov_capture_frame % _POV_CAPTURE_EVERY_N_STEPS == 0:
            from omni.kit.viewport.utility import capture_viewport_to_file

            capture_viewport_to_file(pov_viewport_api, str(_POV_FRAME_PATH))


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene = InteractiveScene(BimanualSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    print("[Quest] Simulation ready (lightbox enclosure walls added).")
    run_simulator(sim, scene)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
