"""armWithStand (wato_arm_v2) keyboard teleoperation (right/L-suffixed arm
only) in the SAME lightbox-enclosure scene and stereo RSD455 camera setup as
run_quest_armWithStand_teleop.py -- for testing camera framing/rest pose in
the actual VR-teleop scene, driven by keyboard instead of the Quest/ROS
pipeline. See keyboard_teleop_armv2.py for the bare-scene (no lightbox)
version this was combined with.

  K       Toggle gripper (open/close)
  W/S     Move along x-axis
  A/D     Move along y-axis
  Q/E     Move along z-axis
  Z/X     Rotate along x-axis
  T/G     Rotate along y-axis
  C/V     Rotate along z-axis
  R       Reset arm to default pose
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))

parser = argparse.ArgumentParser(description="Keyboard teleop for wato_arm_v2 in the lightbox scene.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import omni.usd
from pxr import Gf, UsdGeom

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, subtract_frame_transforms

from armWithStand_v2_cfg import (
    ARM_V2_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    make_ego_cam_cfg,
    make_wrist_cam_cfg,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    RIGHT_EE_BODY,
    RIGHT_FINGER_DISTAL_TIP_LOCAL,
    RIGHT_FINGER_TIP_BODIES,
    RIGHT_GRIPPER_JOINTS,
    apply_joint_limits,
    compute_gripper_tip_pose_b,
    compute_tip_ik_jacobian,
    resolve_body_ids,
    resolve_joint_name,
)

# ── lightbox enclosure geometry (identical to run_quest_armWithStand_teleop.py) ──
_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))
_ENCLOSURE_BACK_X = -0.45
_ENCLOSURE_FRONT_X = 0.75
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
_ENCLOSURE_TOP_Z = 1.8
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_FRONT_X - _ENCLOSURE_BACK_X
_ENCLOSURE_X_CTR = (_ENCLOSURE_BACK_X + _ENCLOSURE_FRONT_X) / 2

_TABLE_X_MIN, _TABLE_X_MAX = _ENCLOSURE_BACK_X, _ENCLOSURE_FRONT_X
_TABLE_Y_MIN, _TABLE_Y_MAX = _ENCLOSURE_Y_MIN, _ENCLOSURE_Y_MAX
_TABLE_THICKNESS = 0.05

# ── finalized stereo head camera (kept in sync manually with
# run_quest_armWithStand_teleop.py, same as keyboard_teleop_armv2.py) ──
_RSD455_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
_HEAD_VIEWPOINT_HOME_POS = (0.08667797629999999, 0.04, 0.25258678130000006)
_HEAD_VIEWPOINT_HOME_QUAT = (0.9063077870366499, 0.0, 0.42261826174069944, 0.0)
_EYE_LOCAL_RIGHT = torch.tensor([0.0, -1.0, 0.0])
_EYE_IPD_M = 0.063
_RSD455_CAMERA_SUBPATH = "rsd455/RSD455/Camera_OmniVision_OV9782_Right"


@configclass
class ArmV2LightboxSceneCfg(InteractiveSceneCfg):
    """Same lightbox enclosure + table + robot lift as run_quest_armWithStand_teleop.py."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = ARM_V2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ARM_V2_CFG.init_state.replace(pos=(0.0, 0.0, 1.1997)),
    )

    # Data-collection cameras. Defined in armWithStand_v2_cfg.py and shared with the other
    # teleop scripts -- adjust camera position and aim there. Requires --enable_cameras.
    ego_cam = make_ego_cam_cfg()
    wrist_cam = make_wrist_cam_cfg()

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


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


def _open_pov_viewport(camera_prim_path: str, window_name: str):
    try:
        from omni.kit.viewport.utility import create_viewport_window

        vw = create_viewport_window(window_name, width=640, height=480)
        vw.viewport_api.camera_path = camera_prim_path
        print(f"[INFO] Opened '{window_name}' -> {camera_prim_path}")
    except Exception as exc:  # noqa: BLE001 -- best-effort convenience feature
        print(f"[INFO] Could not open '{window_name}': {exc}")


def _attach_rsd455_camera(parent_prim_path: str, mount_name: str, translate: tuple, orient_wxyz: tuple) -> str:
    stage = omni.usd.get_context().get_stage()
    mount_path = f"{parent_prim_path}/{mount_name}"
    prim = UsdGeom.Xform.Define(stage, mount_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    o_op = xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    t_op.Set(Gf.Vec3d(*translate))
    w, x, y, z = orient_wxyz
    o_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    rsd_prim = stage.DefinePrim(f"{mount_path}/rsd455")
    rsd_prim.GetPayloads().AddPayload(_RSD455_USD_URL)
    return mount_path


def _open_stereo_pov_preview(robot_prim_path: str) -> None:
    base_link_path = f"{robot_prim_path}/base_link"
    head_quat = torch.tensor([_HEAD_VIEWPOINT_HOME_QUAT], dtype=torch.float32)
    head_pos = torch.tensor([_HEAD_VIEWPOINT_HOME_POS], dtype=torch.float32)
    right_offset = quat_apply(head_quat, _EYE_LOCAL_RIGHT.unsqueeze(0)) * (_EYE_IPD_M / 2)
    left_pos = (head_pos - right_offset)[0].tolist()
    right_pos = (head_pos + right_offset)[0].tolist()

    left_mount = _attach_rsd455_camera(base_link_path, "left_eye_camera_mount", tuple(left_pos), _HEAD_VIEWPOINT_HOME_QUAT)
    right_mount = _attach_rsd455_camera(base_link_path, "right_eye_camera_mount", tuple(right_pos), _HEAD_VIEWPOINT_HOME_QUAT)
    _open_pov_viewport(f"{left_mount}/{_RSD455_CAMERA_SUBPATH}", "Left Eye POV")
    _open_pov_viewport(f"{right_mount}/{_RSD455_CAMERA_SUBPATH}", "Right Eye POV")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()

    scene.update(sim_dt)
    apply_joint_limits(robot)

    _open_stereo_pov_preview("/World/envs/env_0/Robot")
    _open_pov_viewport("/World/envs/env_0/Robot/base_link/ego_cam", "Ego Cam Preview")
    _open_pov_viewport("/World/envs/env_0/Robot/link6l/wrist_cam", "Wrist Cam Preview")

    driven_arm_names = [resolve_joint_name(robot, name) for name in RIGHT_ARM_JOINTS]
    held_arm_names = [resolve_joint_name(robot, name) for name in LEFT_ARM_JOINTS[:6]]

    print(f"[INFO] Driven arm joints (link6l chain): {driven_arm_names}")
    print(f"[INFO] Held arm joints (link6 chain): {held_arm_names}")

    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    robot_entity_cfg = SceneEntityCfg("robot", joint_names=driven_arm_names, body_names=[RIGHT_EE_BODY])
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    finger_body_ids = resolve_body_ids(robot, RIGHT_FINGER_TIP_BODIES)

    driven_arm_ids = robot_entity_cfg.joint_ids
    driven_gripper_ids = _joint_ids(robot, RIGHT_GRIPPER_JOINTS)
    held_arm_ids = _joint_ids(robot, LEFT_ARM_JOINTS[:6])
    held_gripper_ids = _joint_ids(robot, LEFT_ARM_JOINTS[6:8])
    held_default_pos = robot.data.default_joint_pos[:, held_arm_ids].clone()
    held_gripper_default_pos = robot.data.default_joint_pos[:, held_gripper_ids].clone()

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    gripper_open_targets = torch.tensor([[GRIPPER_OPEN[name] for name in RIGHT_GRIPPER_JOINTS]], device=sim.device)
    gripper_closed_targets = torch.tensor([[GRIPPER_CLOSED[name] for name in RIGHT_GRIPPER_JOINTS]], device=sim.device)

    teleop = Se3Keyboard(Se3KeyboardCfg(pos_sensitivity=0.005, rot_sensitivity=0.05))
    should_reset = False

    def reset_arm():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_arm)
    teleop.reset()
    print("[INFO] Teleoperating link6l-chain arm only. Other arm held at default pose.")
    print("[INFO] Click the 3D viewport window, then press W/A/S/D/Q/E to move.")

    while simulation_app.is_running():
        if should_reset:
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            diff_ik_controller.reset()
            teleop.reset()
            should_reset = False

        raw = teleop.advance()
        command = raw[:6].to(dtype=torch.float32, device=sim.device).unsqueeze(0)
        close_gripper = bool(raw[6].item() < 0.0)

        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, driven_arm_ids]

        wrist_body_id = robot_entity_cfg.body_ids[0]
        wrist_pose_w = robot.data.body_state_w[:, wrist_body_id, 0:7]
        wrist_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], wrist_pose_w[:, 0:3], wrist_pose_w[:, 3:7],
        )
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, wrist_body_id, finger_body_ids,
            RIGHT_FINGER_TIP_BODIES, RIGHT_FINGER_DISTAL_TIP_LOCAL,
        )

        diff_ik_controller.set_command(command, ee_pos=tip_pos_b, ee_quat=tip_quat_b)

        jacobian_w = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, driven_arm_ids]
        jacobian_b = compute_tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b)
        joint_pos_des = diff_ik_controller.compute(tip_pos_b, tip_quat_b, jacobian_b, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=driven_arm_ids)

        gripper_targets = gripper_closed_targets if close_gripper else gripper_open_targets
        zero_gripper_vel = torch.zeros(1, len(driven_gripper_ids), device=sim.device)
        robot.set_joint_position_target(gripper_targets, joint_ids=driven_gripper_ids)
        robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=driven_gripper_ids)

        robot.set_joint_position_target(held_default_pos, joint_ids=held_arm_ids)
        robot.set_joint_position_target(held_gripper_default_pos, joint_ids=held_gripper_ids)
        robot.set_joint_velocity_target(
            torch.zeros(1, len(held_gripper_ids), device=sim.device), joint_ids=held_gripper_ids,
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene = InteractiveScene(ArmV2LightboxSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    print("[INFO] Setup complete (lightbox enclosure).")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
