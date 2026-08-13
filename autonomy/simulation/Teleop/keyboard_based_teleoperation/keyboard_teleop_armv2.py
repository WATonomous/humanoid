"""Minimal armWithStand (wato_arm_v2) keyboard teleoperation (right/L-suffixed
arm only) -- quick visual test tool for checking ego_cam framing while
driving the arm, not a recording pipeline (see keyboard_teleop.py for the
bimanual version this is adapted from, including --record support).

Robot: Humanoid_Wato wato_arm_v2 (armWithStand.usd)
Teleop bindings: https://isaac-sim.github.io/IsaacLab/v2.0.1/source/overview/teleop_imitation.html

  K       Toggle gripper (open/close)
  W/S     Move along x-axis
  A/D     Move along y-axis
  Q/E     Move along z-axis
  Z/X     Rotate along x-axis
  T/G     Rotate along y-axis
  C/V     Rotate along z-axis
  R       Reset arm to default pose

Also opens an "Ego Cam Preview" viewport locked to ego_cam, and the same
stereo "Left Eye POV"/"Right Eye POV" RSD455 pair (at the finalized
position/orientation) used in run_quest_armWithStand_teleop.py, so their
framing is visible live while driving the arm with the keyboard.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Keyboard teleoperation for wato_arm_v2 (right/L-suffixed arm only).")
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
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms

# Finalized stereo head camera (see run_quest_armWithStand_teleop.py for the
# derivation/tuning history) -- kept in sync manually, not imported, since
# this is a standalone quick-test script.
_RSD455_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
_HEAD_VIEWPOINT_HOME_POS = (0.08667797629999999, 0.04, 0.25258678130000006)
_HEAD_VIEWPOINT_HOME_QUAT = (0.9063077870366499, 0.0, 0.42261826174069944, 0.0)  # ~50deg downward pitch
_EYE_LOCAL_RIGHT = torch.tensor([0.0, -1.0, 0.0])
_EYE_IPD_M = 0.063
_RSD455_CAMERA_SUBPATH = "rsd455/RSD455/Camera_OmniVision_OV9782_Right"

from armWithStand_v2_cfg import (
    ARM_V2_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
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


@configclass
class ArmV2SceneCfg(InteractiveSceneCfg):
    """Minimal scene with wato_arm_v2, lifted so the stand rests on the ground plane."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # Same lift used in run_quest_armWithStand_teleop.py: the stand's bottom
    # sits ~1.1997m below the robot's own origin -- without this it's buried
    # in/under the ground plane.
    robot = ARM_V2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ARM_V2_CFG.init_state.replace(pos=(0.0, 0.0, 1.1997)),
    )

    # Data-collection cameras (kept in sync with run_quest_armv2_teleop.py) --
    # wrap the Camera prims already baked into armWithStand.usd's sensor
    # layer (spawn=None). Requires launching with --enable_cameras.
    ego_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/ego_cam",
        spawn=None, height=480, width=640, update_period=0.0, data_types=["rgb"],
    )
    wrist_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link6l/wrist_cam",
        spawn=None, height=480, width=640, update_period=0.0, data_types=["rgb"],
    )


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


def _open_pov_viewport(camera_prim_path: str, window_name: str):
    try:
        from omni.kit.viewport.utility import create_viewport_window

        viewport_window = create_viewport_window(window_name, width=640, height=480)
        viewport_window.viewport_api.camera_path = camera_prim_path
        print(f"[INFO] Opened '{window_name}' viewport locked to {camera_prim_path}")
        return viewport_window.viewport_api
    except Exception as exc:  # noqa: BLE001 -- best-effort convenience feature
        print(f"[INFO] Could not open '{window_name}' viewport: {exc}")
        return None


def _open_ego_cam_preview(robot_prim_path: str) -> None:
    _open_pov_viewport(f"{robot_prim_path}/base_link/ego_cam", "Ego Cam Preview")
    _open_pov_viewport(f"{robot_prim_path}/link6l/wrist_cam", "Wrist Cam Preview")


def _attach_rsd455_camera(parent_prim_path: str, mount_name: str, translate: tuple, orient_wxyz: tuple) -> str:
    stage = omni.usd.get_context().get_stage()
    mount_path = f"{parent_prim_path}/{mount_name}"
    prim = UsdGeom.Xform.Define(stage, mount_path)
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(*translate))
    w, x, y, z = orient_wxyz
    orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    rsd_prim = stage.DefinePrim(f"{mount_path}/rsd455")
    rsd_prim.GetPayloads().AddPayload(_RSD455_USD_URL)
    return mount_path


def _open_stereo_pov_preview(robot_prim_path: str) -> None:
    base_link_path = f"{robot_prim_path}/base_link"
    eye_local_right = _EYE_LOCAL_RIGHT
    from isaaclab.utils.math import quat_apply

    head_quat = torch.tensor([_HEAD_VIEWPOINT_HOME_QUAT], dtype=torch.float32)
    head_pos = torch.tensor([_HEAD_VIEWPOINT_HOME_POS], dtype=torch.float32)
    right_offset = quat_apply(head_quat, eye_local_right.unsqueeze(0)) * (_EYE_IPD_M / 2)
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

    _open_ego_cam_preview("/World/envs/env_0/Robot")
    _open_stereo_pov_preview("/World/envs/env_0/Robot")

    driven_arm_names = [resolve_joint_name(robot, name) for name in RIGHT_ARM_JOINTS]
    driven_gripper_names = [resolve_joint_name(robot, name) for name in RIGHT_GRIPPER_JOINTS]
    held_arm_names = [resolve_joint_name(robot, name) for name in LEFT_ARM_JOINTS[:6]]

    print(f"[INFO] Robot joints: {robot.data.joint_names}")
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

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in RIGHT_GRIPPER_JOINTS]], device=sim.device,
    )
    gripper_closed_targets = torch.tensor(
        [[GRIPPER_CLOSED[name] for name in RIGHT_GRIPPER_JOINTS]], device=sim.device,
    )

    # NOTE: this installed IsaacLab version (0.54.2) wraps sensitivity in a
    # Se3KeyboardCfg dataclass instead of accepting pos_sensitivity/
    # rot_sensitivity as direct __init__ kwargs (the bimanual
    # keyboard_teleop.py this was adapted from uses the older direct-kwarg
    # form and would hit the same TypeError on this IsaacLab version).
    teleop = Se3Keyboard(Se3KeyboardCfg(pos_sensitivity=0.005, rot_sensitivity=0.05))
    should_reset = False

    def reset_arm():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_arm)
    teleop.reset()
    print(teleop)
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

        # This IsaacLab version's Se3Keyboard.advance() returns a single
        # 7-element tensor [dx,dy,dz,rx,ry,rz,gripper] instead of a
        # (delta_pose, close_gripper) tuple (another API drift from what
        # keyboard_teleop.py assumes) -- gripper is +1.0 open / -1.0 close.
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

        # Relative mode: target = current_tip + delta (stays close, no runaway).
        diff_ik_controller.set_command(command, ee_pos=tip_pos_b, ee_quat=tip_quat_b)

        jacobian_w = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, driven_arm_ids]
        jacobian_b = compute_tip_ik_jacobian(robot, jacobian_w, wrist_pos_b, tip_pos_b)
        joint_pos_des = diff_ik_controller.compute(tip_pos_b, tip_quat_b, jacobian_b, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=driven_arm_ids)

        gripper_targets = gripper_closed_targets if close_gripper else gripper_open_targets
        zero_gripper_vel = torch.zeros(1, len(driven_gripper_ids), device=sim.device)
        robot.set_joint_position_target(gripper_targets, joint_ids=driven_gripper_ids)
        robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=driven_gripper_ids)

        # Keep the other arm fixed at default pose (including its gripper).
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
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 1.5])

    scene_cfg = ArmV2SceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete. Use keyboard to teleoperate the arm.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
