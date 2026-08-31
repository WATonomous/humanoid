"""
Task-space IK demo for the WATonomous bimanual arm (left arm only).

A cube in the scene is the absolute gripper-tip pose target. Move the cube in
the viewport and the left arm (6 revolute joints) follows via Differential IK.
The right arm and both grippers are held at their default poses.
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Bimanual arm task-space IK (cube target, left arm only)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# pioneer_humanoid package (canonical arm config). Editable-installed in the image; this fallback
# keeps a bare bind-mounted checkout working.
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../pioneer_humanoid")
))

# This script drives the L-suffixed chain (physical LEFT arm), which the canonical config
# names LEFT_*; aliased to RIGHT_* here so the body's "RIGHT_* = the arm we drive" is unchanged.
from pioneer_humanoid.bimanual_arm import (  # noqa: E402
    BIMANUAL_ARM_CFG,
    LEFT_GRIPPER_OPEN as GRIPPER_OPEN,
    LEFT_ARM_JOINTS as RIGHT_ARM_JOINTS,
    LEFT_EE_BODY as RIGHT_EE_BODY,
    LEFT_FINGER_TIP_BODIES as RIGHT_FINGER_TIP_BODIES,
    LEFT_GRIPPER_JOINTS as RIGHT_GRIPPER_JOINTS,
    RIGHT_ARM_JOINTS as LEFT_ARM_JOINTS,
    apply_joint_limits,
    compute_tip_ik_jacobian,
    compute_gripper_tip_pose_b,
    compute_gripper_tip_pose_w,
    resolve_body_ids,
    resolve_joint_name,
)
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers  # noqa: E402
from isaaclab.markers.config import FRAME_MARKER_CFG  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_inv, quat_mul, subtract_frame_transforms  # noqa: E402
import torch  # noqa: E402


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    cube = AssetBaseCfg(
        prim_path="/World/cube",
        spawn=sim_utils.CuboidCfg(size=[0.1, 0.1, 0.1]),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.26, -0.23, 0.15)),
    )

    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()

    scene.update(sim_dt)
    apply_joint_limits(robot)

    right_arm_names = [resolve_joint_name(robot, name) for name in RIGHT_ARM_JOINTS]
    print(f"[INFO] Robot joints: {robot.data.joint_names}")
    print(f"[INFO] Left arm IK joints: {right_arm_names}")
    print(f"[INFO] Left wrist body (Jacobian anchor): {RIGHT_EE_BODY}")
    print("[INFO] IK tracks fingertip center (link7l/link8l mesh distal midpoint)")

    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="pose", use_relative_mode=False, ik_method="dls"
    )
    diff_ik_controller = DifferentialIKController(
        diff_ik_cfg, num_envs=scene.num_envs, device=sim.device
    )

    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    robot_entity_cfg = SceneEntityCfg(
        "robot", joint_names=right_arm_names, body_names=[RIGHT_EE_BODY]
    )
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = (
        robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    )
    wrist_body_id = robot_entity_cfg.body_ids[0]
    finger_body_ids = resolve_body_ids(robot, RIGHT_FINGER_TIP_BODIES)

    left_arm_ids = robot_entity_cfg.joint_ids
    right_gripper_ids = _joint_ids(robot, RIGHT_GRIPPER_JOINTS)
    left_joint_ids = _joint_ids(robot, LEFT_ARM_JOINTS)
    right_gripper_ids = _joint_ids(robot, ["joint7", "joint8"])
    left_default_pos = robot.data.default_joint_pos[:, left_joint_ids].clone()

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in RIGHT_GRIPPER_JOINTS]],
        device=sim.device,
    )

    joint_position = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_position, joint_vel)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    diff_ik_controller.reset(env_ids=torch.arange(scene.num_envs, device=sim.device))

    # Read the cube from its USD transform, not the physics tensor -- an AssetBaseCfg prim's
    # tensor pose doesn't update on a viewport drag.
    _cube_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/cube")
    _cube_xf = UsdGeom.Xformable(_cube_prim)

    def _cube_pose_w():
        m = _cube_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        im = q.GetImaginary()
        pos = torch.tensor([[t[0], t[1], t[2]]], dtype=torch.float32, device=sim.device)
        quat = torch.tensor([[q.GetReal(), im[0], im[1], im[2]]], dtype=torch.float32, device=sim.device)
        return pos, quat

    # Co-locate the cube with the gripper at launch (position only; orientation stays identity).
    _tp_w, _ = compute_gripper_tip_pose_w(robot, wrist_body_id, finger_body_ids)
    _p = _tp_w[0].tolist()
    _cube_xf.ClearXformOpOrder()
    _cube_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(_p[0], _p[1], _p[2]))
    sim.step()
    scene.update(sim_dt)

    # link6l's frame isn't identity, so the IK target is (cube rotation since launch) applied
    # to the gripper's rest orientation -- cube at rest -> gripper at rest, and a world-axis
    # cube rotation is a world-axis gripper rotation.
    _root0 = robot.data.root_state_w[:, 0:7]
    _cp0_w, _cq0_w = _cube_pose_w()
    _, _cube_q0_b = subtract_frame_transforms(_root0[:, 0:3], _root0[:, 3:7], _cp0_w, _cq0_w)
    _, _wrist_q0_b = compute_gripper_tip_pose_b(robot, _root0, wrist_body_id, finger_body_ids)
    _cube_q0_b_inv = quat_inv(_cube_q0_b)

    while simulation_app.is_running():
        cube_pos_w, cube_quat_w = _cube_pose_w()
        root_pose_w = robot.data.root_state_w[:, 0:7]
        cube_pos_b, cube_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], cube_pos_w, cube_quat_w
        )

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        joint_pos = robot.data.joint_pos[:, left_arm_ids]

        ee_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, wrist_body_id, finger_body_ids
        )

        d_cube_b = quat_mul(cube_quat_b, _cube_q0_b_inv)
        target_quat_b = quat_mul(d_cube_b, _wrist_q0_b)
        diff_ik_controller.set_command(
            torch.cat([cube_pos_b, target_quat_b], dim=-1), ee_quat=tip_quat_b
        )

        jacobian = compute_tip_ik_jacobian(
            robot,
            robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, left_arm_ids],
            ee_pos_b,
            tip_pos_b,
        )

        joint_pos_des = diff_ik_controller.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=left_arm_ids)

        # Hold left gripper open; right arm + gripper at default pose
        robot.set_joint_position_target(gripper_open_targets, joint_ids=right_gripper_ids)
        robot.set_joint_position_target(left_default_pos, joint_ids=left_joint_ids)
        robot.set_joint_velocity_target(
            torch.zeros(1, len(right_gripper_ids), device=sim.device),
            joint_ids=right_gripper_ids,
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        tip_pos_w, tip_quat_w = compute_gripper_tip_pose_w(robot, wrist_body_id, finger_body_ids)
        ee_marker.visualize(tip_pos_w, tip_quat_w)
        goal_marker.visualize(cube_pos_w, cube_quat_w)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene_cfg = TableTopSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    print("[INFO]: Setup complete...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
