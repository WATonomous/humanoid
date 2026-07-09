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

# Import bimanual_arm_cfg from keyboard teleoperation (same robot model as keyboard_teleop.py)
_KEYBOARD_TELEOP_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../Teleop/keyboard_based_teleoperation")
)
sys.path.insert(0, _KEYBOARD_TELEOP_DIR)

from bimanual_arm_cfg import (  # noqa: E402
    BIMANUAL_ARM_CFG,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_FINGER_TIP_BODIES,
    LEFT_GRIPPER_JOINTS,
    RIGHT_ARM_JOINTS,
    apply_joint_limits,
    compute_tip_ik_jacobian,
    compute_gripper_tip_pose_b,
    compute_gripper_tip_pose_w,
    resolve_body_ids,
    resolve_joint_name,
)
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.managers import SceneEntityCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers  # noqa: E402
from isaaclab.markers.config import FRAME_MARKER_CFG  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import subtract_frame_transforms  # noqa: E402
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

    left_arm_names = [resolve_joint_name(robot, name) for name in LEFT_ARM_JOINTS]
    print(f"[INFO] Robot joints: {robot.data.joint_names}")
    print(f"[INFO] Left arm IK joints: {left_arm_names}")
    print(f"[INFO] Left wrist body (Jacobian anchor): {LEFT_EE_BODY}")
    print("[INFO] IK tracks fingertip center (link7l/link8l mesh distal midpoint)")

    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="position", use_relative_mode=False, ik_method="dls"
    )
    diff_ik_controller = DifferentialIKController(
        diff_ik_cfg, num_envs=scene.num_envs, device=sim.device
    )

    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    robot_entity_cfg = SceneEntityCfg(
        "robot", joint_names=left_arm_names, body_names=[LEFT_EE_BODY]
    )
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = (
        robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]
    )
    wrist_body_id = robot_entity_cfg.body_ids[0]
    finger_body_ids = resolve_body_ids(robot, LEFT_FINGER_TIP_BODIES)

    left_arm_ids = robot_entity_cfg.joint_ids
    left_gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    right_joint_ids = _joint_ids(robot, RIGHT_ARM_JOINTS)
    right_gripper_ids = _joint_ids(robot, ["joint7", "joint8"])
    right_default_pos = robot.data.default_joint_pos[:, right_joint_ids].clone()

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in LEFT_GRIPPER_JOINTS]],
        device=sim.device,
    )

    joint_position = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_position, joint_vel)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    diff_ik_controller.reset(env_ids=torch.arange(scene.num_envs, device=sim.device))

    while simulation_app.is_running():
        cube_pos_w, cube_quat_w = scene["cube"].get_world_poses()
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

        # set_command needs ee_quat to hold current orientation (used for display only in position mode)
        diff_ik_controller.set_command(cube_pos_b, ee_quat=tip_quat_b)

        jacobian = compute_tip_ik_jacobian(
            robot,
            robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, left_arm_ids],
            ee_pos_b,
            tip_pos_b,
        )

        joint_pos_des = diff_ik_controller.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=left_arm_ids)

        # Hold left gripper open; right arm + gripper at default pose
        robot.set_joint_position_target(gripper_open_targets, joint_ids=left_gripper_ids)
        robot.set_joint_position_target(right_default_pos, joint_ids=right_joint_ids)
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
