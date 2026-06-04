"""
task_space_test.py
==================
Runs a simulation where the 6-DOF Wato humanoid arm is controlled in task space.
A virtual target cube is spawned in the scene. The robot arm uses a Differential Inverse Kinematics
controller to calculate target joint angles so its end effector follows the target cube around in real-time.

Process:
  1. Initialize the simulation launcher and resolve the path to the modelCfg directory
  2. Load the arm-only configuration (humanoid_arm_only) and construct the scene (ground, lights, table, target cube)
  3. Instantiate a Differential IK Controller
  4. Run the physics loop:
     a. Read the target cube's current position and orientation in space
     b. Compute the required joint angles using the Jacobian matrix and IK solver
     c. Command the robot's arm joints to move toward the computed angles
"""

import argparse
import sys
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Robot Arm with Task Space IK Control")
parser.add_argument("--robot", type=str, default="ur10", help="Name of the robot.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# All Isaac Sim / Isaac Lab imports must come after SimulationApp is instantiated
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),
                                             "../../Humanoid_Wato/HumanoidRL/HumanoidRLPackage/HumanoidRLSetup/modelCfg")))

from humanoid_arm_only import ARM_CFG
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms
import torch


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(
                0.75,
                0.75,
                0.75)))

    # Setting the initial state here is important because at certain initial states,
    # mainly on the positive x-axis, the IK breaks and the simulation bugs out a lot
    cube = AssetBaseCfg(
        prim_path="/World/cube",
        spawn=sim_utils.CuboidCfg(
            size=[0.1, 0.1, 0.1]
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.3, 0.0, 0.3)),
    )

    robot = ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):

    robot = scene["robot"]

    diff_ik_cfg = DifferentialIKControllerCfg(
        command_type="pose", use_relative_mode=False, ik_method="dls")
    diff_ik_controller = DifferentialIKController(
        diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(
        frame_marker_cfg.replace(prim_path="/Visuals/ee_current")
    )
    goal_marker = VisualizationMarkers(
        frame_marker_cfg.replace(prim_path="/Visuals/ee_goal")
    )

    robot_entity_cfg = SceneEntityCfg(
        "robot",
        joint_names=["shoulder_flexion_extension", "shoulder_abduction_adduction",
                     "shoulder_rotation", "elbow_flexion_extension",
                     "forearm_rotation", "wrist_extension"],
        body_names=["PALM_GAVIN_1DoF_Hinge_v2_1"]
    )

    robot_entity_cfg.resolve(scene)

    # Obtain the frame index of the end-effector ; For a fixed base robot, the
    # frame index is one less than the body index. This is because ; the root
    # body is not included in the returned Jacobians.
    if robot.is_fixed_base:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1
    else:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0]

    sim_dt = sim.get_physics_dt()

    joint_position = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_position, joint_vel)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    diff_ik_controller.reset(env_ids=torch.arange(scene.num_envs, device=sim.device))

    while simulation_app.is_running():
        position, quaternion = scene["cube"].get_world_poses()
        root_pose_w = robot.data.root_state_w[:, 0:7]
        cube_pos_b, cube_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], position, quaternion
        )
        ik_commands = torch.cat([cube_pos_b, cube_quat_b], dim=1)

        diff_ik_controller.set_command(ik_commands)

        jacobian = robot.root_physx_view.get_jacobians()[
            :, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
        ee_pose_w = robot.data.body_state_w[:,
                                            robot_entity_cfg.body_ids[0], 0:7]
        joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]

        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )

        joint_pos_des = diff_ik_controller.compute(
            ee_pos_b, ee_quat_b, jacobian, joint_pos)

        # apply actions (Smooth movement)
        robot.set_joint_position_target(
            joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()

        sim.step()

        scene.update(sim_dt)

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]

        ee_marker.visualize(ee_pose_w[:, 0:3], ee_pose_w[:, 3:7])
        goal_marker.visualize(
            ik_commands[:, 0:3] + scene.env_origins, ik_commands[:, 3:7])


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    scene_cfg = TableTopSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()

    print("[INFO]: Setup complete...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
