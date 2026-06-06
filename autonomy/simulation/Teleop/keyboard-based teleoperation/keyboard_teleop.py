"""Minimal bimanual-arm keyboard teleoperation (left arm only).

Robot: Humanoid_Wato wato_bimanual_arm (bimanual_arm.usd)
Motor specs: https://watonomous.github.io/humanoid-docs/mechanical/index.html
Teleop bindings: https://isaac-sim.github.io/IsaacLab/v2.0.1/source/overview/teleop_imitation.html

  K       Toggle gripper (open/close)
  W/S     Move along x-axis
  A/D     Move along y-axis
  Q/E     Move along z-axis
  Z/X     Rotate along x-axis
  T/G     Rotate along y-axis
  C/V     Rotate along z-axis
  R       Reset left arm to default pose
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Keyboard teleoperation for the WATonomous bimanual arm (left only).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms

from bimanual_arm_cfg import (
    BIMANUAL_ARM_CFG,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_GRIPPER_JOINTS,
    RIGHT_ARM_JOINTS,
    apply_joint_limits,
    resolve_joint_name,
)


@configclass
class BimanualSceneCfg(InteractiveSceneCfg):
    """Minimal scene with the WATonomous bimanual arm."""

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


def _joint_ids(robot, names: list[str]) -> list[int]:
    name_to_id = {name: i for i, name in enumerate(robot.data.joint_names)}
    return [name_to_id[resolve_joint_name(robot, name)] for name in names]


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()

    # Ensure robot buffers are populated before reading limits / joint names
    scene.update(sim_dt)
    apply_joint_limits(robot)

    left_arm_names = [resolve_joint_name(robot, name) for name in LEFT_ARM_JOINTS]
    left_gripper_names = [resolve_joint_name(robot, name) for name in LEFT_GRIPPER_JOINTS]
    right_arm_names = [resolve_joint_name(robot, name) for name in RIGHT_ARM_JOINTS]

    print(f"[INFO] Robot joints: {robot.data.joint_names}")
    print(f"[INFO] Left arm joints: {left_arm_names}")
    print(f"[INFO] Right arm hold joints: {right_arm_names}")

    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    robot_entity_cfg = SceneEntityCfg("robot", joint_names=left_arm_names, body_names=[LEFT_EE_BODY])
    robot_entity_cfg.resolve(scene)

    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1 if robot.is_fixed_base else robot_entity_cfg.body_ids[0]

    left_arm_ids = robot_entity_cfg.joint_ids
    left_gripper_ids = _joint_ids(robot, LEFT_GRIPPER_JOINTS)
    right_joint_ids = _joint_ids(robot, RIGHT_ARM_JOINTS)
    right_gripper_ids = _joint_ids(robot, ["joint7", "joint8"])
    right_default_pos = robot.data.default_joint_pos[:, right_joint_ids].clone()

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in LEFT_GRIPPER_JOINTS]],
        device=sim.device,
    )
    gripper_closed_targets = torch.tensor(
        [[GRIPPER_CLOSED[name] for name in LEFT_GRIPPER_JOINTS]],
        device=sim.device,
    )

    teleop = Se3Keyboard(pos_sensitivity=0.15, rot_sensitivity=0.15)
    should_reset = False

    def reset_left_arm():
        nonlocal should_reset
        should_reset = True

    teleop.add_callback("R", reset_left_arm)
    teleop.reset()
    print(teleop)
    print("[INFO] Teleoperating left arm only. Right arm is held at default pose.")
    print("[INFO] Click the 3D viewport window, then press W/A/S/D/Q/E to move.")

    debug_steps = 0
    while simulation_app.is_running():
        if should_reset:
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            diff_ik_controller.reset()
            teleop.reset()
            should_reset = False

        delta_pose, close_gripper = teleop.advance()
        command = torch.tensor(delta_pose, dtype=torch.float32, device=sim.device).unsqueeze(0)

        if debug_steps < 5 and torch.any(command.abs() > 1e-4):
            print(f"[DEBUG] Keyboard command: {command[0].tolist()}")
            debug_steps += 1

        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, left_arm_ids]

        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )

        diff_ik_controller.set_command(command, ee_pos=ee_pos_b, ee_quat=ee_quat_b)

        jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, left_arm_ids]
        joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        robot.set_joint_position_target(joint_pos_des, joint_ids=left_arm_ids)

        # Hold gripper fingers at synchronized open/closed pair (one GL40 motor on hardware).
        # High stiffness in cfg + zero velocity target prevents bounce when the arm moves.
        gripper_targets = gripper_closed_targets if close_gripper else gripper_open_targets
        zero_gripper_vel = torch.zeros(1, len(left_gripper_ids), device=sim.device)
        robot.set_joint_position_target(gripper_targets, joint_ids=left_gripper_ids)
        robot.set_joint_velocity_target(zero_gripper_vel, joint_ids=left_gripper_ids)

        # Keep right arm fixed at default pose (including coupled gripper fingers)
        robot.set_joint_position_target(right_default_pos, joint_ids=right_joint_ids)
        robot.set_joint_velocity_target(
            torch.zeros(1, len(right_gripper_ids), device=sim.device),
            joint_ids=right_gripper_ids,
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene_cfg = BimanualSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete. Use keyboard to teleoperate the left arm.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
