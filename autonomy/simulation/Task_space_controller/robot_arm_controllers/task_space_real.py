"""
Task-space IK demo for the WATonomous bimanual arm (left arm only), with sim-to-real.

A cube in the scene is the absolute gripper-tip pose target. Move the cube in
the viewport and the left arm (6 revolute joints) follows via Differential IK.
The right arm and both grippers are held at their default poses.

Pass --ros to publish left-arm joint targets to /behaviour/arm_pose every 20ms for
sim-to-real. Pass --udp to send them over UDP to ros_bridge.py instead (use when the
Isaac Lab env's Python can't import rclpy).

Sim-to-real ROS2 architecture:
    task_space_real.py ──► /behaviour/arm_pose ──► joint_command_node ──► /interfacing/motorCMD ──► can_node ──► AK motors (0x0A-0x0E, POSITION_LOOP frames)

joint_command_node consumes /behaviour/arm_pose and drives the 5 wired AK arm joints
through the safety layer (seed-from-feedback + velocity/delta/low-pass; see
autonomy/behaviour/joint_command). The 6th slot (wrist, 0x16) is currently unwired and
ignored.
"""

import argparse
import os
import sys
import threading

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Bimanual arm task-space IK (cube target, left arm only)")
parser.add_argument("--ros", action="store_true", help="Publish left-arm joint targets to ROS2.")
parser.add_argument("--udp", action="store_true", help="Send joint angles over UDP to ros_bridge.py (use when ROS Python version mismatches Isaac Lab env).")
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
    RIGHT_ARM_JOINTS,
    RIGHT_EE_BODY,
    RIGHT_FINGER_TIP_BODIES,
    RIGHT_GRIPPER_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_GRIPPER_JOINTS,
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

# ---------------------------------------------------------------------------
# ROS2 / UDP setup — only initialized when --ros / --udp is passed so the sim
# still runs standalone without a ROS2 environment.
# Requires: source your ROS2 workspace that has common_msgs built.
# ---------------------------------------------------------------------------
ros_pub = None
ros_node = None
udp_sock = None

UDP_HOST = "127.0.0.1"
UDP_PORT = 5005

# Sim-to-real publish period (matches joint_command_node's expected command rate).
PUBLISH_PERIOD = 0.02  # seconds (20 ms)

# Grace period before any command is sent to real hardware. joint_command_node applies
# no velocity/delta rate-limiting to the very first ArmPose message it ever receives, so
# this gives you time to manually position the real arm near the sim's starting pose
# before that unramped first command goes out.
PUBLISH_START_DELAY = 5.0  # seconds


def init_udp():
    global udp_sock
    import socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[UDP] Sending joint angles to {UDP_HOST}:{UDP_PORT}")


def init_ros():
    global ros_pub, ros_node
    import rclpy
    from rclpy.node import Node
    from common_msgs.msg import ArmPose

    rclpy.init()
    ros_node = Node("isaac_sim_arm_publisher")
    ros_pub = ros_node.create_publisher(ArmPose, "/behaviour/arm_pose", 10)

    # Spin ROS2 in a background thread so it doesn't block the sim loop
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    print("[ROS2] Publisher ready on /behaviour/arm_pose")


def publish_joint_pos(joint_pos_des):
    """Pack the 6 left-arm IK joint targets into ArmPose and publish.

    joint_pos_des is in radians (Isaac Lab convention).
    hardware_mapping.yaml limits and PositionDeg CAN signal expect degrees,
    so convert here before publishing.
    """
    import math
    from common_msgs.msg import ArmPose, JointState as WatoJointState

    # Convert radians -> degrees for the CAN motor controller
    q = [math.degrees(v) for v in joint_pos_des[0].tolist()]

    msg = ArmPose()
    msg.header.stamp = ros_node.get_clock().now().to_msg()

    shoulder = WatoJointState()
    shoulder.position = [q[0], q[1], q[2]]   # flexion, abduction, rotation
    msg.shoulder = shoulder

    elbow = WatoJointState()
    elbow.position = [q[3], q[4]]             # flexion, forearm_rotation
    msg.elbow = elbow

    wrist = WatoJointState()
    wrist.position = [q[5]]                   # wrist_extension
    msg.wrist = wrist

    ros_pub.publish(msg)


def publish_joint_pos_udp(joint_pos_des):
    import math
    import struct
    q = [math.degrees(v) for v in joint_pos_des[0].tolist()]
    data = struct.pack("6d", *q)
    udp_sock.sendto(data, (UDP_HOST, UDP_PORT))


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

    # Spawned at the active/correct arm's (link7/link8, unsuffixed chain) actual measured
    # fingertip midpoint at its default pose -- the old value (0.26, -0.23, 0.15) was
    # ~0.31 units from the OTHER (passive, joint1L..6l) arm's resting hand but ~0.95 units
    # from this arm's, meaning the IK was starting by reaching almost a meter toward where
    # the wrong arm's hand rests. Drag the cube from here once the sim is running.
    cube = AssetBaseCfg(
        prim_path="/World/cube",
        spawn=sim_utils.CuboidCfg(size=[0.1, 0.1, 0.1]),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.24, 0.64, 0.53)),
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
    right_gripper_ids = _joint_ids(robot, RIGHT_GRIPPER_JOINTS)

    gripper_open_targets = torch.tensor(
        [[GRIPPER_OPEN[name] for name in RIGHT_GRIPPER_JOINTS]],
        device=sim.device,
    )

    joint_position = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_position, joint_vel)
    left_default_pos = joint_position[:, left_joint_ids].clone()
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim_dt)

    diff_ik_controller.reset(env_ids=torch.arange(scene.num_envs, device=sim.device))

    # Sim-to-real: accumulate sim time and publish joint targets every 20ms,
    # independent of the physics step size.
    time_since_publish = 0.0
    elapsed_time = 0.0
    started_publishing = False
    if ros_pub is not None or udp_sock is not None:
        print(f"[INFO] Waiting {PUBLISH_START_DELAY:.0f}s before publishing to real "
              f"hardware -- position the real arm near the sim's starting pose now.")

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
        robot.set_joint_position_target(gripper_open_targets, joint_ids=right_gripper_ids)
        robot.set_joint_position_target(left_default_pos, joint_ids=left_joint_ids)
        robot.set_joint_velocity_target(
            torch.zeros(1, len(right_gripper_ids), device=sim.device),
            joint_ids=right_gripper_ids,
        )

        # Send desired left-arm joint targets to the real arm via ROS2 or UDP bridge,
        # throttled to PUBLISH_PERIOD (20ms) regardless of the physics step size, and
        # held off entirely for PUBLISH_START_DELAY seconds after startup. time_since_publish
        # only starts accumulating once the delay has passed, so it can't build up a backlog
        # during the hold and then burst-publish to "catch up" once it ends.
        elapsed_time += sim_dt
        if elapsed_time >= PUBLISH_START_DELAY:
            time_since_publish += sim_dt
        if elapsed_time >= PUBLISH_START_DELAY and time_since_publish >= PUBLISH_PERIOD:
            time_since_publish -= PUBLISH_PERIOD
            if not started_publishing and (ros_pub is not None or udp_sock is not None):
                started_publishing = True
                print("[INFO] Publishing to real hardware now.")
            if ros_pub is not None:
                publish_joint_pos(joint_pos_des)
            elif udp_sock is not None:
                publish_joint_pos_udp(joint_pos_des)

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
    if args_cli.ros:
        init_ros()
    elif args_cli.udp:
        init_udp()
    main()
    simulation_app.close()
    if ros_node is not None:
        import rclpy
        ros_node.destroy_node()
        rclpy.shutdown()
