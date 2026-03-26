"""
wato_hand_isaaclab_teleop.py
============================
Isaac Lab simulation that drives the arm_assembly hand joints in real-time
from the /wato/hand_joint_angles ROS2 topic published by wato_hand_ros2_node.py.

Usage (in the Docker container):
  # Terminal 1: rosbridge already running
  # Terminal 2: wato_hand_ros2_node.py already running
  # Terminal 3:
  cd /workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato
  PYTHONPATH=/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato \
    /home/hy/IsaacLab/isaaclab.sh -p wato_hand_isaaclab_teleop.py

  # To see the robot in VNC (port 5900):
  ... same command, without --headless
"""

import argparse
import json
import threading

from isaaclab.app import AppLauncher

# ── AppLauncher must come before any Isaac/USD imports ───────────────────────
parser = argparse.ArgumentParser(description="Wato Hand Isaac Lab Teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Now safe to import Isaac / torch / ROS2 ─────────────────────────────────
import torch                                        # noqa: E402

import isaaclab.sim as sim_utils                   # noqa: E402
from isaaclab.assets import AssetBaseCfg           # noqa: E402
from isaaclab.managers import SceneEntityCfg        # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass             # noqa: E402

import rclpy                                       # noqa: E402
from rclpy.node import Node                        # noqa: E402
from std_msgs.msg import String                    # noqa: E402

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG  # noqa: E402

# ── Joint name ordering (must match ARM_CFG / URDF) ─────────────────────────
# The arm has 6 DOF then 15 hand DOF.
ARM_JOINT_NAMES = [
    "shoulder_flexion_extension",
    "shoulder_abduction_adduction",
    "shoulder_rotation",
    "elbow_flexion_extension",
    "forearm_rotation",
    "wrist_extension",
]
HAND_JOINT_NAMES = [
    "mcp_index", "pip_index", "dip_index",
    "mcp_middle", "pip_middle", "dip_middle",
    "mcp_ring", "pip_ring", "dip_ring",
    "mcp_pinky", "pip_pinky", "dip_pinky",
    "cmc_thumb", "mcp_thumb", "ip_thumb",
]
ALL_JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES  # 21 DOF total


# ── Scene ────────────────────────────────────────────────────────────────────
@configclass
class ArmHandSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


# ── ROS2 subscriber node (runs in background thread) ─────────────────────────
class HandJointSubscriber(Node):
    """Lightweight ROS2 node that buffers the latest /wato/hand_joint_angles msg."""

    def __init__(self):
        super().__init__("wato_isaaclab_hand_teleop")
        self._lock = threading.Lock()
        self._latest: dict[str, float] = {}
        self.create_subscription(
            String, "/wato/hand_joint_angles", self._cb, 10
        )
        self.get_logger().info("Subscribed to /wato/hand_joint_angles")

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._latest = data
        except Exception as e:
            self.get_logger().error(f"Bad message: {e}")

    def get_latest(self) -> dict[str, float]:
        with self._lock:
            return dict(self._latest)


def _ros_spin_thread(node: HandJointSubscriber, stop_event: threading.Event):
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.005)


# ── Main sim loop ─────────────────────────────────────────────────────────────
def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    ros_node: HandJointSubscriber,
):
    robot = scene["robot"]
    robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
    robot_entity_cfg.resolve(scene)

    sim_dt = sim.get_physics_dt()

    # Initialise robot to default pose
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    # Build a mapping: joint_name → column index in the sim tensor
    sim_joint_names: list[str] = list(robot.data.joint_names)
    name_to_sim_idx: dict[str, int] = {name: i for i, name in enumerate(sim_joint_names)}

    # Persistent position target (1 env × 21 DOF)
    num_joints = len(sim_joint_names)
    joint_pos_target = robot.data.default_joint_pos.clone()  # shape [1, 21]

    print("[INFO]: Setup complete — waiting for hand data on /wato/hand_joint_angles ...")

    while simulation_app.is_running():
        # -- Fetch latest ROS2 hand angles --
        hand_dict = ros_node.get_latest()

        if hand_dict:
            for joint_name, angle in hand_dict.items():
                if joint_name in name_to_sim_idx:
                    col = name_to_sim_idx[joint_name]
                    joint_pos_target[0, col] = float(angle)

        # -- Apply to robot --
        robot.reset()
        joint_pos_des = joint_pos_target[:, robot_entity_cfg.joint_ids].clone()
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()

        sim.step()
        scene.update(sim_dt)


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    # Simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Scene
    scene_cfg = ArmHandSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    # ROS2
    rclpy.init()
    ros_node = HandJointSubscriber()
    stop_event = threading.Event()
    ros_thread = threading.Thread(target=_ros_spin_thread, args=(ros_node, stop_event), daemon=True)
    ros_thread.start()

    try:
        run_simulator(sim, scene, ros_node)
    finally:
        stop_event.set()
        ros_thread.join(timeout=2.0)
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
