import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

SCRIPT_DIR = Path(__file__).resolve().parent
SIMULATION_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(SIMULATION_DIR / "Humanoid_Wato"))

parser = argparse.ArgumentParser(description="Quest hand teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import rclpy
from rclpy.node import Node
from common_msgs.msg import QuestHandPose
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG, LEFT_ARM_CFG

from quest_isaac_teleop.quest_calc import compute_all_targets


LEFT_FINGER_JOINTS = [
    "left_mcp_index", "left_pip_index", "left_dip_index",
    "left_mcp_middle", "left_pip_middle", "left_dip_middle",
    "left_mcp_ring", "left_pip_ring", "left_dip_ring",
    "left_mcp_pinky", "left_pip_pinky", "left_dip_pinky",
    "left_cmc_thumb", "left_mcp_thumb", "left_ip_thumb",
]

RIGHT_FINGER_JOINTS = [
    "mcp_index", "pip_index", "dip_index",
    "mcp_middle", "pip_middle", "dip_middle",
    "mcp_ring", "pip_ring", "dip_ring",
    "mcp_pinky", "pip_pinky", "dip_pinky",
    "cmc_thumb", "mcp_thumb", "ip_thumb",
]


class QuestListener(Node):
    def __init__(self):
        super().__init__("quest_listener")
        self.latest = None
        self.message_count = 0
        self.create_subscription(QuestHandPose, "/quest_teleop", self.cb, 1)

    def cb(self, msg):
        self.latest = msg
        self.message_count += 1


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    right_arm = ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/RightArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=ARM_CFG.init_state.joint_pos,
        ),
    )
    left_arm = LEFT_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/LeftArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.5, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


def apply_hand_targets(arm, joint_indices, joint_names, hand_joints, side, logger):
    if len(hand_joints) != 75:
        logger.warn(f"Expected 75 {side or 'right'} hand values, got {len(hand_joints)}")
        return

    targets = compute_all_targets(hand_joints, side)
    target_positions = arm.data.joint_pos[:, joint_indices].clone()
    for target_idx, joint_name in enumerate(joint_names):
        target_positions[0, target_idx] = targets[joint_name]
    arm.set_joint_position_target(target_positions, joint_ids=joint_indices)


def run_simulator(sim, scene, listener):
    sim_dt = sim.get_physics_dt()
    left_arm = scene["left_arm"]
    right_arm = scene["right_arm"]

    left_joint_names = left_arm.data.joint_names
    left_joint_indices = [left_joint_names.index(j) for j in LEFT_FINGER_JOINTS]
    right_joint_names = right_arm.data.joint_names
    right_joint_indices = [right_joint_names.index(j) for j in RIGHT_FINGER_JOINTS]

    print(f"[INFO]: Controlling left joints: {LEFT_FINGER_JOINTS}")
    print(f"[INFO]: Controlling right joints: {RIGHT_FINGER_JOINTS}")

    while simulation_app.is_running():
        rclpy.spin_once(listener, timeout_sec=0)

        if listener.latest is not None:
            apply_hand_targets(
                left_arm,
                left_joint_indices,
                LEFT_FINGER_JOINTS,
                list(listener.latest.left_hand_joints),
                "left",
                listener.get_logger(),
            )
            apply_hand_targets(
                right_arm,
                right_joint_indices,
                RIGHT_FINGER_JOINTS,
                list(listener.latest.right_hand_joints),
                "",
                listener.get_logger(),
            )
            listener.latest = None

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    rclpy.init()
    listener = QuestListener()

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.0])

    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()

    print("[INFO]: Quest teleop ready")
    run_simulator(sim, scene, listener)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
