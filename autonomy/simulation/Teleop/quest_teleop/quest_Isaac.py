import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Quest hand teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import rclpy
from rclpy.node import Node
from quest_teleop.msg import QuestHandPose

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import LEFT_ARM_CFG
from quest_calc import compute_all_targets


FINGER_JOINTS = [
    "left_mcp_index", "left_pip_index", "left_dip_index",
    "left_mcp_middle", "left_pip_middle", "left_dip_middle",
    "left_mcp_ring", "left_pip_ring", "left_dip_ring",
    "left_mcp_pinky", "left_pip_pinky", "left_dip_pinky",
    "left_cmc_thumb", "left_mcp_thumb", "left_ip_thumb",
]


class QuestListener(Node):
    def __init__(self):
        super().__init__("quest_listener")
        self.latest = None
        self.create_subscription(QuestHandPose, "/quest_teleop", self.cb, 1)

    def cb(self, msg):
        self.latest = msg


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
    left_arm = LEFT_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/LeftArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


def run_simulator(sim, scene, listener):
    sim_dt = sim.get_physics_dt()
    arm = scene["left_arm"]
    joint_names = arm.data.joint_names
    joint_indices = [joint_names.index(j) for j in FINGER_JOINTS]

    while simulation_app.is_running():
        rclpy.spin_once(listener, timeout_sec=0)

        if listener.latest is not None:
            targets = compute_all_targets(list(listener.latest.left_hand_joints), "left")
            positions = arm.data.joint_pos.clone()
            for i, name in zip(joint_indices, FINGER_JOINTS):
                positions[0, i] = targets[name]
            arm.set_joint_position_target(positions)
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
