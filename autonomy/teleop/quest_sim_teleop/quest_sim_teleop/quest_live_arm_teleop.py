import argparse
import time

import numpy as np
import rclpy
from quest_teleop.msg import QuestHandPose
from rclpy.node import Node

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Quest live teleop for Wato arms")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG, LEFT_ARM_CFG
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

WEBXR_JOINTS = [
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
]

JOINT_LIMITS = {
    "mcp_index": (-1.570796, 0.0),
    "pip_index": (0.0, 1.570796),
    "dip_index": (-1.570796, 0.0),
    "mcp_middle": (-1.570796, 0.0),
    "pip_middle": (0.0, 1.570796),
    "dip_middle": (0.0, 1.570796),
    "mcp_ring": (0.0, 1.570796),
    "pip_ring": (-1.570796, 0.0),
    "dip_ring": (-1.570796, 0.0),
    "mcp_pinky": (0.0, 1.570796),
    "pip_pinky": (-1.570796, 0.0),
    "dip_pinky": (0.0, 1.570796),
    "cmc_thumb": (-0.349066, 2.094395),
    "mcp_thumb": (0.785398, 2.530727),
    "ip_thumb": (0.0, 1.570796),
}


@configclass
class QuestBothArmsSceneCfg(InteractiveSceneCfg):
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
            pos=(0.0, -0.35, 0.0),
            rot=(0.7071068, 0.0, 0.0, -0.7071068),
            joint_pos=ARM_CFG.init_state.joint_pos,
        ),
    )

    left_arm = LEFT_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/LeftArm",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.35, 0.0),
            rot=(0.7071068, 0.0, 0.0, 0.7071068),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


class QuestTeleopSubscriber(Node):
    def __init__(self):
        super().__init__("quest_live_arm_teleop")
        self.latest_msg = None
        self.last_msg_time = None
        self.create_subscription(QuestHandPose, "/quest_teleop", self._on_msg, 1)

    def _on_msg(self, msg):
        self.latest_msg = msg
        self.last_msg_time = time.monotonic()


def hand_array_to_points(values):
    if len(values) != 75:
        return None

    points = {}
    for i, name in enumerate(WEBXR_JOINTS):
        j = 3 * i
        points[name] = np.array(values[j:j + 3], dtype=np.float32)
    return points


def vector_angle(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm < 1e-6 or b_norm < 1e-6:
        return 0.0

    a = a / a_norm
    b = b / b_norm
    return float(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0)))


def finger_angles(points, finger):
    metacarpal = points[f"{finger}-finger-metacarpal"]
    proximal = points[f"{finger}-finger-phalanx-proximal"]
    intermediate = points[f"{finger}-finger-phalanx-intermediate"]
    distal = points[f"{finger}-finger-phalanx-distal"]
    tip = points[f"{finger}-finger-tip"]

    return (
        vector_angle(proximal - metacarpal, intermediate - proximal),
        vector_angle(intermediate - proximal, distal - intermediate),
        vector_angle(distal - intermediate, tip - distal),
    )


def thumb_angles(points):
    wrist = points["wrist"]
    metacarpal = points["thumb-metacarpal"]
    proximal = points["thumb-phalanx-proximal"]
    distal = points["thumb-phalanx-distal"]
    tip = points["thumb-tip"]

    return (
        vector_angle(metacarpal - wrist, proximal - metacarpal),
        vector_angle(proximal - metacarpal, distal - proximal),
        vector_angle(distal - proximal, tip - distal),
    )


def map_curl_to_joint(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]
    curl = float(np.clip(curl, 0.0, 1.570796))
    value = -curl if upper <= 0.0 else curl
    return float(np.clip(value, lower, upper))


def map_thumb_mcp(curl, joint_name):
    base_name = joint_name.removeprefix("left_")
    lower, upper = JOINT_LIMITS[base_name]
    return float(np.clip(lower + float(np.clip(curl, 0.0, 1.570796)), lower, upper))


def compute_hand_targets(points, side):
    prefix = "left_" if side == "left" else ""
    targets = {}

    for finger in ["index", "middle", "ring", "pinky"]:
        mcp, pip, dip = finger_angles(points, finger)
        targets[f"{prefix}mcp_{finger}"] = map_curl_to_joint(mcp, f"{prefix}mcp_{finger}")
        targets[f"{prefix}pip_{finger}"] = map_curl_to_joint(pip, f"{prefix}pip_{finger}")
        targets[f"{prefix}dip_{finger}"] = map_curl_to_joint(dip, f"{prefix}dip_{finger}")

    cmc, mcp, ip = thumb_angles(points)
    targets[f"{prefix}cmc_thumb"] = map_curl_to_joint(cmc, f"{prefix}cmc_thumb")
    targets[f"{prefix}mcp_thumb"] = map_thumb_mcp(mcp, f"{prefix}mcp_thumb")
    targets[f"{prefix}ip_thumb"] = map_curl_to_joint(ip, f"{prefix}ip_thumb")
    return targets


def apply_targets(target_tensor, robot, target_dict):
    for joint_name, value in target_dict.items():
        target_tensor[0, robot.joint_names.index(joint_name)] = value


def run_simulator(sim, scene, node):
    right_arm = scene["right_arm"]
    left_arm = scene["left_arm"]

    right_cfg = SceneEntityCfg("right_arm", joint_names=[".*"], body_names=[".*"])
    left_cfg = SceneEntityCfg("left_arm", joint_names=[".*"], body_names=[".*"])
    right_cfg.resolve(scene)
    left_cfg.resolve(scene)

    right_target = right_arm.data.default_joint_pos.clone()
    left_target = left_arm.data.default_joint_pos.clone()
    sim_dt = sim.get_physics_dt()

    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)

        if node.latest_msg is not None and time.monotonic() - node.last_msg_time < 0.25:
            right_points = hand_array_to_points(node.latest_msg.right_hand_joints)
            left_points = hand_array_to_points(node.latest_msg.left_hand_joints)

            if right_points is not None:
                apply_targets(right_target, right_arm, compute_hand_targets(right_points, "right"))
            if left_points is not None:
                apply_targets(left_target, left_arm, compute_hand_targets(left_points, "left"))

        right_arm.set_joint_position_target(right_target, joint_ids=right_cfg.joint_ids)
        left_arm.set_joint_position_target(left_target, joint_ids=left_cfg.joint_ids)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    rclpy.init()
    node = QuestTeleopSubscriber()

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.0])

    scene_cfg = QuestBothArmsSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Quest live arm teleop ready. Waiting for /quest_teleop...")

    try:
        run_simulator(sim, scene, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
