"""
wato_hand_isaaclab_teleop.py
============================
Isaac Lab simulation that drives the arm_assembly hand joints in real-time
by reading /tmp/wato_joints.json written by wato_hand_ros2_node.py.

No rclpy required — works with isaaclab.sh's own Python interpreter.

Usage (in the Docker container):
  # Terminal 1: rosbridge already running
  # Terminal 2: wato_hand_ros2_node.py already running (writes /tmp/wato_joints.json)
  # Terminal 3:
  cd /workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato
  PYTHONPATH=/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato \
    /workspace/isaaclab/isaaclab.sh -p wato_hand_isaaclab_teleop.py
"""

import argparse
import json
import os
import time

from isaaclab.app import AppLauncher

# ── AppLauncher must come before any Isaac/USD imports ───────────────────────
parser = argparse.ArgumentParser(description="Wato Hand Isaac Lab Teleop")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Now safe to import Isaac / torch ─────────────────────────────────────────
import torch                                        # noqa: E402

import isaaclab.sim as sim_utils                   # noqa: E402
from isaaclab.assets import AssetBaseCfg           # noqa: E402
from isaaclab.managers import SceneEntityCfg       # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass             # noqa: E402

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import ARM_CFG  # noqa: E402

# ── Shared file written by wato_hand_ros2_node.py ────────────────────────────
JOINT_FILE = "/tmp/wato_joints.json"
HAND_TIMEOUT = 2.0  # seconds before hand is considered lost


def read_joint_file() -> dict[str, float]:
    """Read the latest joint angles. Returns {} if file missing/corrupt."""
    try:
        with open(JOINT_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def is_hand_visible(hand_dict: dict) -> bool:
    """True if the dict is non-empty and its timestamp is fresh."""
    if not hand_dict:
        return False
    ts = hand_dict.get("timestamp", 0.0)
    return (time.time() - ts) < HAND_TIMEOUT


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


# ── Main sim loop ─────────────────────────────────────────────────────────────
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
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

    # ── Wait for first valid camera frame and snap robot to that pose ───────────
    print("[INFO]: Waiting for first hand frame from camera to set start pose...")
    while True:
        hand_dict = read_joint_file()
        if hand_dict:
            break
        # Tick the sim while waiting so it stays alive
        sim.step()
        scene.update(sim_dt)

    # Build initial joint target from the first observed frame
    joint_pos_target = robot.data.default_joint_pos.clone()
    for joint_name, angle in hand_dict.items():
        if joint_name in name_to_sim_idx:
            joint_pos_target[0, name_to_sim_idx[joint_name]] = float(angle)

    # Teleport the robot directly to the start pose (no physics convergence delay)
    robot.write_joint_state_to_sim(joint_pos_target, robot.data.default_joint_vel.clone())
    print(f"[INFO]: Start pose set from camera. forearm_rotation = "
          f"{hand_dict.get('forearm_rotation', 0.0):.3f} rad, "
          f"wrist_extension = {hand_dict.get('wrist_extension', 0.0):.3f} rad")

    hand_visible_prev = True

    while simulation_app.is_running():
        hand_dict = read_joint_file()
        hand_visible = is_hand_visible(hand_dict)

        # -- Normal tracking: update target from latest data --
        if hand_visible:
            for joint_name, angle in hand_dict.items():
                if joint_name in ("timestamp",):
                    continue
                if joint_name in name_to_sim_idx:
                    joint_pos_target[0, name_to_sim_idx[joint_name]] = float(angle)

        hand_visible_prev = hand_visible

        # -- Apply to robot --
        joint_pos_des = joint_pos_target[:, robot_entity_cfg.joint_ids].clone()
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()

        sim.step()
        scene.update(sim_dt)


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    scene_cfg = ArmHandSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
