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


def read_joint_file() -> dict[str, float]:
    """Read the latest joint angles from the shared file. Returns {} if not ready."""
    try:
        with open(JOINT_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


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

    # ── Override finger joint limits to allow deeper curl (120° instead of 90°) ──
    # URDF caps at ±1.57 rad (90°). We widen PIP/DIP to 2.09 rad (120°) in sim only.
    WIDER_LIMITS: dict[str, tuple[float, float]] = {
        "pip_index":  (0.0,   2.09),
        "dip_index":  (-2.09, 0.0),
        "pip_middle": (0.0,   2.09),
        "dip_middle": (0.0,   2.09),
        "pip_ring":   (-2.09, 0.0),
        "dip_ring":   (-2.09, 0.0),
        "pip_pinky":  (-2.09, 0.0),
        "dip_pinky":  (0.0,   2.09),
    }
    try:
        limits = robot.data.joint_pos_limits.clone()  # [num_envs, num_joints, 2]
        for joint_name, (lo, hi) in WIDER_LIMITS.items():
            if joint_name in name_to_sim_idx:
                idx = name_to_sim_idx[joint_name]
                limits[:, idx, 0] = lo
                limits[:, idx, 1] = hi
        # Isaac Lab 0.36 uses the underlying PhysX view to set DOF limits
        env_ids = torch.tensor([0], dtype=torch.int32, device=sim.device)
        robot._root_physx_view.set_dof_limits(limits, env_ids)
        print("[INFO]: Finger joint limits widened to ±2.09 rad (120°) for PIP/DIP joints.")
    except Exception as e:
        print(f"[WARN]: Could not override joint limits: {e} — using URDF defaults.")

    # Persistent position target (1 env × 21 DOF), starts at default
    joint_pos_target = robot.data.default_joint_pos.clone()

    print("[INFO]: Setup complete — reading hand data from", JOINT_FILE)
    print("[INFO]: Make sure wato_hand_ros2_node.py is running in another terminal.")

    while simulation_app.is_running():
        # -- Read latest joint angles from shared file --
        hand_dict = read_joint_file()

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
