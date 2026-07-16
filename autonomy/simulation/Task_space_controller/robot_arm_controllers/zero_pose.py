"""Loads the bimanual arm in Isaac Sim with every joint held at raw position 0 degrees
(true USD zero, no rest-pose offsets, no bimanual_arm_cfg.py teleop default pose).

Same model as task_space_real.py / keyboard_teleop.py (BIMANUAL_ARM_CFG, the arm on the
test stand). Unlike joint_mapping_reference.py, this doesn't move anything or apply any
per-joint rest offset -- it's just the plain zero pose, held forever, for visual
reference (e.g. comparing against a physically zeroed real arm).

Usage:
  python zero_pose.py
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Same model as task_space_real.py / keyboard_teleop.py
_KEYBOARD_TELEOP_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "Teleop", "keyboard_based_teleoperation")
)
sys.path.insert(0, _KEYBOARD_TELEOP_DIR)

# All Isaac Sim / Isaac Lab imports must come after SimulationApp is instantiated
from bimanual_arm_cfg import BIMANUAL_ARM_CFG, apply_joint_limits  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
import torch  # noqa: E402


@configclass
class ArmSceneCfg(InteractiveSceneCfg):
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


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot = scene["robot"]
    apply_joint_limits(robot)

    zero_pos = torch.zeros_like(robot.data.default_joint_pos)
    zero_vel = torch.zeros_like(robot.data.default_joint_vel)
    robot.write_joint_state_to_sim(zero_pos, zero_vel)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    print("[INFO] All joints held at raw position 0 degrees (true USD zero). Ctrl+C to stop.")

    sim_dt = sim.get_physics_dt()
    all_ids = list(range(len(robot.data.joint_names)))
    while simulation_app.is_running():
        robot.set_joint_position_target(zero_pos, joint_ids=all_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])

    scene = InteractiveScene(ArmSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()

    print("[INFO]: Setup complete...")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
