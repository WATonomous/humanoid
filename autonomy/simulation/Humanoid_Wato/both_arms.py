import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Load both Humanoid Wato arms")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import (
    ARM_CFG,
    LEFT_ARM_CFG,
)
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass


@configclass
class BothArmsSceneCfg(InteractiveSceneCfg):
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
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=LEFT_ARM_CFG.init_state.joint_pos,
        ),
    )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()

    while simulation_app.is_running():
        scene.write_data_to_sim()

        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.0])

    scene_cfg = BothArmsSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
