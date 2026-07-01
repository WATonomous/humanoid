import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="RealSense D455 camera simulation.")
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to spawn."
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    help="Disable Fabric API and use USD instead.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import os
import torch


import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass


@configclass
class SensorsSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)),
    )
    camera = CameraCfg(
        prim_path="/World/Camera",
        update_period=0.1,
        height=800,
        width=1280,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.93,
            focus_distance=400.0,
            horizontal_aperture=3.6630,
            vertical_aperture=2.1396,
            clipping_range=(0.4, 20),
        ),
    )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    while simulation_app.is_running():
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)
        print("RGB shape:", scene["camera"].data.output["rgb"].shape)
        print(
            "Depth shape:", scene["camera"].data.output["distance_to_image_plane"].shape
        )


def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=0.005, device=args_cli.device, use_fabric=not args_cli.disable_fabric
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[3.5, 3.5, 3.5], target=[0.0, 0.0, 0.0])
    scene_cfg = SensorsSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    print("[INFO]: Setup complete...")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()