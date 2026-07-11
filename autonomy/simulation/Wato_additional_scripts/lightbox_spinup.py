"""Open the bimanual arm lightbox USD stage and run the simulation."""

import argparse
import os

from isaaclab.app import AppLauncher

DEFAULT_USD_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "Humanoid_Wato",
    "wato_bimanual_arm",
    "bimanual_arm_usd",
    "bimanual_arm_lightbox.usd",
)

parser = argparse.ArgumentParser(description="Spin up the bimanual arm lightbox USD scene")
parser.add_argument("--usd-path", type=str, default=DEFAULT_USD_PATH, help="Path to the lightbox USD file to open.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaacsim.core.utils.stage as stage_utils
import isaaclab.sim as sim_utils


def main():
    stage_utils.open_stage(os.path.abspath(args_cli.usd_path))

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([1.569, 0.042, 1.0], [0.069, 0.042, 0.4])

    sim.reset()
    print(f"[INFO]: Loaded {args_cli.usd_path}")

    while simulation_app.is_running():
        sim.step()


if __name__ == "__main__":
    main()
    simulation_app.close()
