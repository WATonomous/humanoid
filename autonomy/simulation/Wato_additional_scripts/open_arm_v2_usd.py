"""Open the wato_arm_v2 arm USD scene in Isaac Sim.

Usage (from the repo root):
  ~/IsaacLab/isaaclab.sh -p autonomy/simulation/Wato_additional_scripts/open_arm_v2_usd.py
"""

import argparse
import contextlib
import os

from isaaclab.app import AppLauncher

USD_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "Humanoid_Wato",
    "wato_arm_v2",
    "arm_usd",
    "armWithStand.usd",
)

parser = argparse.ArgumentParser(description="Open the arm USD scene in Isaac Sim.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.kit.app
import omni.timeline
import isaaclab.sim as sim_utils


def main():
    usd_path = os.path.abspath(USD_PATH)
    print(f"[INFO] Opening: {usd_path}")
    sim_utils.open_stage(usd_path)
    omni.timeline.get_timeline_interface().play()

    # Open a second viewport locked to ego_cam so its exact framing is
    # visible side-by-side with the main Perspective view, for repositioning.
    try:
        from omni.kit.viewport.utility import create_viewport_window

        ego_cam_path = "/World/armWithStand/base_link/ego_cam"
        viewport_window = create_viewport_window("Ego Cam Preview", width=640, height=480)
        viewport_window.viewport_api.camera_path = ego_cam_path
        print(f"[INFO] Opened 'Ego Cam Preview' viewport locked to {ego_cam_path}")
    except Exception as exc:  # noqa: BLE001 -- best-effort convenience feature
        print(f"[INFO] Could not open Ego Cam Preview viewport: {exc}")

    app = omni.kit.app.get_app_interface()
    with contextlib.suppress(KeyboardInterrupt):
        while app.is_running():
            app.update()


if __name__ == "__main__":
    main()
    simulation_app.close()
