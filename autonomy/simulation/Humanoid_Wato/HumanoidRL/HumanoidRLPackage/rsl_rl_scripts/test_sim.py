"""
Simple script to boot up the Isaac Lab GUI with the Cabinet scene.
Run this using: python autonomy/simulation/Humanoid_Wato/HumanoidRL/HumanoidRLPackage/rsl_rl_scripts/test_sim.py
"""

import argparse
from isaaclab.app import AppLauncher

# Launch the app
parser = argparse.ArgumentParser(description="Test Isaac Lab GUI over VNC.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Force the app to open the GUI (turn off headless)
args_cli.headless = False
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg, InteractiveScene
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# Define a simple scene with just the ground and the cabinet
class CabinetTestSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    cabinet = AssetBaseCfg(
        prim_path="/World/Cabinet",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            scale=(1.15, 1.15, 1.15),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.75, 0.0, 0.35),
            rot=(0.0, 0.0, 0.0, 1.0),
        )
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )

# Create the scene
scene_cfg = CabinetTestSceneCfg(num_envs=1, env_spacing=2.0)
scene = InteractiveScene(scene_cfg)

# Play the simulation
simulation_app.update()
sim = sim_utils.SimulationContext()
sim.play()

print("==================================================================")
print("Simulation is running! Try to look around using your mouse.")
print("If the VNC crashes now, it means it cannot handle the RTX Renderer.")
print("==================================================================")

# Keep the window open
while simulation_app.is_running():
    sim.step()

simulation_app.close()
