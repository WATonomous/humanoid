"""Headless check that `keyboard_teleop.py --scene garment_fold` actually
builds: runs the same path keyboard_teleop.py's main() does (make_scene_cfg
-> InteractiveScene -> sim.reset() -> scene_post_init), minus the
Se3Keyboard/live-render loop (needs a real window, can't run headless -- the
interactive part itself still needs to be checked on your own machine):

    isaaclab.sh -p scripts/scene_flag_check.py

STATUS (2026-10-03): passes. garment_fold registers correctly, the scene
(worksurface + cameras) builds, sim.reset() succeeds, and post_init builds
the GarmentObject -- confirmed the robot lands at the same reach-tested base
pose GarmentPioneerEnvCfg uses. See teleop_scene.py's `_wrist_cam_cfg` for a
real camera-sensor bug this caught along the way.
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args([])
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from humanoid_isaac_scenes import list_scenes, make_scene_cfg, scene_camera, scene_post_init
from pioneer_humanoid.bimanual_arm import BIMANUAL_ARM_CFG
import humanoid_garment_fold  # noqa: F401 -- side effect: garment_fold scene module importable

print("SCENES:", list_scenes())
assert "garment_fold" in list_scenes(), "garment_fold not registered"

sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cpu")
sim = sim_utils.SimulationContext(sim_cfg)
sim.set_camera_view(*(scene_camera("garment_fold") or ([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])))

scene_cfg = make_scene_cfg("garment_fold", BIMANUAL_ARM_CFG, num_envs=1, env_spacing=2.0)
scene = InteractiveScene(scene_cfg)
print("SCENE_BUILT_OK")

sim.reset()
print("SIM_RESET_OK")

post_init = scene_post_init("garment_fold")
assert post_init is not None
post_init(scene, sim)
print("POST_INIT_OK garment=", scene.garment)

for _ in range(20):
    sim.step(render=True)
    scene.update(sim.get_physics_dt())

robot = scene["robot"]
print("ROBOT_POS", robot.data.root_state_w[:, 0:3].tolist())
print("SCENE_FLAG_CHECK_DONE")
import sys
sys.stdout.flush()
sys.stderr.flush()
import os
os._exit(0)
