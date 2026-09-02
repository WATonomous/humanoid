# Vendored from the LeHome Challenge (https://github.com/lehome-official/lehome-challenge)
# @ a805ad2f7ab52a4583066fc4ee5180459a7f9d15, originally under the Apache License, Version 2.0.
# See ../../LICENSE and ../../NOTICE for the list of modifications.

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.sensors import TiledCameraCfg

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_TASK_DIR = os.path.abspath(os.path.join(_PKG_DIR, ".."))


@configclass
class GarmentEnvCfg(DirectRLEnvCfg):
    """Robot-agnostic base config for the garment-fold task.

    MODIFIED (WATonomous): upstream (`garment_bi_cfg_v2.GarmentEnvCfg`) hard-coded
    two `SO101_FOLLOWER_CFG` arms as `left_robot` / `right_robot`. Those fields and
    the `lehome.assets.robots.lerobot` import are removed here -- a concrete
    subclass provides the robot(s) (see `garment_pioneer_cfg.GarmentPioneerEnvCfg`).
    The wrist/top camera `prim_path`s below still point at the SO101 prim layout
    and are meant to be overridden by the subclass too.
    """

    # env
    decimation = 1
    episode_length_s = 60
    action_scale = 1.0  # [N]
    action_space = 12
    observation_space = 12
    state_space = 0
    # simulation
    render_cfg = sim_utils.RenderCfg(rendering_mode="quality", antialiasing_mode="FXAA")
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 90,
        render_interval=decimation,
        render=render_cfg,
        use_fabric=False,
    )
    # garment_name (str): Garment name in the format "Type_Length_Seen/Unseen_Index",
    # e.g., "Top_Long_Unseen_0", "Top_Short_Seen_1",
    garment_name: str = None
    garment_version: str = "Release"  # "Release" or "Holdout"
    # MODIFIED: default to the vendored sample garments; point at the full
    # `hf download lehome/asset_challenge` tree for real training/eval.
    garment_cfg_base_path: str = os.path.join(_TASK_DIR, "vendor_assets", "garments")
    particle_cfg_path: str = os.path.join(_PKG_DIR, "config", "particle_garment_cfg.yaml")
    # random seed
    use_random_seed: bool = True
    random_seed: int = 42

    left_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/Robot/Left_Robot/gripper/left_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04),
            rot=(-0.404379, -0.912179, -0.0451242, 0.0486914),
            convention="ros",
        ),  # wxyz
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5,
            focus_distance=400.0,
            horizontal_aperture=36.83,  # For a 75° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,  # 30FPS
    )
    right_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/Robot/Right_Robot/gripper/right_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04),
            rot=(-0.404379, -0.912179, -0.0451242, 0.0486914),
            convention="ros",
        ),  # wxyz
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5,
            focus_distance=400.0,
            horizontal_aperture=36.83,  # For a 75° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,  # 30FPS
    )
    top_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/Robot/Right_Robot/base/top_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.245, -0.44, 0.56),
            rot=(0.1650476, -0.9862856, 0.0, 0.0),
            convention="ros",
        ),  # wxyz
        data_types=["rgb", "depth"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=28.7,
            focus_distance=400.0,
            horizontal_aperture=38.11,  # For a 78° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
    )
    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1, env_spacing=4.0, replicate_physics=True
    )

    viewer = ViewerCfg(eye=(1.9, -4.7, 1.4), lookat=(1.3, 1.2, -1))
