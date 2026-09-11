"""Bare scene: ground + light + arm, nothing else. The default teleop scene."""
from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from humanoid_scenes import scene


@scene("bare", camera=([2.5, 2.5, 2.0], [0.0, 0.0, 0.8]))
@configclass
class BareSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = MISSING

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
