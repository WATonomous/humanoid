"""Shared InteractiveScene configs for pioneer_bimanual_arm teleop scripts.

Selected by a ``--scene`` flag on the teleop script:

- ``BimanualBareSceneCfg``  ground + light + arm only (default).
- ``BimanualPushSceneCfg``  the push-block task scene -- imported directly from
                            the RL task (``...tasks/push/scene.py``) so teleop
                            demos are collected in the *exact* scene the push
                            policy trains on: same table, ramp-box, block,
                            lightbox and grounding. Only ``robot`` (lifted onto
                            its floor stand) and ``ee_frame`` (dropped) differ.

The push scene's geometry and the teleop-verified placement (table top at
``TABLE_TOP_Z``, arm lifted ``ROBOT_STAND_LIFT_Z``) live in one place --
``push/scene.py`` -- not copied here.
"""

from __future__ import annotations

import sys
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from pioneer_humanoid.bimanual_arm import BIMANUAL_ARM_CFG

# The push task package isn't installable yet (it lives 4 dirs deep in
# HumanoidRLPackage); add its `tasks/` parent to sys.path and import `push.scene`
# directly. This runs push/__init__.py (gym.register x4) but NOT the heavy
# tasks/__init__.py that pulls in every RL task.
_RL_TASKS = (
    Path(__file__).resolve().parents[2]
    / "simulation" / "Humanoid_Wato" / "HumanoidRL"
    / "HumanoidRLPackage" / "HumanoidRLSetup" / "tasks"
)
if str(_RL_TASKS) not in sys.path:
    sys.path.insert(0, str(_RL_TASKS))
from push.scene import PushBlockSceneCfg, ROBOT_STAND_LIFT_Z  # noqa: E402


@configclass
class BimanualBareSceneCfg(InteractiveSceneCfg):
    """Ground + light + the pioneer bimanual arm. Nothing else."""

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


@configclass
class BimanualPushSceneCfg(PushBlockSceneCfg):
    """The RL push scene, driven by the pioneer bimanual arm (no RL ee_frame)."""

    def __post_init__(self):
        self.robot = BIMANUAL_ARM_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=BIMANUAL_ARM_CFG.init_state.replace(pos=(0.0, 0.0, ROBOT_STAND_LIFT_Z)),
        )
        self.ee_frame = None


SCENE_CFGS = {
    "bare": BimanualBareSceneCfg,
    "push": BimanualPushSceneCfg,
}
