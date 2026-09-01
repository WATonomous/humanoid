"""Scene lookup for pioneer_bimanual_arm teleop scripts (``--scene`` flag).

Most scenes are **discovered** from the ``humanoid_scenes`` package -- adding one
is a single folder there (see ``humanoid_scenes/_register.py``), nothing changes
here. Two are hand-wired:

- ``bare``  ground + light + arm only. Not a real scene, no package needed.
- ``push``  the RL push-block scene. Still lives 4 dirs deep in
            ``HumanoidRLPackage`` (not an installable package yet), so it's
            imported via a sys.path shim until it's extracted. Once it is, delete
            the shim and give it an ``@scene`` decorator like any other.

Teleop scripts call ``scene_names()`` (for ``--scene`` validation),
``build_scene_cfg(name)`` and ``camera_for(name)``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from humanoid_scenes import list_scenes, make_scene_cfg, scene_camera
from pioneer_humanoid.bimanual_arm import BIMANUAL_ARM_CFG

# ── legacy: push still lives in HumanoidRLPackage ────────────────────────────
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


_LEGACY = {"bare": BimanualBareSceneCfg, "push": BimanualPushSceneCfg}
_LEGACY_CAMERAS = {
    "bare": ([2.5, 2.5, 2.0], [0.0, 0.0, 0.8]),
    "push": ([1.4, -1.0, 0.9], [0.35, -0.2, 0.05]),
}


def scene_names() -> list[str]:
    """All available ``--scene`` values (legacy + discovered)."""
    return sorted({*_LEGACY, *list_scenes()})


def build_scene_cfg(name: str, *, num_envs: int = 1, env_spacing: float = 2.0):
    if name in _LEGACY:
        return _LEGACY[name](num_envs=num_envs, env_spacing=env_spacing)
    return make_scene_cfg(name, BIMANUAL_ARM_CFG, num_envs=num_envs, env_spacing=env_spacing)


def camera_for(name: str):
    """(eye, target) for the teleop initial view, or a sane default."""
    if name in _LEGACY:
        return _LEGACY_CAMERAS[name]
    return scene_camera(name) or ([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])
