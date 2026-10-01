"""Scene registry: ``@scene``-decorated builders, auto-discovered from
``humanoid_mujoco_scenes/<name>/scene.py``.

A scene is one function that adds world geometry to an ``mujoco.MjSpec``; ``make_model``
attaches the Pioneer arm (pioneer_humanoid.mujoco_arm) at ``robot_pos`` and compiles.
CPU only: no Isaac imports anywhere in this package.
"""
from __future__ import annotations

import importlib
import pkgutil
from dataclasses import dataclass
from typing import Callable, Optional

import mujoco

# base_link lift that puts the stand's feet on the floor (z=0); same value as Isaac's push scene.
ROBOT_STAND_LIFT_Z = 1.1997


@dataclass
class _Entry:
    build: Callable[[mujoco.MjSpec], None]
    robot_pos: tuple = (0.0, 0.0, ROBOT_STAND_LIFT_Z)
    camera: Optional[dict] = None  # mujoco free-camera fields: lookat, distance, azimuth, elevation


_REGISTRY: dict[str, _Entry] = {}
_DISCOVERED = False


def scene(name: str, *, robot_pos=(0.0, 0.0, ROBOT_STAND_LIFT_Z), camera=None):
    """Register ``build(spec)`` under ``name``. ``robot_pos``: arm base placement (default: on its stand)."""
    def deco(build):
        _REGISTRY[name] = _Entry(build, tuple(robot_pos), camera)
        return build

    return deco


def _discover() -> None:
    global _DISCOVERED
    if _DISCOVERED:
        return
    import warnings

    import humanoid_mujoco_scenes

    for m in pkgutil.iter_modules(humanoid_mujoco_scenes.__path__):
        if not m.ispkg or m.name.startswith("_"):
            continue
        try:
            importlib.import_module(f"humanoid_mujoco_scenes.{m.name}.scene")
        except Exception as e:  # noqa: BLE001 -- one broken scene must not hide the rest
            warnings.warn(f"humanoid_mujoco_scenes: skipping {m.name!r} -- {type(e).__name__}: {e}", stacklevel=2)
    _DISCOVERED = True


def list_scenes() -> list[str]:
    _discover()
    return sorted(_REGISTRY)


def scene_camera(name: str) -> Optional[dict]:
    _discover()
    return _REGISTRY[name].camera


def add_floor(spec: mujoco.MjSpec) -> None:
    """Checker floor at z=0, a light, and a skybox."""
    tex = spec.add_texture(
        name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], width=512, height=512,
    )
    spec.add_texture(
        name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.6, 0.7, 0.8], rgb2=[0.1, 0.1, 0.15], width=256, height=256,
    )
    mat = spec.add_material(name="grid", texrepeat=[8, 8], reflectance=0.1)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.05], material="grid")
    spec.worldbody.add_light(pos=[0.5, 0, 3.0], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8], castshadow=True)


def make_model(name: str, cameras: dict[str, tuple[int, int]] | None = None) -> mujoco.MjModel:
    """Compile scene ``name`` with the arm attached; joint/actuator names are the URDF joint names.

    ``cameras``: robot cameras to mount, {name: (height, width)} (pioneer_humanoid.camera_params).
    """
    from pioneer_humanoid.mujoco_arm import arm_spec

    _discover()
    entry = _REGISTRY[name]
    spec = mujoco.MjSpec()
    spec.modelname = name
    spec.option.timestep = 0.002
    # Implicit damping keeps the stiff arm PD (kp up to 2270) stable at this step.
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960
    entry.build(spec)
    frame = spec.worldbody.add_frame(pos=list(entry.robot_pos))
    frame.attach_body(arm_spec(cameras).body("base_link"), "", "")
    return spec.compile()
