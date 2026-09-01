"""Scene registry: ``@scene``-decorated ``InteractiveSceneCfg`` classes,
auto-discovered from ``humanoid_scenes/<name>/scene.py``.

Adding a manipulation scene for teleop data collection is **one folder**:

    humanoid_scenes/my_scene/
        __init__.py      # empty
        scene.py         # @scene("my_scene") class MySceneCfg(InteractiveSceneCfg): ...

Nothing else -- no edits to ``pioneer_humanoid.teleop_scenes``, ``keyboard_teleop``
or the Dockerfile. The scene declares ``robot = MISSING``; a teleop script plugs
its own arm in via ``make_scene_cfg``.
"""
from __future__ import annotations

import importlib
import pkgutil
from dataclasses import dataclass
from typing import Optional


@dataclass
class _Entry:
    cfg_cls: type
    robot_pos: tuple = (0.0, 0.0, 0.0)
    camera: Optional[tuple] = None  # (eye_xyz, target_xyz) for the teleop initial view


_REGISTRY: dict[str, _Entry] = {}
_DISCOVERED = False


def scene(name: str, *, robot_pos=(0.0, 0.0, 0.0), camera=None):
    """Register an ``InteractiveSceneCfg`` subclass under ``name``.

    robot_pos: where to place the arm base. Scenes with a low table (arm reaching
               down from origin) use ``(0, 0, 0)``; scenes where the arm stands
               on its floor stand use roughly ``(0, 0, 1.2)``.
    camera:    optional ``(eye, target)`` for the teleop initial view; ``None``
               lets the teleop script fall back to its default framing.
    """
    def deco(cfg_cls):
        _REGISTRY[name] = _Entry(cfg_cls, tuple(robot_pos), camera)
        return cfg_cls

    return deco


def _discover() -> None:
    global _DISCOVERED
    if _DISCOVERED:
        return
    import humanoid_scenes

    for m in pkgutil.iter_modules(humanoid_scenes.__path__):
        if m.name.startswith("_"):
            continue
        importlib.import_module(f"humanoid_scenes.{m.name}.scene")
    _DISCOVERED = True


def list_scenes() -> list[str]:
    _discover()
    return sorted(_REGISTRY)


def scene_camera(name: str):
    _discover()
    return _REGISTRY[name].camera


def make_scene_cfg(name, robot_cfg, *, num_envs=1, env_spacing=2.0, prim_path="{ENV_REGEX_NS}/Robot"):
    """Instantiate scene ``name`` with ``robot_cfg`` plugged into its ``MISSING`` robot."""
    _discover()
    entry = _REGISTRY[name]
    cfg = entry.cfg_cls(num_envs=num_envs, env_spacing=env_spacing)
    cfg.robot = robot_cfg.replace(
        prim_path=prim_path,
        init_state=robot_cfg.init_state.replace(pos=entry.robot_pos),
    )
    if hasattr(cfg, "ee_frame"):
        cfg.ee_frame = None
    return cfg
