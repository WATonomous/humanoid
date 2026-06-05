"""Gym entry point that restores spawn flags lost during Hydra cfg round-trips."""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.envs import ManagerBasedRLEnv

if TYPE_CHECKING:
    from HumanoidRLPackage.HumanoidRLSetup.tasks.force.force_env_cfg import ForceEnvCfg


def finalize_force_env_cfg(cfg: ForceEnvCfg) -> None:
    """Ensure contact reporters are enabled on the robot before scene spawn."""
    cfg.scene.robot.spawn.activate_contact_sensors = True


def make_force_env(cfg=None, render_mode=None, **kwargs):
    if cfg is not None:
        finalize_force_env_cfg(cfg)
    return ManagerBasedRLEnv(cfg=cfg, render_mode=render_mode, **kwargs)
