"""Bare scene: floor + light + arm on its stand. The default scene."""
from __future__ import annotations

from humanoid_mujoco_scenes import add_floor, scene


@scene("bare", camera=dict(lookat=[0.2, 0.0, 0.9], distance=2.5, azimuth=150, elevation=-20))
def build(spec):
    add_floor(spec)
