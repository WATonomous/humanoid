"""Manipulation scenes for teleop data collection — one folder per scene.

See humanoid_scenes/_register.py for the @scene decorator and how discovery works.
"""
from ._register import list_scenes, make_scene_cfg, scene, scene_camera  # noqa: F401
