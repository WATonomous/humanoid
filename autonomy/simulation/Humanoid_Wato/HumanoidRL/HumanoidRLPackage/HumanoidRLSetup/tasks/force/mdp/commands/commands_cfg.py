from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG
from isaaclab.utils import configclass

from .impulse_force_command import UniformImpulseForceCommand


@configclass
class UniformImpulseForceCommandCfg(CommandTermCfg):
    """Configuration for uniform impulse force command generator."""

    class_type: type = UniformImpulseForceCommand

    asset_name: str = MISSING
    """Robot asset used to transform commands into the world frame."""

    contact_sensor_name: str = "contact_forces"
    """Contact sensor used to read measured push forces for debug visualization."""

    impulse_duration_s: float = 0.5
    """Duration of each force impulse window [s]."""

    @configclass
    class Ranges:
        """Uniform distribution ranges for impulse force commands."""

        pos_x: tuple[float, float] = MISSING
        pos_y: tuple[float, float] = MISSING
        pos_z: tuple[float, float] = MISSING
        force_x: tuple[float, float] = MISSING
        force_y: tuple[float, float] = MISSING
        force_z: tuple[float, float] = MISSING

    ranges: Ranges = MISSING

    goal_force_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/goal_force"
    )
    current_force_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/current_force"
    )
    target_region_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/target_region",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.08,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.8, 0.2), opacity=0.35),
            )
        },
    )

    goal_force_visualizer_cfg.markers["arrow"].scale = (0.5, 0.12, 0.12)
    current_force_visualizer_cfg.markers["arrow"].scale = (0.5, 0.12, 0.12)
