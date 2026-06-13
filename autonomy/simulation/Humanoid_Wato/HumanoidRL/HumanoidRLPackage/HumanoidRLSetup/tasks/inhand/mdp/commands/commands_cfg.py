import isaaclab.sim as sim_utils
from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from .orientation_command import InHandReOrientationCommand


@configclass
class InHandReOrientationCommandCfg(CommandTermCfg):
    """Configuration for the uniform 3D orientation command term.

    Please refer to the :class:`InHandReOrientationCommand` class for more details.
    """

    class_type: type = InHandReOrientationCommand
    resampling_time_range: tuple[float, float] = (1e6, 1e6)  # no resampling based on time

    asset_name: str = "object"
    """Scene asset that receives orientation goals (the manipuland rigid body)."""

    init_pos_offset: tuple[float, float, float] = (0.0, 0.0, -0.04)
    """Offset added to the object's default spawn position to form the goal position command.

    Does not move the physical cube at reset — only shifts the position target used by
    ``track_pos_l2`` and the goal-marker anchor (before ``marker_pos_offset``).
    """

    make_quat_unique: bool = False
    """Whether sampled goal quaternions are forced to have a positive real part."""

    orientation_success_threshold: float = 0.4
    """Orientation error (rad) below which the goal counts as reached."""

    update_goal_on_success: bool = True
    """Resample goal orientation when the current goal is reached."""

    rotation_axes: list[str] = ["x", "y"]
    """World axes used when sampling random goal orientations."""

    marker_pos_offset: tuple[float, float, float] = (-0.2, -0.06, 0.08)
    """Extra offset for the goal-orientation debug marker only (not the physical cube)."""

    debug_vis: bool = True

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/goal_marker",
        markers={
            "goal": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(1.0, 1.0, 1.0),
            ),
        },
    )
    """Goal-orientation debug marker (DexCube visual only)."""