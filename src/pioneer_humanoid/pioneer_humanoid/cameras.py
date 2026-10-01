"""Isaac Lab cameras mounted on the Pioneer bimanual arm; mounts and lenses in camera_params.py.

Defined in code so the arm asset stays camera-free. Used for recording (teleop) and by anything
that needs the robot's views (e.g. policy eval). Requires --enable_cameras.
Prim paths assume the robot is spawned at ``{ENV_REGEX_NS}/Robot``.
"""
import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg

from .camera_params import CAMERA_NAMES, CAMERAS, vertical_aperture  # noqa: F401 -- CAMERA_NAMES re-exported


def make_camera_cfg(name: str, height: int, width: int) -> CameraCfg:
    """CameraCfg for camera ``name`` at height x width (vertical aperture follows the aspect)."""
    cam = CAMERAS[name]
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + f"{cam.body}/{cam.prim}",
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=cam.focal,
            horizontal_aperture=cam.h_aperture,
            vertical_aperture=vertical_aperture(name, height, width),
            clipping_range=cam.clip,
        ),
        offset=CameraCfg.OffsetCfg(pos=cam.pos, rot=cam.rot, convention="opengl"),
        height=height,
        width=width,
        update_period=0.0,
        data_types=["rgb"],
    )
