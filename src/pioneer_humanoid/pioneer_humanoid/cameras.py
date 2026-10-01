"""Cameras mounted on the Pioneer bimanual arm (poses and lenses from PR #296's camera USD).

Defined in code so the arm asset stays camera-free. Used for recording (teleop) and by anything
that needs the robot's views (e.g. policy eval). Requires --enable_cameras.
Prim paths assume the robot is spawned at ``{ENV_REGEX_NS}/Robot``.
"""
import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg

# name: (parent link, prim, pos, rot wxyz [opengl], focal length, horizontal aperture, clipping,
#        sensor aspect w/h or None)
#   ego          RealSense D455 colour at 640x480 (4:3), 40 deg down. ESTIMATE ~80 x 65 deg: native
#                1280x800 lens with the sides cropped to 4:3. Replace with the real camera_info.
#   wrist_left   between the fingers, ~60 deg hFOV, real camera not chosen yet
#   wrist_right  mirror of wrist_left
_CAMERAS = {
    "ego": (
        "base_link", "cam_ego",
        (0.08421, -0.00008, 0.26038), (0.640856, 0.298836, -0.298836, -0.640856),
        1.93, 3.896 * (800 * 4 / 3) / 1280, (0.01, 100.0), 4 / 3,
    ),
    "wrist_left": (
        "link6l", "cam_wrist_left",
        (0.06179, 0.05297, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0), None,
    ),
    "wrist_right": (
        "link6", "cam_wrist_right",
        (0.06179, -0.04397, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0), None,
    ),
}
CAMERA_NAMES = tuple(_CAMERAS)


def make_camera_cfg(name: str, height: int, width: int) -> CameraCfg:
    """CameraCfg for camera ``name`` at height x width. Vertical aperture follows the aspect ratio (no stretch).

    A camera modelled on a real sensor must keep that sensor's aspect, so sim sees what the real one sees."""
    body, prim, pos, rot, focal, h_aperture, clip, aspect = _CAMERAS[name]
    if aspect is not None and abs(width / height - aspect) > 0.02 * aspect:
        raise ValueError(f"camera {name}: {width}x{height} must match its sensor aspect {aspect:.3g} (w/h)")
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + f"{body}/{prim}",
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=focal,
            horizontal_aperture=h_aperture,
            vertical_aperture=h_aperture * height / width,
            clipping_range=clip,
        ),
        offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="opengl"),
        height=height,
        width=width,
        update_period=0.0,
        data_types=["rgb"],
    )
