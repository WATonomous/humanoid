"""Simulator-neutral camera mounts on the Pioneer bimanual arm (poses and lenses from PR #296's camera USD).

Plain Python, shared by cameras.py (Isaac Lab) and mujoco_arm.py (MuJoCo). Poses are relative to the
parent link, OpenGL convention (camera looks along -Z, +Y up), which both simulators use.
"""
import math
from typing import NamedTuple


class CameraMount(NamedTuple):
    body: str                 # parent link
    prim: str                 # Isaac prim name under the link
    pos: tuple                # m, link frame
    rot: tuple                # wxyz, link frame, OpenGL convention
    focal: float              # mm
    h_aperture: float         # mm
    clip: tuple               # near, far (m)
    aspect: float | None      # sensor w/h the image must keep, or None


# ego          RealSense D455 colour at 640x480 (4:3), 40 deg down. ESTIMATE ~80 x 65 deg: native
#              1280x800 lens with the sides cropped to 4:3. Replace with the real camera_info.
# wrist_left   between the fingers, ~60 deg hFOV, real camera not chosen yet
# wrist_right  mirror of wrist_left
CAMERAS = {
    "ego": CameraMount(
        "base_link", "cam_ego",
        (0.08421, -0.00008, 0.26038), (0.640856, 0.298836, -0.298836, -0.640856),
        1.93, 3.896 * (800 * 4 / 3) / 1280, (0.01, 100.0), 4 / 3,
    ),
    "wrist_left": CameraMount(
        "link6l", "cam_wrist_left",
        (0.06179, 0.05297, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0), None,
    ),
    "wrist_right": CameraMount(
        "link6", "cam_wrist_right",
        (0.06179, -0.04397, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0), None,
    ),
}
CAMERA_NAMES = tuple(CAMERAS)


def vertical_aperture(name: str, height: int, width: int) -> float:
    """Vertical aperture (mm) at height x width: follows the image aspect, no stretch.

    A camera modelled on a real sensor must keep that sensor's aspect, so sim sees what the real one sees."""
    cam = CAMERAS[name]
    if cam.aspect is not None and abs(width / height - cam.aspect) > 0.02 * cam.aspect:
        raise ValueError(f"camera {name}: {width}x{height} must match its sensor aspect {cam.aspect:.3g} (w/h)")
    return cam.h_aperture * height / width


def vertical_fov_deg(name: str, height: int, width: int) -> float:
    return math.degrees(2.0 * math.atan(vertical_aperture(name, height, width) / (2.0 * CAMERAS[name].focal)))
