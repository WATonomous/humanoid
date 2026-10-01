"""Data-collection camera configs for pioneer_bimanual_arm teleop.

ego_cam (on base_link) and wrist_cam (on link6l) are the cameras that record the
dataset image reads. They live here rather than in the robot config
because they are teleop-scene config, not properties of the robot: the arm asset
ships no cameras (the superseded armWithStand.usd baked them into its sensor
layer), so they are defined and SPAWNED in code, which also keeps a future
re-export from silently dropping them.

The recording cameras at the bottom (make_record_cam_cfg: ego, wrist_left, wrist_right; PR #296's poses)
are a separate set, used by keyboard_teleop --record. wrist_right is defined but the default
camera schema leaves it disabled (the right arm is held still in left-arm teleop).

Prim paths assume the robot is spawned at ``{ENV_REGEX_NS}/Robot``.
"""
import math

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg

# focal_length 18 is ~60deg horizontal; lower it to widen. run_quest_bimanual_teleop.py overwrites
# ego_cam's at runtime to match the headset's widened RSD455 FOV.
DATA_CAM_LENS = sim_utils.PinholeCameraCfg(
    focal_length=7.336, horizontal_aperture=20.955, vertical_aperture=15.2908,
    clipping_range=(0.01, 100.0),
)

# ego_cam pose, relative to base_link. Only an initial value in the Quest teleop script, which
# re-aims ego_cam at the operator's head viewpoint at startup when --record is passed.
EGO_CAM_POS = (0.047450090928410314, -0.008096717438775313, 0.21180604954921534)
EGO_CAM_ROT = (0.8660254037844387, 0.49999999999999983, 0.0, 0.0)

# wrist_cam aiming. The two angles are independent: roll spins the image, pitch aims the camera.
WRIST_CAM_ROLL_DEG = 270.0   # rotates the image counter-clockwise; 90 / 180 / 270
WRIST_CAM_PITCH_DEG = 30.0   # angles the camera down toward the gripper

# wrist_cam mount, relative to link6l's origin. Gripper-relative: +X up, +Y left, -Z forward
# (follows from the roll/pitch above). It sits inside the wrist housing, so large moves bury it
# in the mesh; the Quest script's _SHOW_WRIST_CAM_MARKER flag draws a sphere here to check.
WRIST_CAM_POS = (0.06168, 0.053, -0.06995)


def _quat_mul_wxyz(a: tuple, b: tuple) -> tuple:
    """Hamilton product of two (w, x, y, z) quaternions as plain tuples.

    isaaclab.utils.math.quat_mul wants torch tensors; these are module constants."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


# Pitch must be composed OUTSIDE the roll. The other order rolls the pitch axis too, so changing
# the roll would silently change which way the camera tilts.
def _wrist_cam_rot(roll_deg: float) -> tuple:
    """Wrist-camera orientation for a given roll, at the shared WRIST_CAM_PITCH_DEG."""
    return _quat_mul_wxyz(
        (math.cos(math.radians(WRIST_CAM_PITCH_DEG) / 2.0), 0.0,
         math.sin(math.radians(WRIST_CAM_PITCH_DEG) / 2.0), 0.0),
        (math.cos(math.radians(roll_deg) / 2.0), 0.0, 0.0,
         math.sin(math.radians(roll_deg) / 2.0)),
    )


WRIST_CAM_ROT = _wrist_cam_rot(WRIST_CAM_ROLL_DEG)


def make_ego_cam_cfg() -> CameraCfg:
    """Fresh CameraCfg for ego_cam. Requires launching with --enable_cameras.

    A new instance per call, not a shared constant: InteractiveScene rewrites prim_path in place
    when it resolves {ENV_REGEX_NS}."""
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/ego_cam",
        spawn=DATA_CAM_LENS,
        offset=CameraCfg.OffsetCfg(pos=EGO_CAM_POS, rot=EGO_CAM_ROT, convention="opengl"),
        height=480, width=640,
        # Refresh whenever the app renders. An additional per-camera update_period on top of
        # that froze this feed on a stale scene state.
        update_period=0.0,
        data_types=["rgb"],
    )


# Right wrist cam is the left one mirrored through the robot's XZ plane: position (x,y,z)->
# (x,-y,z), roll -> 180 - roll. The +180 is not cosmetic: a pure frame mirror aims correctly
# but flips image-up (local +Y -> -Y), rendering the gripper upside down; the extra 180 roll
# about the view axis restores it. Pitch is unchanged (already about the mirror-plane normal).
#
# Asset asymmetry, compensated: link6's origin sits 9.0mm further out in Y than the exact mirror
# of link6l's (CAD frame placement; the fingertips themselves mirror perfectly). Without adding
# it back, the right feed sat too far outboard in the headset.
_LINK6_ORIGIN_Y_ASYMMETRY_M = 0.008997  # link6l.y + link6.y, measured in-sim at the rest pose


def make_wrist_cam_cfg(body: str = "link6l", name: str = "wrist_cam", mirror: bool = False) -> CameraCfg:
    """Fresh CameraCfg for a wrist camera -- see make_ego_cam_cfg for why it is a factory.

    Defaults reproduce the original single link6l camera exactly, so existing call sites are
    unchanged. Pass ``body="link6", mirror=True`` for the opposite arm's wrist."""
    mirrored_pos = (WRIST_CAM_POS[0],
                    -WRIST_CAM_POS[1] + _LINK6_ORIGIN_Y_ASYMMETRY_M,
                    WRIST_CAM_POS[2])
    pos = mirrored_pos if mirror else WRIST_CAM_POS
    rot = _wrist_cam_rot(180.0 - WRIST_CAM_ROLL_DEG) if mirror else WRIST_CAM_ROT
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + f"{body}/{name}",
        spawn=DATA_CAM_LENS,
        offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="opengl"),
        height=480, width=640,
        update_period=0.0,
        data_types=["rgb"],
    )


# --- Recording cameras: pose and lens from PR #296's camera USD, defined here in code so the arm
# asset stays camera-free. Used by keyboard_teleop --record; separate from the Quest cameras above.
#
# Each entry: (parent link, prim name, pos, rot wxyz in USD/opengl camera axes, focal length,
# horizontal aperture, clipping range). Poses are #296's composed prim transforms, relative to the
# parent link:
#   ego         RealSense D455 colour sensor (OV9782) on base_link: faces +X, 40 deg down
#               (#296 env v3 tilt). Lens from Isaac 5.1's rsd455.usd, ~90 deg horizontal FOV.
#   wrist_left  link6l, looking between the fingers. ~60 deg horizontal FOV.
#   wrist_right link6, mirror of wrist_left (same orientation in its own link frame).
_RECORD_CAMS = {
    "ego": (
        "base_link", "record_ego_cam",
        (0.08421, -0.00008, 0.26038), (0.640856, 0.298836, -0.298836, -0.640856),
        1.93, 3.896, (0.01, 100.0),
    ),
    "wrist_left": (
        "link6l", "record_wrist_cam_left",
        (0.06179, 0.05297, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0),
    ),
    "wrist_right": (
        "link6", "record_wrist_cam_right",
        (0.06179, -0.04397, -0.07077), (0.696364, -0.122788, 0.122788, -0.696364),
        18.147562, 20.955, (0.05, 5.0),
    ),
}
RECORD_CAM_NAMES = tuple(_RECORD_CAMS)


def make_record_cam_cfg(name: str, height: int, width: int) -> CameraCfg:
    """Fresh CameraCfg for recording camera ``name`` (see RECORD_CAM_NAMES) at height x width.

    Requires launching with --enable_cameras. The vertical aperture follows the output aspect
    ratio, so the horizontal FOV is fixed and the image is never stretched."""
    body, prim, pos, rot, focal, h_aperture, clip = _RECORD_CAMS[name]
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
