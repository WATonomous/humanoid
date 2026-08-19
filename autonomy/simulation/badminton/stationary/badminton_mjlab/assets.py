"""Entity and scene specs for the mjlab task, split out of scene/badminton.xml.

The gate-validated scene file stays the single source of geometry and
calibration. Three views of it are produced here:

  robot_spec()    arm + stand + racket, XML actuators kept
  shuttle_spec()  free-floating shuttle (3 geoms + visual mesh)
  court_fn(spec)  scene hook: attaches court/net/floor and adds the four
                  explicit contact pairs across the entity prefixes

Splitting is by deletion, not reconstruction, so params.yaml edits and
build_scene.py reruns flow through unchanged.
"""

from __future__ import annotations

import math
import os

import mujoco

import aero

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_XML = os.path.join(_HERE, os.pardir, "scene", "badminton.xml")
SCENE_DIR = os.path.dirname(os.path.abspath(SCENE_XML))

ROBOT_ROOT = "arm_base_link"
SHUTTLE_ROOT = "shuttle"
ARM_JOINTS = tuple(f"arm_joint{i}" for i in range(1, 7))


def _load() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(os.path.abspath(SCENE_XML))
    # Attaching into the scene spec re-resolves asset paths relative to the
    # parent spec, so make every mesh path absolute first.
    for mesh in spec.meshes:
        if mesh.file and not os.path.isabs(mesh.file):
            mesh.file = os.path.join(SCENE_DIR, mesh.file)
    return spec


def _delete_worldbody_furniture(spec: mujoco.MjSpec) -> None:
    """Drop floor/lines/lights/sites that sit directly under worldbody."""
    for geom in [g for g in spec.worldbody.geoms]:
        spec.delete(geom)
    for site in [s for s in spec.worldbody.sites]:
        spec.delete(site)
    for light in [li for li in spec.worldbody.lights]:
        spec.delete(light)


def _delete_keys_and_pairs(spec: mujoco.MjSpec) -> None:
    for key in [k for k in spec.keys]:
        spec.delete(key)
    for pair in [p for p in spec.pairs]:
        spec.delete(pair)


def robot_spec() -> mujoco.MjSpec:
    spec = _load()
    _delete_worldbody_furniture(spec)
    _delete_keys_and_pairs(spec)
    for name in (SHUTTLE_ROOT, "net_body"):
        spec.delete(spec.body(name))
    return spec


def shuttle_spec() -> mujoco.MjSpec:
    spec = _load()
    _delete_worldbody_furniture(spec)
    _delete_keys_and_pairs(spec)
    for name in (ROBOT_ROOT, "net_body"):
        spec.delete(spec.body(name))
    for act in [a for a in spec.actuators]:
        spec.delete(act)
    return spec


def court_fn(spec: mujoco.MjSpec) -> None:
    """SceneCfg.spec_fn: attach the court and declare the contact pairs.

    Entity geoms all carry contype=0/conaffinity=0, so these explicit pairs
    are the only source of contacts — same contract as the CPU scene.
    """
    court = _load()
    _delete_keys_and_pairs(court)
    for name in (ROBOT_ROOT, SHUTTLE_ROOT):
        court.delete(court.body(name))
    for act in [a for a in court.actuators]:
        court.delete(act)
    frame = spec.worldbody.add_frame()
    spec.attach(court, prefix="court/", frame=frame)

    p = aero.load_params()
    c = p["contact"]
    cork, skirt = "shuttle/shuttle_cork", "shuttle/shuttle_col"
    face, floor, net = "robot/racket_face", "court/floor", "court/net"

    def pair(g1, g2, solref, solimp=None):
        pr = spec.add_pair(geomname1=g1, geomname2=g2, solref=solref)
        if solimp is not None:
            pr.solimp[: len(solimp)] = solimp

    pair(floor, skirt, list(c["floor_solref"]))
    pair(floor, cork, list(c["floor_solref"]))
    pair(net, skirt, list(c["net_solref"]))
    pair(cork, face, list(c["face_solref"]), list(c["face_solimp"]))


def arm_base_pose(params: dict) -> tuple[tuple, tuple]:
    """(pos, quat) of the arm base in the world, from params.

    mjlab overwrites a fixed-base entity's root body pos/quat with
    InitialStateCfg.pos/rot, so the XML placement must be restated there.
    """
    a = params["arm"]
    half = math.radians(a["mount_yaw_deg"]) / 2.0
    return ((a["base_x"], a["base_y"], a["mount_height"]),
            (math.cos(half), 0.0, 0.0, math.sin(half)))


# Ready pose = the scene keyframe's arm joints (see scripts/build_scene.py).
READY_JOINT_POS = {
    "arm_joint1": -1.5708,
    "arm_joint2": 0.0,
    "arm_joint3": 0.0,
    "arm_joint4": 0.0,
    "arm_joint5": 1.5708,
    "arm_joint6": -1.5708,
}
