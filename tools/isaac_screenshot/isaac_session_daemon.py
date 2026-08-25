"""Persistent Isaac Sim session daemon.

Launches Isaac Sim once (headless) and keeps it running, reading JSON
commands from a queue directory and writing JSON responses — instead of the
relaunch-per-shot model of isaac_screenshot.sh / isaac_multiview_screenshot.sh.
A fresh Isaac Sim launch costs ~15-40s; against an already-running session,
each command (spawn an object, move the camera, take a screenshot, query a
pose) costs a fraction of a second. Use this when iterating repeatedly on the
same scene; use the one-shot scripts when you just need a single check.

Run from inside the sim container (see isaac_session.sh for the host-side
launcher that does this for you):

    cd /workspace && PYTHONUNBUFFERED=1 /workspace/isaaclab/isaaclab.sh -p \
        tools/isaac_screenshot/isaac_session_daemon.py --queue-dir /workspace/.isaac_session

The queue directory (shared with the host via the repo's bind mount — no
docker cp needed) has two subdirs:
  cmds/      — host writes <id>.cmd.json here (atomically: write to .tmp, rename)
  responses/ — daemon writes <id>.response.json here

Command shapes (all fields besides "id" and "cmd" are command-specific):
  {"id": "...", "cmd": "ping"}
  {"id": "...", "cmd": "spawn_primitive", "name": "Obj1", "shape": "cuboid",
   "size": [0.1,0.1,0.1], "pos": [0,0,0.5], "color": [1,0,0], "dynamic": true}
  {"id": "...", "cmd": "spawn_usd", "name": "Robot", "usd_path": "...",
   "pos": [0,0,0], "rot": [1,0,0,0], "articulation": true}
  {"id": "...", "cmd": "remove", "name": "Obj1"}
  {"id": "...", "cmd": "set_pose", "name": "Obj1", "pos": [0,0,1], "rot": [1,0,0,0]}
  {"id": "...", "cmd": "query", "name": "Obj1"}
  {"id": "...", "cmd": "list"}
  {"id": "...", "cmd": "step", "n": 30}
  {"id": "...", "cmd": "screenshot", "eye": [1,1,1], "target": [0,0,0],
   "out_path": "/workspace/.isaac_session/shots/foo.png"}
  {"id": "...", "cmd": "bbox", "name": "Obj1"}
  {"id": "...", "cmd": "overlap", "name_a": "Obj1", "name_b": "Table"}
  {"id": "...", "cmd": "distance", "name_a": "Obj1", "name_b": "Obj2"}
  {"id": "...", "cmd": "scene_tree"}
  {"id": "...", "cmd": "shutdown"}

Structured introspection (bbox/overlap/distance/scene_tree) exists so the
scene can be checked programmatically — "is the rack actually resting on the
table", "are these two objects interpenetrating" — instead of only being
checkable by eyeballing a screenshot. bbox/scene_tree work on any prim on the
stage (raw USD), not just objects this daemon spawned and is tracking.

Response shape: {"id": "...", "ok": true, ...} or {"id": "...", "ok": false, "error": "..."}
"""

import argparse
import json
import os
import time
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--queue-dir", type=str, required=True)
parser.add_argument("--poll-interval", type=float, default=0.1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.sim import SimulationContext

CMDS_DIR = os.path.join(args_cli.queue_dir, "cmds")
RESP_DIR = os.path.join(args_cli.queue_dir, "responses")
os.makedirs(CMDS_DIR, exist_ok=True)
os.makedirs(RESP_DIR, exist_ok=True)

sim_cfg = sim_utils.SimulationCfg(dt=0.01)
sim = SimulationContext(sim_cfg)
sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.0])

cfg_ground = sim_utils.GroundPlaneCfg()
cfg_ground.func("/World/defaultGroundPlane", cfg_ground, translation=(0.0, 0.0, 0.0))
cfg_light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.9))
cfg_light.func("/World/Light", cfg_light)

sim.reset()

# name -> RigidObject | Articulation
objects: dict = {}
_object_is_dynamic: dict = {}  # name -> bool, for the bbox staleness warning

_PRIMITIVE_SPAWNERS = {
    "cuboid": sim_utils.CuboidCfg,
    "sphere": sim_utils.SphereCfg,
    "cylinder": sim_utils.CylinderCfg,
    "cone": sim_utils.ConeCfg,
}

_camera = None


def _get_camera() -> Camera:
    global _camera
    if _camera is None:
        cam_cfg = CameraCfg(
            prim_path="/World/SessionCamera",
            update_period=0,
            height=720,
            width=1280,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 20.0)),
        )
        _camera = Camera(cfg=cam_cfg)
        sim.reset()
        for _ in range(5):
            sim.step()
            simulation_app.update()
    return _camera


def _shape_kwargs(shape: str, size) -> dict:
    if shape == "sphere":
        return {"radius": size[0]}
    if shape == "cylinder" or shape == "cone":
        return {"radius": size[0], "height": size[1]}
    return {"size": tuple(size)}


def handle_spawn_primitive(cmd: dict) -> dict:
    name = cmd["name"]
    if name in objects:
        return {"ok": False, "error": f"name '{name}' already exists"}
    shape = cmd.get("shape", "cuboid")
    size = cmd.get("size", [0.1, 0.1, 0.1])
    color = cmd.get("color", [0.8, 0.8, 0.8])
    dynamic = cmd.get("dynamic", True)
    spawn_kwargs = _shape_kwargs(shape, size)
    spawn_kwargs["visual_material"] = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color))
    # RigidObject requires USD RigidBodyAPI on the prim regardless of whether
    # it's meant to move — "static" means kinematic (still a rigid body, just
    # not driven by physics forces), not "skip rigid_props entirely". Skipping
    # rigid_props leaves no RigidBodyAPI applied and RigidObject's
    # _initialize_impl() throws "Failed to find a rigid body".
    spawn_kwargs["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=not dynamic,
        kinematic_enabled=not dynamic,
        solver_position_iteration_count=16,
        solver_velocity_iteration_count=1,
    )
    spawn_kwargs["mass_props"] = sim_utils.MassPropertiesCfg(mass=cmd.get("mass", 0.1))
    spawn_kwargs["collision_props"] = sim_utils.CollisionPropertiesCfg()
    spawn_cfg = _PRIMITIVE_SPAWNERS[shape](**spawn_kwargs)
    obj_cfg = RigidObjectCfg(
        prim_path=f"/World/{name}",
        spawn=spawn_cfg,
        init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(cmd.get("pos", [0, 0, 0]))),
    )
    try:
        objects[name] = RigidObject(cfg=obj_cfg)
        _object_is_dynamic[name] = dynamic
        sim.reset()
    except Exception:
        # don't leave a half-initialized object in the registry — a broken
        # entry there crashes every later command that touches it (query,
        # step, distance, ...) with an unrelated-looking AttributeError.
        objects.pop(name, None)
        _object_is_dynamic.pop(name, None)
        raise
    return {"ok": True}


def handle_spawn_usd(cmd: dict) -> dict:
    name = cmd["name"]
    if name in objects:
        return {"ok": False, "error": f"name '{name}' already exists"}
    pos = tuple(cmd.get("pos", [0, 0, 0]))
    rot = tuple(cmd.get("rot", [1, 0, 0, 0]))
    try:
        if cmd.get("articulation", False):
            obj_cfg = ArticulationCfg(
                prim_path=f"/World/{name}",
                spawn=sim_utils.UsdFileCfg(usd_path=cmd["usd_path"]),
                init_state=ArticulationCfg.InitialStateCfg(pos=pos, rot=rot),
            )
            objects[name] = Articulation(cfg=obj_cfg)
        else:
            obj_cfg = RigidObjectCfg(
                prim_path=f"/World/{name}",
                spawn=sim_utils.UsdFileCfg(usd_path=cmd["usd_path"]),
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot),
            )
            objects[name] = RigidObject(cfg=obj_cfg)
        # Whether a USD-file rigid body is actually kinematic depends on
        # what's baked into the asset itself, which we don't inspect here —
        # conservatively assume dynamic (i.e. bbox may go stale) unless
        # proven otherwise.
        _object_is_dynamic[name] = True
        sim.reset()
    except Exception:
        objects.pop(name, None)
        _object_is_dynamic.pop(name, None)
        raise
    return {"ok": True}


def handle_remove(cmd: dict) -> dict:
    name = cmd["name"]
    if name not in objects:
        return {"ok": False, "error": f"no such object '{name}'"}
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(f"/World/{name}")
    del objects[name]
    _object_is_dynamic.pop(name, None)
    sim.reset()
    return {"ok": True}


def handle_set_pose(cmd: dict) -> dict:
    name = cmd["name"]
    if name not in objects:
        return {"ok": False, "error": f"no such object '{name}'"}
    obj = objects[name]
    pos = cmd.get("pos")
    rot = cmd.get("rot")
    cur_pos = obj.data.root_pos_w.clone()
    cur_rot = obj.data.root_quat_w.clone()
    if pos is not None:
        cur_pos[:] = torch.tensor(pos, device=cur_pos.device)
    if rot is not None:
        cur_rot[:] = torch.tensor(rot, device=cur_rot.device)
    obj.write_root_pose_to_sim(torch.cat([cur_pos, cur_rot], dim=-1))
    return {"ok": True}


def handle_query(cmd: dict) -> dict:
    name = cmd["name"]
    if name not in objects:
        return {"ok": False, "error": f"no such object '{name}'"}
    obj = objects[name]
    return {
        "ok": True,
        "pos": obj.data.root_pos_w[0].tolist(),
        "rot": obj.data.root_quat_w[0].tolist(),
    }


def handle_list(cmd: dict) -> dict:
    return {"ok": True, "names": list(objects.keys())}


def _prim_path_for(name: str) -> str:
    """Resolve a name to a /World prim path — works for tracked objects
    (spawned via spawn_primitive/spawn_usd) and untracked static prims
    (ground, light, or anything spawned outside the daemon's own registry)
    alike, since bbox uses raw USD, not the RigidObject/Articulation wrapper.
    """
    return f"/World/{name}"


def handle_bbox(cmd: dict) -> dict:
    """World-space AABB of a prim via raw USD (UsdGeom.BBoxCache).

    KNOWN LIMITATION: for a dynamic (non-kinematic) tracked object that has
    moved under physics since spawn, this can return a STALE, spawn-time
    bounding box. Physics for RigidObject/Articulation runs on a tensor/
    Fabric layer that does not reliably sync back to the USD stage's xform
    attributes every step, so a raw-USD bbox read can lag behind reality —
    confirmed by comparing against query()'s tensor-sourced pose, which
    stayed correct while this drifted. An attempted fix (reconstruct the
    world bbox from a cached local bbox + the live tracked pose) produced
    numbers that didn't match query() either and was reverted rather than
    ship something subtly wrong — needs a real fix, not a guess.
    For a moving tracked object, treat query()'s pos/rot as authoritative
    and use bbox mainly for STATIC prims (ground, table, anything kinematic
    or not yet stepped) until this is properly fixed.
    """
    from pxr import UsdGeom

    import omni.usd

    name = cmd["name"]
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(_prim_path_for(name))
    if not prim.IsValid():
        return {"ok": False, "error": f"no prim at /World/{name}"}

    bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_], useExtentsHint=True)
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    result = {"ok": True, "min": list(rng.GetMin()), "max": list(rng.GetMax())}
    if _object_is_dynamic.get(name):
        result["warning"] = (
            "bbox may be stale for a moved dynamic object — see query() for the authoritative live pose"
        )
    return result


def handle_overlap(cmd: dict) -> dict:
    """AABB overlap check between two named prims — a cheap proxy for
    'are these two things colliding/touching', not exact mesh-level
    collision, but usually enough to answer 'is the rack actually on the
    table' / 'are these two vials interpenetrating' without eyeballing a
    screenshot.
    """
    a = handle_bbox({"name": cmd["name_a"]})
    if not a["ok"]:
        return a
    b = handle_bbox({"name": cmd["name_b"]})
    if not b["ok"]:
        return b
    overlap = all(a["min"][i] <= b["max"][i] and b["min"][i] <= a["max"][i] for i in range(3))
    return {"ok": True, "overlap": overlap, "bbox_a": a, "bbox_b": b}


def handle_distance(cmd: dict) -> dict:
    """Center-to-center distance between two named objects. Uses tracked
    pose data when available (cheaper, already up to date after step()),
    falls back to bbox center for untracked/static prims.
    """
    def _center(name: str):
        if name in objects:
            return objects[name].data.root_pos_w[0].tolist()
        bbox = handle_bbox({"name": name})
        if not bbox["ok"]:
            return None
        return [(bbox["min"][i] + bbox["max"][i]) / 2 for i in range(3)]

    pos_a = _center(cmd["name_a"])
    if pos_a is None:
        return {"ok": False, "error": f"no such prim '{cmd['name_a']}'"}
    pos_b = _center(cmd["name_b"])
    if pos_b is None:
        return {"ok": False, "error": f"no such prim '{cmd['name_b']}'"}

    dist = sum((pos_a[i] - pos_b[i]) ** 2 for i in range(3)) ** 0.5
    return {"ok": True, "distance": dist, "pos_a": pos_a, "pos_b": pos_b}


def handle_scene_tree(cmd: dict) -> dict:
    """Full /World prim hierarchy with types — includes static prims (ground,
    light, anything spawned outside the daemon's own tracked-object registry)
    that `list` doesn't see, since `list` only reports objects this daemon
    instance spawned and is tracking Python-side.
    """
    import omni.usd
    from pxr import Usd

    stage = omni.usd.get_context().get_stage()
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return {"ok": False, "error": "no /World prim on stage"}

    tree = []
    for prim in Usd.PrimRange(world):
        tree.append({
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "tracked": prim.GetPath().name in objects,
        })
    return {"ok": True, "tree": tree}


def handle_step(cmd: dict) -> dict:
    n = cmd.get("n", 1)
    dt = sim.get_physics_dt()
    for _ in range(n):
        sim.step()
        simulation_app.update()
        # RigidObject/Articulation cache their tensors — like the Camera
        # sensor, they need an explicit update() each step to stay in sync
        # with the physics view, or query() silently returns stale data.
        for obj in objects.values():
            obj.update(dt)
    return {"ok": True}


def handle_screenshot(cmd: dict) -> dict:
    camera = _get_camera()
    eye = cmd.get("eye", [2.0, 2.0, 2.0])
    target = cmd.get("target", [0.0, 0.0, 0.0])
    eyes = torch.tensor([eye], dtype=torch.float32, device=camera.device)
    targets = torch.tensor([target], dtype=torch.float32, device=camera.device)
    camera.set_world_poses_from_view(eyes, targets)
    for _ in range(10):
        sim.step()
        simulation_app.update()
    camera.update(dt=0.0, force_recompute=True)

    import numpy as np
    from PIL import Image

    rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
    out_path = cmd["out_path"]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    Image.fromarray(rgb).save(out_path)
    return {"ok": True, "out_path": out_path}


def handle_ping(cmd: dict) -> dict:
    return {"ok": True, "pong": True}


HANDLERS = {
    "ping": handle_ping,
    "spawn_primitive": handle_spawn_primitive,
    "spawn_usd": handle_spawn_usd,
    "remove": handle_remove,
    "set_pose": handle_set_pose,
    "query": handle_query,
    "list": handle_list,
    "step": handle_step,
    "screenshot": handle_screenshot,
    "bbox": handle_bbox,
    "overlap": handle_overlap,
    "distance": handle_distance,
    "scene_tree": handle_scene_tree,
}


def process_command(cmd: dict) -> dict:
    cmd_type = cmd.get("cmd")
    handler = HANDLERS.get(cmd_type)
    if handler is None:
        return {"ok": False, "error": f"unknown command '{cmd_type}'"}
    try:
        return handler(cmd)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"{exc}\n{traceback.format_exc()}"}


def write_response(cmd_id: str, response: dict) -> None:
    response["id"] = cmd_id
    tmp_path = os.path.join(RESP_DIR, f"{cmd_id}.response.json.tmp")
    final_path = os.path.join(RESP_DIR, f"{cmd_id}.response.json")
    with open(tmp_path, "w") as f:
        json.dump(response, f)
    os.rename(tmp_path, final_path)


print("DAEMON_READY", flush=True)

running = True
while running and simulation_app.is_running():
    cmd_files = sorted(f for f in os.listdir(CMDS_DIR) if f.endswith(".cmd.json"))
    if not cmd_files:
        simulation_app.update()
        time.sleep(args_cli.poll_interval)
        continue

    for fname in cmd_files:
        path = os.path.join(CMDS_DIR, fname)
        try:
            with open(path) as f:
                cmd = json.load(f)
        except (json.JSONDecodeError, OSError):
            # still being written — try again next loop
            continue
        os.remove(path)

        cmd_id = cmd.get("id", fname)
        if cmd.get("cmd") == "shutdown":
            write_response(cmd_id, {"ok": True})
            running = False
            break

        response = process_command(cmd)
        write_response(cmd_id, response)

simulation_app.close()
