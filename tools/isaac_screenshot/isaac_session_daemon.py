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
  {"id": "...", "cmd": "shutdown"}

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
    if dynamic:
        spawn_kwargs["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
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
    objects[name] = RigidObject(cfg=obj_cfg)
    sim.reset()
    return {"ok": True}


def handle_spawn_usd(cmd: dict) -> dict:
    name = cmd["name"]
    if name in objects:
        return {"ok": False, "error": f"name '{name}' already exists"}
    pos = tuple(cmd.get("pos", [0, 0, 0]))
    rot = tuple(cmd.get("rot", [1, 0, 0, 0]))
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
    sim.reset()
    return {"ok": True}


def handle_remove(cmd: dict) -> dict:
    name = cmd["name"]
    if name not in objects:
        return {"ok": False, "error": f"no such object '{name}'"}
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(f"/World/{name}")
    del objects[name]
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
