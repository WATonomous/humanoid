"""Four-camera demo: one receive at a time, seen from four angles at once,
tiled 2x2 and streamed as MJPEG to a plain web page (and/or recorded to a
video). The page keeps the picture's aspect ratio at any window size.

  MUJOCO_GL=egl uv run scripts/demo_quad.py --checkpoint-file <model.pt> \\
      [--task Mjlab-Badminton-Receive-Student-PPO] [--port 8080] \\
      [--speed 0.5] [--record runs/demo.mp4 --seconds 40] \\
      [--preview runs/preview.png]
  # live: open http://localhost:8080 through an SSH tunnel;
  # scripts/slurm_demo.sh submits the job and prints the tunnel line.
  # Live frame rate adapts to the GPU (up to --fps) while the sim stays at
  # real time (or --speed times it); --record renders at a fixed rate.

Every rally is a fresh random serve from the launcher bank, so a run shows a
steady stream of different receives. Cameras: broadcast side view (whole
flight, serve to landing), behind the robot (the approach), the opponent's
view (the return coming over), and a high view (where it lands vs the serve
origin, marked on the floor). A status bar carries rally count, time,
hit/miss and the predicted landing error.
"""

import argparse
import io
import os
import sys
import threading
import time
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import aero
import badminton_mjlab  # noqa: F401  (registers the tasks)
from badminton_mjlab import mdp
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.viewer.offscreen_renderer import OffscreenRenderer
from mjlab.viewer.viewer_config import ViewerConfig

GOOD, BAD, WARN = (12, 163, 12), (208, 59, 59), (230, 160, 30)
INK, PANEL, GAP = (252, 252, 251), (11, 11, 11), (40, 40, 40)
WORLD = ViewerConfig.OriginType.WORLD

# MuJoCo free camera: azimuth 90 = looking along +y (from behind the robot
# toward the net), 270 = from the far court back at the robot, 180 = from the
# -x sideline. The arm stands at y = -2, the net at y = 0, serves come from
# y = 3..6.
CAMERAS = {
    "broadcast":    dict(azimuth=180.0, elevation=-14.0, distance=7.0,
                         lookat=(0.0, 0.6, 1.0)),
    "behind robot": dict(azimuth=90.0, elevation=-14.0, distance=4.0,
                         lookat=(-0.2, -1.0, 1.4)),
    "opponent":     dict(azimuth=270.0, elevation=-12.0, distance=5.0,
                         lookat=(-0.2, -1.6, 1.2)),
    "high":         dict(azimuth=90.0, elevation=-60.0, distance=9.5,
                         lookat=(0.0, 0.8, 0.0)),
}


class FrameFeed:
    """Latest JPEG frame, handed to every connected browser as MJPEG."""

    def __init__(self) -> None:
        self._jpeg = b""
        self._seq = 0
        self._cv = threading.Condition()

    def publish(self, jpeg: bytes) -> None:
        with self._cv:
            self._jpeg, self._seq = jpeg, self._seq + 1
            self._cv.notify_all()

    def wait_next(self, seen: int, timeout: float = 1.0):
        with self._cv:
            self._cv.wait_for(lambda: self._seq != seen, timeout=timeout)
            return self._jpeg, self._seq


PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<title>badminton receive demo</title><style>
html,body{margin:0;height:100%;background:#0b0b0b;overflow:hidden}
img{width:100vw;height:100vh;object-fit:contain;display:block}
</style></head><body><img src="/stream"></body></html>"""


def serve(feed: FrameFeed, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        jpeg, seen = feed.wait_next(seen)
                        if not jpeg:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                         + jpeg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)

    httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def load_policy(task: str, checkpoint: str, device: str):
    env_cfg = load_env_cfg(task)
    env_cfg.scene.num_envs = 1
    agent_cfg = load_rl_cfg(task)
    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner_cls = load_runner_cls(task) or MjlabOnPolicyRunner
    runner = runner_cls(env, asdict(agent_cfg), device=device)
    from rsl_rl.runners import DistillationRunner
    distilled = isinstance(runner, DistillationRunner)
    runner.load(checkpoint,
                load_cfg={"student": True} if distilled else {"actor": True},
                strict=True, map_location=device)
    return env, runner.get_inference_policy(device=device)


def make_renderers(uenv, width: int, height: int) -> list[OffscreenRenderer]:
    base = replace(uenv.cfg.viewer, width=width, height=height,
                   origin_type=WORLD, entity_name=None, body_name=None,
                   max_extra_envs=0, enable_shadows=True,
                   enable_reflections=True)
    out = []
    for cam in CAMERAS.values():
        r = OffscreenRenderer(model=uenv.sim.mj_model, cfg=replace(base, **cam),
                              scene=uenv.scene, sim_model=uenv.sim.model,
                              expanded_fields=uenv.sim.expanded_fields)
        r.initialize()
        out.append(r)
    return out


def label(frame: np.ndarray, text: str, font) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    box = draw.textbbox((6, 4), text, font=font)
    draw.rectangle((0, 0, box[2] + 6, box[3] + 4), fill=PANEL)
    draw.text((6, 4), text, fill=INK, font=font)
    return np.asarray(img)


def wall(frames: list[np.ndarray], status: str, colour, font, bar: int) -> np.ndarray:
    h, w = frames[0].shape[:2]
    gap = 4
    out = np.zeros((bar + 2 * h + gap, 2 * w + gap, 3), dtype=np.uint8)
    out[:] = GAP
    for i, f in enumerate(frames):
        r, c = divmod(i, 2)
        y, x = bar + r * (h + gap), c * (w + gap)
        out[y:y + h, x:x + w] = f
    img = Image.fromarray(out)
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, out.shape[1], bar), fill=PANEL)
    draw.text((10, (bar - font.size) // 2 - 1), status, fill=colour, font=font)
    return np.asarray(img)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="Mjlab-Badminton-Receive-Student-PPO")
    ap.add_argument("--checkpoint-file", required=True)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--panel-width", type=int, default=640)
    ap.add_argument("--panel-height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30,
                    help="frame-rate cap for the live page (the sim ticks at "
                         "50 Hz); the recording rate for --record")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback speed for the live page (0.5 = slow motion)")
    ap.add_argument("--record", default=None, help="video path (mp4)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this much sim time (default: run "
                         "until killed; with --record defaults to 40)")
    ap.add_argument("--no-viser", "--no-web", dest="no_web", action="store_true",
                    help="record only, do not start the web page")
    ap.add_argument("--debug", action="store_true",
                    help="print render/encode timings to stderr")
    ap.add_argument("--preview", default=None,
                    help="write one frame per camera at two moments of a "
                         "rally to this PNG and exit (camera tuning)")
    args = ap.parse_args()
    if args.record and args.seconds is None:
        args.seconds = 40.0
    if args.preview:
        args.no_web, args.seconds = True, 2.4

    env, policy = load_policy(args.task, args.checkpoint_file, args.device)
    uenv = env.unwrapped
    store = uenv._badminton
    renderers = make_renderers(uenv, args.panel_width, args.panel_height)
    font = ImageFont.load_default(size=max(14, args.panel_height // 24))
    bar = font.size + 14
    step_dt = uenv.step_dt
    render_every = max(1, round(1.0 / (args.fps * step_dt)))
    prm = aero.load_params()

    feed, httpd = None, None
    if not args.no_web:
        feed = FrameFeed()
        httpd = serve(feed, args.port)
        print(f"[demo] web page on port {args.port}", flush=True)
    writer = None
    if args.record:
        import imageio
        writer = imageio.get_writer(args.record, fps=args.fps,
                                    macro_block_size=1)

    # markers drawn into every camera: serve origin on the floor, and the
    # predicted landing point once the return is in the air
    marks = {"origin": None, "landing": None, "landing_ok": True}

    def draw_marks(vis) -> None:
        if marks["origin"] is not None:
            x, y = marks["origin"]
            vis.add_cylinder(np.array([x, y, 0.0]), np.array([x, y, 0.02]),
                             0.25, (0.2, 0.45, 1.0, 0.9))
        if marks["landing"] is not None:
            x, y = marks["landing"]
            c = (0.05, 0.75, 0.05, 0.9) if marks["landing_ok"] else (0.9, 0.6, 0.1, 0.9)
            vis.add_sphere(np.array([x, y, 0.08]), 0.08, c)

    rally, status, colour = 1, "serving...", INK
    obs = env.get_observations()
    marks["origin"] = tuple(store["p0_xy"][0].tolist())
    t_sim, tick, t_wall0 = 0.0, 0, time.perf_counter()
    last_render_wall = -1.0
    n_frames = 0
    previews = []

    def render_wall() -> np.ndarray:
        frames = []
        for name, r in zip(CAMERAS, renderers):
            r.update(uenv.sim.data, debug_vis_callback=draw_marks)
            frames.append(label(r.render(), name, font))
        return wall(frames, f"rally {rally}   {ep_t:4.1f} s   {status}",
                    colour, font, bar)

    try:
        with torch.no_grad():
            while args.seconds is None or t_sim < args.seconds:
                prev_hit = bool(store["hit"][0])
                obs, _, dones, _ = env.step(policy(obs))
                if bool(store["first"][0]) and not prev_hit:
                    pos, vel = mdp._shuttle_state(uenv)
                    xy, ok = mdp.predict_landing(pos[:1], vel[:1])
                    err = float((xy[0] - store["p0_xy"][0]).norm())
                    marks["landing"] = tuple(xy[0].tolist())
                    marks["landing_ok"] = bool(ok[0])
                    if bool(ok[0]):
                        status, colour = f"HIT   return lands {err:.1f} m from the serve origin", GOOD
                    else:
                        status, colour = "HIT   return short / into the net", WARN
                if bool(dones[0]):
                    if not prev_hit and not bool(store["hit"][0]):
                        status, colour = "MISS", BAD
                    status = f"last rally: {status}"
                    rally += 1
                    marks["origin"] = tuple(store["p0_xy"][0].tolist())
                    marks["landing"] = None
                t_sim += step_dt
                tick += 1
                ep_t = float(uenv.episode_length_buf[0]) * step_dt
                if 0.3 < ep_t < 0.4 and not bool(store["hit"][0]):
                    status, colour = "serving...", INK
                if writer is not None or args.preview:
                    # fixed cadence for files
                    if tick % render_every == 0:
                        img = render_wall()
                        if writer is not None:
                            writer.append_data(img)
                        if args.preview and (abs(ep_t - 1.0) < step_dt / 2
                                             or abs(ep_t - 2.0) < step_dt / 2):
                            previews.append(img)
                if feed is not None:
                    # live: keep the sim at real time (x speed); render a
                    # frame whenever the fps cap allows and we are not
                    # behind schedule, so the page gets as many frames as
                    # the GPU can draw
                    now = time.perf_counter()
                    lag = t_sim / args.speed - (now - t_wall0)
                    if now - last_render_wall >= 1.0 / args.fps and lag > -0.05:
                        t0 = time.perf_counter()
                        img = render_wall()
                        t1 = time.perf_counter()
                        buf = io.BytesIO()
                        Image.fromarray(img).save(
                            buf, "JPEG", quality=args.jpeg_quality)
                        feed.publish(buf.getvalue())
                        last_render_wall = time.perf_counter()
                        n_frames += 1
                        if args.debug and n_frames % 30 == 1:
                            print(f"[demo] tick {tick} t={t_sim:.1f}s lag={lag:+.3f}s "
                                  f"render {1e3*(t1-t0):.0f} ms encode "
                                  f"{1e3*(last_render_wall-t1):.0f} ms "
                                  f"{len(buf.getvalue())//1024} KB frames={n_frames}",
                                  file=sys.stderr, flush=True)
                    lag = t_sim / args.speed - (time.perf_counter() - t_wall0)
                    if lag > 0:
                        time.sleep(lag)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.close()
            print(f"[demo] wrote {args.record}", flush=True)
        if args.preview and previews:
            Image.fromarray(np.concatenate(previews, axis=0)).save(args.preview)
            print(f"[demo] wrote {args.preview}", flush=True)
        for r in renderers:
            r.close()
        if httpd is not None:
            httpd.shutdown()
        env.close()


if __name__ == "__main__":
    main()
