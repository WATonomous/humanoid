"""Four-screen demo: four independent random rallies, each rendered from its
own environment and tiled 2x2 into one picture that streams to a viser page
(and can be recorded to a video file).

  MUJOCO_GL=egl uv run scripts/demo_quad.py --checkpoint-file <model.pt> \\
      [--task Mjlab-Badminton-Receive-Student-PPO] [--port 8080] \\
      [--record runs/demo.mp4 --seconds 40]
  # then open http://localhost:8080 (SSH-tunnel the port from the cluster;
  # scripts/slurm_demo.sh does the job + tunnel line for you)

Every screen is a separate env drawing its own launcher-bank row on every
reset, so the four rallies are always different and change every ~3 s.
Each panel is captioned with its rally count, hit/miss, and where the
return is predicted to land relative to the launch origin.
"""

import argparse
import os
import sys
import time
from dataclasses import asdict, replace

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import badminton_mjlab  # noqa: F401  (registers the tasks)
from badminton_mjlab import mdp
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.viewer.offscreen_renderer import OffscreenRenderer

N_SCREENS = 4
GOOD, BAD, WARN = (12, 163, 12), (208, 59, 59), (230, 160, 30)
INK, PANEL = (252, 252, 251), (11, 11, 11)


def load_policy(task: str, checkpoint: str, device: str):
    env_cfg = load_env_cfg(task)
    env_cfg.scene.num_envs = N_SCREENS
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
    # one renderer per env; max_extra_envs=0 so each screen shows only its
    # own robot even though all four share the court in the sim world
    base = replace(uenv.cfg.viewer, width=width, height=height,
                   max_extra_envs=0, distance=4.5)
    out = []
    for i in range(N_SCREENS):
        r = OffscreenRenderer(model=uenv.sim.mj_model,
                              cfg=replace(base, env_idx=i),
                              scene=uenv.scene, sim_model=uenv.sim.model,
                              expanded_fields=uenv.sim.expanded_fields)
        r.initialize()
        out.append(r)
    return out


def caption(frame: np.ndarray, text: str, colour, font) -> np.ndarray:
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    pad = 6
    box = draw.textbbox((pad, pad), text, font=font)
    draw.rectangle((0, 0, box[2] + pad, box[3] + pad), fill=PANEL)
    draw.text((pad, pad), text, fill=colour, font=font)
    return np.asarray(img)


def tile(frames: list[np.ndarray], gap: int = 4) -> np.ndarray:
    h, w = frames[0].shape[:2]
    wall = np.full((2 * h + gap, 2 * w + gap, 3), 40, dtype=np.uint8)
    for i, f in enumerate(frames):
        r, c = divmod(i, 2)
        y, x = r * (h + gap), c * (w + gap)
        wall[y:y + h, x:x + w] = f
    return wall


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="Mjlab-Badminton-Receive-Student-PPO")
    ap.add_argument("--checkpoint-file", required=True)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--panel-width", type=int, default=480)
    ap.add_argument("--panel-height", type=int, default=360)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--record", default=None, help="video path (mp4)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this much sim time (default: run "
                         "until killed; with --record defaults to 40)")
    ap.add_argument("--no-viser", action="store_true",
                    help="record only, do not start the web page")
    args = ap.parse_args()
    if args.record and args.seconds is None:
        args.seconds = 40.0

    env, policy = load_policy(args.task, args.checkpoint_file, args.device)
    uenv = env.unwrapped
    store = uenv._badminton
    renderers = make_renderers(uenv, args.panel_width, args.panel_height)
    font = ImageFont.load_default(size=max(14, args.panel_height // 22))
    step_dt = uenv.step_dt
    render_every = max(1, round(1.0 / (args.fps * step_dt)))

    server = None
    if not args.no_viser:
        import viser
        server = viser.ViserServer(host="0.0.0.0", port=args.port)
        server.gui.add_markdown(
            "**Badminton receive - four random rallies**  \n"
            "Each screen is an independent simulation drawing a new random "
            "serve every rally. Caption: rally count, hit/miss, predicted "
            "landing distance from the serve origin.")
        print(f"[demo] viser on port {args.port}")
    writer = None
    if args.record:
        import imageio
        writer = imageio.get_writer(args.record, fps=args.fps,
                                    macro_block_size=1)

    rally = np.zeros(N_SCREENS, dtype=int)
    status = ["serving..."] * N_SCREENS
    colour = [INK] * N_SCREENS
    obs = env.get_observations()
    t_sim, tick, t_wall0 = 0.0, 0, time.perf_counter()
    try:
        with torch.no_grad():
            while args.seconds is None or t_sim < args.seconds:
                prev_hit = store["hit"].clone()
                obs, _, dones, _ = env.step(policy(obs))
                first = store["first"] & ~prev_hit
                if bool(first.any()):
                    pos, vel = mdp._shuttle_state(uenv)
                    xy, ok = mdp.predict_landing(pos[first], vel[first])
                    err = (xy - store["p0_xy"][first]).norm(dim=-1)
                    for j, i in enumerate(first.nonzero().flatten().tolist()):
                        if bool(ok[j]):
                            status[i] = (f"HIT  return lands {float(err[j]):.1f} m "
                                         f"from origin")
                            colour[i] = GOOD
                        else:
                            status[i], colour[i] = "HIT  return short / into net", WARN
                for i in dones.nonzero().flatten().tolist():
                    if not bool(prev_hit[i]):
                        status[i], colour[i] = "MISS", BAD
                    rally[i] += 1
                t_sim += step_dt
                tick += 1
                if tick % render_every == 0:
                    frames = []
                    for i, r in enumerate(renderers):
                        r.update(uenv.sim.data)
                        f = r.render()
                        ep_t = float(uenv.episode_length_buf[i]) * step_dt
                        # the last verdict stays up until the next serve is
                        # clearly in flight, then the caption resets
                        if 0.3 < ep_t < 0.4 and not bool(store["hit"][i]):
                            status[i], colour[i] = "serving...", INK
                        frames.append(caption(
                            f, f"screen {i + 1}   rally {rally[i] + 1}   "
                            f"{ep_t:4.1f} s   {status[i]}", colour[i], font))
                    wall = tile(frames)
                    if server is not None:
                        server.scene.set_background_image(
                            wall, format="jpeg", jpeg_quality=80)
                    if writer is not None:
                        writer.append_data(wall)
                if server is not None:  # real-time pacing for the live page
                    lag = t_sim - (time.perf_counter() - t_wall0)
                    if lag > 0:
                        time.sleep(lag)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.close()
            print(f"[demo] wrote {args.record}")
        for r in renderers:
            r.close()
        env.close()


if __name__ == "__main__":
    main()
