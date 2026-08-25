# isaac_screenshot

A Claude Code harness for iterating on Isaac Sim / IsaacLab scenes remotely —
built for driving this repo's sim dev loop from Claude Code's Remote Control
(e.g. from your phone), where there's no physical access to the monitor to
eyeball a scene yourself. Launches a script inside the sim container, waits
for it to report ready, then captures a real screenshot of the running GUI
window and saves it as a PNG that Claude can send back to you. Works equally
well for a human over plain SSH to a headless box, but the primary use case
is Claude driving this loop on your behalf: write a scene script, run it
through this harness, look at the screenshot, iterate.

## Why this exists

GNOME's screenshot APIs (`org.gnome.Shell.Screenshot` over D-Bus, and the
`org.freedesktop.portal.Screenshot` xdg-desktop-portal call) both require an
interactive click on the actual screen to confirm — neither works from a
remote/headless session. This uses the old-school `xwd` X11 tool instead,
which works non-interactively as long as there's a real X11 session (not pure
Wayland/XWayland — root-window/window capture is blocked there for security).

## Usage

```bash
# your script must do: print("SCENE_READY", flush=True) once the scene is set up,
# and then keep the app alive (e.g. loop on simulation_app.is_running())
tools/isaac_screenshot/isaac_screenshot.sh /workspace/my_scene.py --marker SCENE_READY
```

Options: `--cwd DIR` (working directory inside the container, default:
dirname of the script — matters for scripts with package-relative imports),
`--marker TEXT` (default `SCENE_READY`), `--timeout SECONDS` (default 180),
`--out PNG_PATH` (default `./last_screenshot.png`).

Override `ISAAC_CONTAINER` / `ISAACLAB_SH_PATH` env vars if your container
name or `isaaclab.sh` location differs from the WATonomous default
(`watod_hy-simulation_isaac_dev-1`, `/workspace/isaaclab/isaaclab.sh`).

The harness does not stop your script afterward — when you're done iterating,
stop it yourself with a graceful signal (see Gotchas below):

```bash
docker exec watod_hy-simulation_isaac_dev-1 bash -c 'kill -SIGINT <pid>'
```

## Gotchas this harness handles for you

- **No `--experience isaacsim.exp.full.kit`.** That flag is for an unrelated
  cloud/pip-only Isaac Sim setup. On this repo's Docker-based install it
  causes a deterministic native segfault in `omni.graph.core.plugin.so` on
  startup. Just run `isaaclab.sh -p <script>` with no `--experience` flag —
  it correctly auto-selects the local `isaaclab.python.kit` experience.
- **`$DISPLAY` is not always `:0`.** This can change across reboots depending
  on whether the desktop boots into Wayland (`:0`) or a plain X11 session
  (`:1`). The harness reads whatever `$DISPLAY` is set to in your shell —
  make sure it's correct before running (check with `w` if unsure).
- **Never `SIGKILL` (`kill -9`) an Isaac Sim process.** It gives the process
  no chance to release its GPU/graphics context, which can corrupt state
  badly enough that even a fresh launch afterward crashes instantly — in the
  worst case requiring a full machine reboot to recover. Always stop with
  plain `kill` (SIGTERM) or SIGINT (same as Ctrl+C), which this harness does
  automatically before relaunching.
- **Python stdout buffering.** `print()` output is fully buffered (not
  line-buffered) when stdout isn't a TTY, so a marker can sit invisible in
  memory for many minutes after the app is actually ready. The harness sets
  `PYTHONUNBUFFERED=1`; if you add prints of your own for debugging, use
  `flush=True` or the same env var.
