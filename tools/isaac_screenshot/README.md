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

### Single screenshot, GUI mode

```bash
# your script must do: print("SCENE_READY", flush=True) once the scene is set up,
# and then keep the app alive (e.g. loop on simulation_app.is_running())
tools/isaac_screenshot/isaac_screenshot.sh /workspace/my_scene.py --marker SCENE_READY
```

Options: `--cwd DIR` (working directory inside the container, default:
dirname of the script — matters for scripts with package-relative imports),
`--marker TEXT` (default `SCENE_READY`), `--timeout SECONDS` (default 180),
`--out PNG_PATH` (default `./last_screenshot.png`).

The harness does not stop your script afterward — when you're done iterating,
stop it yourself with a graceful signal (see Gotchas below):

```bash
docker exec watod_hy-simulation_isaac_dev-1 bash -c 'kill -SIGINT <pid>'
```

### Multiple views in one launch, GUI mode

One view is rarely enough to judge object placement — this captures several
named camera angles from the *same* running process (no relaunching between
views, so physics/object placement is identical across shots):

```python
# in your scene script, after building the scene and sim.reset():
from multiview_capture import capture_views
capture_views(sim, simulation_app, [
    ("front", (1.5, -1.6, 1.4), (0.6, -0.2, 0.1)),
    ("top",   (0.6, -0.2, 2.2), (0.6, -0.2, 0.0)),
    ("side",  (0.6, -2.0, 0.6), (0.6, -0.2, 0.1)),
])
```

```bash
tools/isaac_screenshot/isaac_multiview_screenshot.sh /workspace/my_scene.py \
  --views front,top,side --out-dir /tmp/my_views
```

### Multiple views, headless mode (faster, no `$DISPLAY` needed)

```python
from multiview_capture import capture_views_headless
capture_views_headless(sim, simulation_app, [
    ("front", (1.5, -1.6, 1.4), (0.6, -0.2, 0.1)),
    ("top",   (0.6, -0.2, 2.2), (0.6, -0.2, 0.0)),
])
```

```bash
tools/isaac_screenshot/isaac_headless_multiview.sh /workspace/my_scene.py \
  --views front,top --out-dir /tmp/my_views
```

See "Two modes" below for when to reach for which.

All three drivers accept `--cwd DIR` / `--timeout SECONDS` / an output path
option, and `ISAAC_CONTAINER` / `ISAACLAB_SH_PATH` env vars if your container
name or `isaaclab.sh` location differs from the WATonomous default
(`watod_hy-simulation_isaac_dev-1`, `/workspace/isaaclab/isaaclab.sh`).

## Two modes: GUI (xwd) vs headless (Camera sensor)

There are two harnesses, and **both should stay available** — pick per task
rather than assuming one is strictly better:

| | `isaac_screenshot.sh` / `isaac_multiview_screenshot.sh` (GUI) | `isaac_headless_multiview.sh` (headless) |
|---|---|---|
| How it captures | Real GUI window + `xwd` + manual PNG decode | `isaaclab.sensors.Camera` reading `.data.output["rgb"]`, saved with PIL |
| Needs `$DISPLAY`/X11 | Yes | No |
| Measured time (3-view capture, this repo's `simulation_isaac_dev` container, warm shader cache) | ~42s | ~37s (~12% faster) |
| Output | Exactly what's on screen, including any UI you left open | Clean render, no UI chrome — usually the better default for "just show me the scene" |
| When to prefer it | Debugging something UI-specific, or confirming what a human would actually see live on the desktop | Everything else — faster, no display dependency, more reliable for repeated iteration |

The GUI path took real debugging to get to a *working* headless path: naively
calling `omni.kit.viewport.utility.capture_viewport_to_file()` in headless
mode silently no-ops — `get_active_viewport()` needs an actual GUI viewport
that doesn't exist headless even with `--enable_cameras`. Route through an
IsaacLab `Camera` sensor instead (same mechanism `pick_place_bimanual`'s task
cameras already use).

**If Claude is driving this and it's not obvious which mode fits, it should
ask which one you want** rather than silently picking one.

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
