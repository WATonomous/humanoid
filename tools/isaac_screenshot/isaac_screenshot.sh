#!/usr/bin/env bash
# Harness for the "launch an IsaacLab script, wait for it, screenshot it" loop
# needed for remote sim dev (no physical access to the monitor). Handles the
# gotchas hit doing this by hand: correct isaaclab.sh invocation (no
# --experience flag — that's for an unrelated cloud/pip-only Isaac Sim setup
# and causes a native crash on a proper local install), the host's actual
# current $DISPLAY (varies between :0 Wayland and :1 X11 across reboots),
# graceful process shutdown only (never SIGKILL Isaac Sim — it corrupts the
# GPU/graphics context and can require a full reboot to recover), Python
# output buffering (stdout is fully buffered when redirected to a file, so an
# unflushed print() marker can sit invisible for many minutes), and
# xwd->PNG screenshot capture (GNOME's screenshot D-Bus/portal APIs require
# an interactive click and don't work headless/remotely).
#
# Usage:
#   isaac_screenshot.sh <container_script_path> [--cwd DIR] [--marker TEXT]
#                        [--timeout SECONDS] [--out PNG_PATH] [-- extra args]
#
#   <container_script_path>  path INSIDE the container to the .py script to run
#   --cwd DIR                 working directory inside the container to launch
#                              from (default: dirname of the script)
#   --marker TEXT             string to watch for in the log meaning "ready to
#                              screenshot" (default: SCENE_READY) — have your
#                              script `print("SCENE_READY", flush=True)` after
#                              it's done setting up the scene
#   --timeout SECONDS         max seconds to wait for the marker (default: 180)
#   --out PNG_PATH            where to save the screenshot (default:
#                              ./last_screenshot.png next to this script)
#
# Env vars (override for a different setup):
#   ISAAC_CONTAINER   docker container name (default: watod_hy-simulation_isaac_dev-1)
#   ISAACLAB_SH_PATH  path to isaaclab.sh inside the container (default: /workspace/isaaclab/isaaclab.sh)
#
# Example:
#   isaac_screenshot.sh /workspace/my_scene.py --marker SCENE_READY
#
# Your script should keep running after printing the marker (e.g. loop on
# `simulation_app.is_running()`) so there's something live to screenshot; this
# harness does NOT stop it afterward — stop it yourself with a graceful signal
# when done iterating, e.g.:
#   docker exec $ISAAC_CONTAINER bash -c 'kill -SIGINT <pid>'

set -euo pipefail

CONTAINER="${ISAAC_CONTAINER:-watod_hy-simulation_isaac_dev-1}"
ISAACLAB_SH="${ISAACLAB_SH_PATH:-/workspace/isaaclab/isaaclab.sh}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PATH="/tmp/isaac_harness_$(date +%s).log"

SCRIPT_PATH=""
CWD=""
MARKER="SCENE_READY"
TIMEOUT=180
OUT="${TOOLS_DIR}/last_screenshot.png"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cwd) CWD="$2"; shift 2 ;;
    --marker) MARKER="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --) shift; EXTRA_ARGS=("$@"); break ;;
    *)
      if [[ -z "$SCRIPT_PATH" ]]; then SCRIPT_PATH="$1"; shift;
      else echo "Unexpected arg: $1" >&2; exit 1; fi
      ;;
  esac
done

if [[ -z "$SCRIPT_PATH" ]]; then
  echo "Usage: $0 <container_script_path> [--cwd DIR] [--marker TEXT] [--timeout SECONDS] [--out PNG_PATH]" >&2
  exit 1
fi
[[ -z "$CWD" ]] && CWD="$(dirname "$SCRIPT_PATH")"

HOST_DISPLAY="${DISPLAY:-}"
if [[ -z "$HOST_DISPLAY" ]]; then
  echo "ERROR: host \$DISPLAY is not set. Check 'w' for the active session (:0 Wayland vs :1 X11)." >&2
  exit 1
fi
echo "[harness] host DISPLAY=$HOST_DISPLAY"
DISPLAY="$HOST_DISPLAY" xhost +local:root >/dev/null

# Gracefully stop any prior Isaac Sim python process in the container first —
# never SIGKILL, always SIGTERM/SIGINT so the GPU/graphics context tears down
# cleanly (SIGKILL corrupts it and can require a full machine reboot to fix).
PREV_PID="$(docker exec "$CONTAINER" bash -c "pgrep -f '_isaac_sim/kit/python/bin/python3'" 2>/dev/null | tail -1 || true)"
if [[ -n "${PREV_PID:-}" ]]; then
  echo "[harness] stopping previous Isaac Sim process (pid $PREV_PID) gracefully"
  docker exec "$CONTAINER" bash -c "kill -SIGINT $PREV_PID" || true
  for _ in $(seq 1 15); do
    docker exec "$CONTAINER" bash -c "kill -0 $PREV_PID" 2>/dev/null || break
    sleep 1
  done
fi

echo "[harness] launching $SCRIPT_PATH (cwd=$CWD, log=$LOG_PATH)"
# PYTHONUNBUFFERED=1 matters: Python fully-buffers stdout when it's not a TTY
# (i.e. redirected to a file), so a print() marker can sit in memory for many
# minutes after the app is actually visibly ready, well past when the sim
# finished loading — the harness would see nothing in the log and misreport a
# timeout even though the scene is sitting there ready on screen.
docker exec -d "$CONTAINER" bash -c \
  "cd '$CWD' && DISPLAY=$HOST_DISPLAY PYTHONUNBUFFERED=1 '$ISAACLAB_SH' -p '$SCRIPT_PATH' ${EXTRA_ARGS[*]:-} > '$LOG_PATH' 2>&1"

echo "[harness] waiting for marker '$MARKER' (timeout ${TIMEOUT}s)"
ELAPSED=0
STATUS="timeout"
while [[ $ELAPSED -lt $TIMEOUT ]]; do
  if docker exec "$CONTAINER" bash -c "grep -q '$MARKER' '$LOG_PATH'" 2>/dev/null; then
    STATUS="ready"
    break
  fi
  if docker exec "$CONTAINER" bash -c "grep -qi -E 'Traceback|Segmentation fault' '$LOG_PATH'" 2>/dev/null; then
    STATUS="crashed"
    break
  fi
  sleep 3
  ELAPSED=$((ELAPSED + 3))
done

if [[ "$STATUS" != "ready" ]]; then
  echo "[harness] $STATUS — last 30 log lines:" >&2
  docker exec "$CONTAINER" bash -c "tail -30 '$LOG_PATH'" >&2
  exit 1
fi

echo "[harness] marker found, locating window"
WIN_ID="$(DISPLAY="$HOST_DISPLAY" xwininfo -root -tree 2>/dev/null | grep -i "isaac sim" | head -1 | awk '{print $1}')"
if [[ -z "$WIN_ID" ]]; then
  echo "[harness] ERROR: no Isaac Sim window found on DISPLAY=$HOST_DISPLAY (headless run, or window not yet mapped)" >&2
  exit 1
fi
echo "[harness] window: $WIN_ID"

XWD_PATH="/tmp/isaac_harness_$(date +%s).xwd"
DISPLAY="$HOST_DISPLAY" xwd -id "$WIN_ID" -silent -out "$XWD_PATH"
python3 "$TOOLS_DIR/xwd_to_png.py" "$XWD_PATH" "$OUT"
rm -f "$XWD_PATH"

echo "[harness] screenshot saved: $OUT"
echo "$OUT"
