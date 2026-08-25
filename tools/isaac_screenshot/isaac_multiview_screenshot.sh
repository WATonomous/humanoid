#!/usr/bin/env bash
# Multi-view variant of isaac_screenshot.sh: launches a scene script once and
# captures several named camera angles from the SAME running process (no
# relaunching between views, so physics/object placement stays identical
# across shots — important when checking scene layout from multiple angles).
#
# Your scene script must import and call multiview_capture.capture_views()
# (see that file's docstring) after building the scene. This driver watches
# for each "VIEW_READY:<name>" marker, screenshots, then signals the script
# to advance to the next view.
#
# Usage:
#   isaac_multiview_screenshot.sh <container_script_path> --views name1,name2,name3
#                                  [--cwd DIR] [--timeout SECONDS] [--out-dir DIR]
#
#   --views NAMES       comma-separated view names, matching what the script
#                        passes to capture_views() — order matters
#   --out-dir DIR        where to save screenshots (default: this script's dir);
#                        each view saves as <out-dir>/<name>.png
#
# Env vars: ISAAC_CONTAINER, ISAACLAB_SH_PATH (same as isaac_screenshot.sh)
#
# Example:
#   isaac_multiview_screenshot.sh /workspace/my_scene.py --views front,top,side

set -euo pipefail

CONTAINER="${ISAAC_CONTAINER:-watod_hy-simulation_isaac_dev-1}"
ISAACLAB_SH="${ISAACLAB_SH_PATH:-/workspace/isaaclab/isaaclab.sh}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PATH="/tmp/isaac_harness_$(date +%s).log"
SIGNAL_DIR="/tmp/isaac_multiview_signals"

SCRIPT_PATH=""
CWD=""
VIEWS=""
TIMEOUT=180
OUT_DIR="$TOOLS_DIR"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cwd) CWD="$2"; shift 2 ;;
    --views) VIEWS="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --) shift; EXTRA_ARGS=("$@"); break ;;
    *)
      if [[ -z "$SCRIPT_PATH" ]]; then SCRIPT_PATH="$1"; shift;
      else echo "Unexpected arg: $1" >&2; exit 1; fi
      ;;
  esac
done

if [[ -z "$SCRIPT_PATH" || -z "$VIEWS" ]]; then
  echo "Usage: $0 <container_script_path> --views name1,name2,... [--cwd DIR] [--timeout SECONDS] [--out-dir DIR]" >&2
  exit 1
fi
[[ -z "$CWD" ]] && CWD="$(dirname "$SCRIPT_PATH")"
mkdir -p "$OUT_DIR"
IFS=',' read -ra VIEW_NAMES <<< "$VIEWS"

HOST_DISPLAY="${DISPLAY:-}"
if [[ -z "$HOST_DISPLAY" ]]; then
  echo "ERROR: host \$DISPLAY is not set. Check 'w' for the active session (:0 Wayland vs :1 X11)." >&2
  exit 1
fi
echo "[harness] host DISPLAY=$HOST_DISPLAY"
DISPLAY="$HOST_DISPLAY" xhost +local:root >/dev/null

PREV_PID="$(docker exec "$CONTAINER" bash -c "pgrep -f '_isaac_sim/kit/python/bin/python3'" 2>/dev/null | tail -1 || true)"
if [[ -n "${PREV_PID:-}" ]]; then
  echo "[harness] stopping previous Isaac Sim process (pid $PREV_PID) gracefully"
  docker exec "$CONTAINER" bash -c "kill -SIGINT $PREV_PID" || true
  for _ in $(seq 1 15); do
    docker exec "$CONTAINER" bash -c "kill -0 $PREV_PID" 2>/dev/null || break
    sleep 1
  done
fi
docker exec "$CONTAINER" bash -c "rm -rf '$SIGNAL_DIR' && mkdir -p '$SIGNAL_DIR'"

echo "[harness] launching $SCRIPT_PATH (cwd=$CWD, log=$LOG_PATH, views=$VIEWS)"
docker exec -d "$CONTAINER" bash -c \
  "cd '$CWD' && DISPLAY=$HOST_DISPLAY PYTHONUNBUFFERED=1 '$ISAACLAB_SH' -p '$SCRIPT_PATH' ${EXTRA_ARGS[*]:-} > '$LOG_PATH' 2>&1"

WIN_ID=""
for VIEW_NAME in "${VIEW_NAMES[@]}"; do
  MARKER="VIEW_READY:${VIEW_NAME}"
  echo "[harness] waiting for '$MARKER' (timeout ${TIMEOUT}s)"
  ELAPSED=0
  STATUS="timeout"
  while [[ $ELAPSED -lt $TIMEOUT ]]; do
    if docker exec "$CONTAINER" bash -c "grep -q '$MARKER' '$LOG_PATH'" 2>/dev/null; then
      STATUS="ready"; break
    fi
    if docker exec "$CONTAINER" bash -c "grep -qi -E 'Traceback|Segmentation fault' '$LOG_PATH'" 2>/dev/null; then
      STATUS="crashed"; break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
  done

  if [[ "$STATUS" != "ready" ]]; then
    echo "[harness] $STATUS on view '$VIEW_NAME' — last 30 log lines:" >&2
    docker exec "$CONTAINER" bash -c "tail -30 '$LOG_PATH'" >&2
    exit 1
  fi

  if [[ -z "$WIN_ID" ]]; then
    WIN_ID="$(DISPLAY="$HOST_DISPLAY" xwininfo -root -tree 2>/dev/null | grep -i "isaac sim" | head -1 | awk '{print $1}')"
    if [[ -z "$WIN_ID" ]]; then
      echo "[harness] ERROR: no Isaac Sim window found on DISPLAY=$HOST_DISPLAY" >&2
      exit 1
    fi
    echo "[harness] window: $WIN_ID"
  fi

  XWD_PATH="/tmp/isaac_harness_${VIEW_NAME}.xwd"
  DISPLAY="$HOST_DISPLAY" xwd -id "$WIN_ID" -silent -out "$XWD_PATH"
  python3 "$TOOLS_DIR/xwd_to_png.py" "$XWD_PATH" "$OUT_DIR/$VIEW_NAME.png"
  rm -f "$XWD_PATH"
  echo "[harness] saved $OUT_DIR/$VIEW_NAME.png"

  docker exec "$CONTAINER" bash -c "touch '$SIGNAL_DIR/${VIEW_NAME}.advance'"
done

echo "[harness] all views captured:"
for VIEW_NAME in "${VIEW_NAMES[@]}"; do
  echo "$OUT_DIR/$VIEW_NAME.png"
done
