#!/usr/bin/env bash
# Headless variant of isaac_multiview_screenshot.sh. Renders straight to PNG
# files via Isaac Sim's own viewport capture (multiview_capture.capture_views_headless)
# instead of creating a GUI window and screenshotting it with xwd — no window,
# no X11, no $DISPLAY needed at all, no host-side XWD decode step. Should be
# noticeably faster than the GUI+xwd path once the GPU shader cache is warm.
#
# Your scene script must import and call
# multiview_capture.capture_views_headless() (see that file's docstring).
#
# Usage:
#   isaac_headless_multiview.sh <container_script_path> --views name1,name2,name3
#                                [--cwd DIR] [--timeout SECONDS] [--out-dir DIR]
#
# Env vars: ISAAC_CONTAINER, ISAACLAB_SH_PATH (same as isaac_harness.sh)

set -euo pipefail

CONTAINER="${ISAAC_CONTAINER:-watod_hy-simulation_isaac_dev-1}"
ISAACLAB_SH="${ISAACLAB_SH_PATH:-/workspace/isaaclab/isaaclab.sh}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PATH="/tmp/isaac_harness_$(date +%s).log"
CONTAINER_OUT_DIR="/tmp/isaac_multiview_out"

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

# No $DISPLAY / xhost needed at all in headless mode.

PREV_PID="$(docker exec "$CONTAINER" bash -c "pgrep -f '_isaac_sim/kit/python/bin/python3'" 2>/dev/null | tail -1 || true)"
if [[ -n "${PREV_PID:-}" ]]; then
  echo "[harness] stopping previous Isaac Sim process (pid $PREV_PID) gracefully"
  docker exec "$CONTAINER" bash -c "kill -SIGINT $PREV_PID" || true
  for _ in $(seq 1 15); do
    docker exec "$CONTAINER" bash -c "kill -0 $PREV_PID" 2>/dev/null || break
    sleep 1
  done
fi
docker exec "$CONTAINER" bash -c "rm -rf '$CONTAINER_OUT_DIR'"

echo "[harness] launching $SCRIPT_PATH headless (cwd=$CWD, log=$LOG_PATH, views=$VIEWS)"
START_TS=$(date +%s)
docker exec -d "$CONTAINER" bash -c \
  "cd '$CWD' && PYTHONUNBUFFERED=1 '$ISAACLAB_SH' -p '$SCRIPT_PATH' --headless --enable_cameras ${EXTRA_ARGS[*]:-} > '$LOG_PATH' 2>&1"

echo "[harness] waiting for 'ALL_VIEWS_SAVED' (timeout ${TIMEOUT}s)"
ELAPSED=0
STATUS="timeout"
while [[ $ELAPSED -lt $TIMEOUT ]]; do
  if docker exec "$CONTAINER" bash -c "grep -q 'ALL_VIEWS_SAVED' '$LOG_PATH'" 2>/dev/null; then
    STATUS="ready"; break
  fi
  if docker exec "$CONTAINER" bash -c "grep -qi -E 'Traceback|Segmentation fault' '$LOG_PATH'" 2>/dev/null; then
    STATUS="crashed"; break
  fi
  sleep 2
  ELAPSED=$((ELAPSED + 2))
done
END_TS=$(date +%s)

if [[ "$STATUS" != "ready" ]]; then
  echo "[harness] $STATUS — last 30 log lines:" >&2
  docker exec "$CONTAINER" bash -c "tail -30 '$LOG_PATH'" >&2
  exit 1
fi

echo "[harness] done in $((END_TS - START_TS))s, copying files out"
IFS=',' read -ra VIEW_NAMES <<< "$VIEWS"
for VIEW_NAME in "${VIEW_NAMES[@]}"; do
  docker cp "$CONTAINER:$CONTAINER_OUT_DIR/$VIEW_NAME.png" "$OUT_DIR/$VIEW_NAME.png"
  echo "$OUT_DIR/$VIEW_NAME.png"
done
