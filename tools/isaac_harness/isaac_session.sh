#!/usr/bin/env bash
# Start/stop the persistent Isaac Sim session daemon (isaac_session_daemon.py).
# Once running, drive it with isaac_session_client.py — each command costs a
# fraction of a second against the warm session, instead of the ~15-40s a
# fresh Isaac Sim launch costs with the one-shot isaac_harness.sh scripts.
# Use this when iterating repeatedly on the same scene; use the one-shot
# scripts for a single check.
#
# Usage:
#   isaac_session.sh start [--timeout SECONDS]
#   isaac_session.sh stop
#   isaac_session.sh status
#
# Env vars: ISAAC_CONTAINER, ISAACLAB_SH_PATH (same as isaac_harness.sh)

set -euo pipefail

CONTAINER="${ISAAC_CONTAINER:-watod_hy-simulation_isaac_dev-1}"
ISAACLAB_SH="${ISAACLAB_SH_PATH:-/workspace/isaaclab/isaaclab.sh}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QUEUE_DIR_HOST="$TOOLS_DIR/.session"
QUEUE_DIR_CONTAINER="/workspace/humanoid/tools/isaac_harness/.session"
LOG_PATH="$QUEUE_DIR_HOST/daemon.log"
LOG_PATH_CONTAINER="$QUEUE_DIR_CONTAINER/daemon.log"

ACTION="${1:-}"
TIMEOUT=180
if [[ "${2:-}" == "--timeout" ]]; then TIMEOUT="$3"; fi

case "$ACTION" in
  start)
    mkdir -p "$QUEUE_DIR_HOST/cmds" "$QUEUE_DIR_HOST/responses" "$QUEUE_DIR_HOST/shots"
    rm -f "$QUEUE_DIR_HOST"/cmds/*.cmd.json "$QUEUE_DIR_HOST"/responses/*.response.json 2>/dev/null || true

    PREV_PID="$(docker exec "$CONTAINER" bash -c "pgrep -f '_isaac_sim/kit/python/bin/python3'" 2>/dev/null | tail -1 || true)"
    if [[ -n "${PREV_PID:-}" ]]; then
      echo "[session] stopping previous Isaac Sim process (pid $PREV_PID) gracefully"
      docker exec "$CONTAINER" bash -c "kill -SIGINT $PREV_PID" || true
      for _ in $(seq 1 15); do
        docker exec "$CONTAINER" bash -c "kill -0 $PREV_PID" 2>/dev/null || break
        sleep 1
      done
    fi

    echo "[session] starting daemon (log=$LOG_PATH)"
    docker exec -d "$CONTAINER" bash -c \
      "cd /workspace/humanoid && PYTHONUNBUFFERED=1 '$ISAACLAB_SH' -p tools/isaac_harness/isaac_session_daemon.py --queue-dir '$QUEUE_DIR_CONTAINER' > '$LOG_PATH_CONTAINER' 2>&1"

    echo "[session] waiting for DAEMON_READY (timeout ${TIMEOUT}s)"
    ELAPSED=0
    while [[ $ELAPSED -lt $TIMEOUT ]]; do
      if grep -q "DAEMON_READY" "$LOG_PATH" 2>/dev/null; then
        echo "[session] ready"
        exit 0
      fi
      if grep -qi -E "Traceback|Segmentation fault" "$LOG_PATH" 2>/dev/null; then
        echo "[session] daemon crashed on startup — last 30 log lines:" >&2
        tail -30 "$LOG_PATH" >&2
        exit 1
      fi
      sleep 2
      ELAPSED=$((ELAPSED + 2))
    done
    echo "[session] timed out waiting for daemon to start" >&2
    tail -30 "$LOG_PATH" >&2
    exit 1
    ;;

  stop)
    echo '{"cmd": "shutdown"}' | python3 -c "
import json, sys
sys.path.insert(0, '$TOOLS_DIR')
from isaac_session_client import send_command
print(send_command(json.load(sys.stdin), timeout=15))
"
    ;;

  status)
    PID="$(docker exec "$CONTAINER" bash -c "pgrep -f 'isaac_session_daemon.py'" 2>/dev/null | tail -1 || true)"
    if [[ -n "${PID:-}" ]]; then
      echo "[session] daemon running (pid $PID)"
    else
      echo "[session] no daemon running"
    fi
    ;;

  *)
    echo "Usage: $0 {start [--timeout SECONDS]|stop|status}" >&2
    exit 1
    ;;
esac
