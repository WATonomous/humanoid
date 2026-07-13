#!/bin/bash
# Isaac Lab entrypoint. Container runs as root (needed by Omniverse), but the
# humanoid repo is bind-mounted from the host. Without fixing ownership, logs /
# checkpoints land as root (or nobody) and the host user cannot delete them.
#
# Pass HOST_UID / HOST_GID from compose (see modules/docker-compose.simulation_isaac.yaml).

HOST_UID="${HOST_UID:-1000}"
HOST_GID="${HOST_GID:-1000}"
HUMANOID_ROOT="${HUMANOID_ROOT:-/workspace/humanoid}"
# How often to reclaim ownership during long trainings (seconds).
OWNERSHIP_FIX_INTERVAL="${OWNERSHIP_FIX_INTERVAL:-30}"

fix_host_ownership() {
  if [ "$(id -u)" -ne 0 ]; then
    return 0
  fi
  if [ ! -d "$HUMANOID_ROOT" ]; then
    return 0
  fi
  # Prefer fast paths that training writes most; fall back to whole mount.
  local targets=(
    "$HUMANOID_ROOT/autonomy/simulation/Humanoid_Wato/HumanoidRL/logs"
    "$HUMANOID_ROOT/autonomy/simulation/Humanoid_Wato/HumanoidRL/outputs"
    "$HUMANOID_ROOT"
  )
  local t
  for t in "${targets[@]}"; do
    if [ -e "$t" ]; then
      chown -R "${HOST_UID}:${HOST_GID}" "$t" 2>/dev/null || true
    fi
  done
}

FIX_PID=""
if [ "$(id -u)" -eq 0 ]; then
  (
    while true; do
      sleep "$OWNERSHIP_FIX_INTERVAL"
      fix_host_ownership
    done
  ) &
  FIX_PID=$!
fi

cleanup() {
  if [ -n "$FIX_PID" ]; then
    kill "$FIX_PID" 2>/dev/null || true
    wait "$FIX_PID" 2>/dev/null || true
  fi
  fix_host_ownership
}
trap cleanup EXIT INT TERM

# Do not use exec — we need the EXIT trap to run so ownership is fixed when
# the container command (bash / train / sleep) finishes.
set +e
"$@"
status=$?
exit "$status"
