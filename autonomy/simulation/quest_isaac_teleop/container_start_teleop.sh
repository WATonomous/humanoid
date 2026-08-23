#!/bin/bash
# Runs inside the simulation_isaac_dev container. VR -> Sim. Builds ROS
# packages, starts quest_teleop_node and webxr_server.py in the background
# (reusing them if already running -- see the README's "Check what's already
# running first"), then runs the Isaac Sim IK script in the foreground. Only
# processes THIS invocation started are stopped on exit; anything already
# running beforehand is left alone.
#
# Which IK script to launch varies by branch (e.g. run_quest_armv2_teleop.sh
# only exists where the armWithStand asset does) -- auto-detected below
# rather than hardcoded, so this doesn't silently break when switching
# branches. Prefers armv2 if present, falls back to bimanual.
set -e

WORKSPACE="/workspace/humanoid"
AMENT_WS="/root/ament_ws"

echo "[teleop] Building ROS packages..."
cd "$AMENT_WS"
source /opt/ros/humble/setup.bash
# Force deadsnakes Python 3.11 -- matches the colcon invocation baked into
# .bashrc in isaac_lab.Dockerfile. Non-interactive `docker exec` doesn't
# source .bashrc, so this must be run explicitly here.
colcon build --packages-select common_msgs quest_teleop \
    --cmake-args \
        -DPython3_EXECUTABLE=/usr/bin/python3.11 \
        -DPYTHON_EXECUTABLE=/usr/bin/python3.11
source install/setup.bash
echo "[teleop] Build complete."

TELEOP_PID=""
WEBXR_PID=""

cleanup() {
    echo ""
    echo "[teleop] Shutting down..."
    [ -n "$TELEOP_PID" ] && kill "$TELEOP_PID" 2>/dev/null || true
    [ -n "$WEBXR_PID" ] && kill "$WEBXR_PID" 2>/dev/null || true
    [ -n "$TELEOP_PID" ] && wait "$TELEOP_PID" 2>/dev/null || true
    [ -n "$WEBXR_PID" ] && wait "$WEBXR_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

if pgrep -f "quest_teleop_node" >/dev/null 2>&1; then
    echo "[teleop] quest_teleop_node already running -- reusing (won't be stopped on exit)."
else
    echo "[teleop] Starting quest_teleop_node..."
    ros2 run quest_teleop quest_teleop_node &
    TELEOP_PID=$!
fi

if pgrep -f "webxr_server.py" >/dev/null 2>&1; then
    echo "[teleop] webxr_server.py already running -- reusing (won't be stopped on exit)."
else
    echo "[teleop] Starting webxr_server.py..."
    python3 "${WORKSPACE}/autonomy/teleop/quest_teleop/scripts/webxr_server.py" &
    WEBXR_PID=$!
fi

sleep 2

cd "${WORKSPACE}/autonomy/simulation/quest_isaac_teleop"
if [ -x ./run_quest_armv2_teleop.sh ]; then
    LAUNCH_SCRIPT="./run_quest_armv2_teleop.sh"
    LAUNCH_LABEL="armWithStand / wato_arm_v2"
else
    echo "[teleop] ERROR: run_quest_armv2_teleop.sh"
    echo "[teleop]        found in $(pwd) on this branch. Nothing to launch."
    exit 1
fi

echo "[teleop] Starting Isaac Sim IK script (${LAUNCH_LABEL}, takes 1-3 min to load)..."
"$LAUNCH_SCRIPT" "$@"
