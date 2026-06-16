#!/usr/bin/env bash
# Launch CloudXR + Pink IK teleop for the bimanual arm.
#
# Use --gui for local debug without CloudXR.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_LAB="${ISAAC_LAB:-$HOME/robotics/IsaacLab-3.0}"

export PYTHONPATH="${SCRIPT_DIR}:${SCRIPT_DIR}/../Humanoid_Wato:${PYTHONPATH}"

MODE_ARGS=(--xr --headless)
if [[ "${1:-}" == "--gui" || "${WATO_GUI:-0}" == "1" ]]; then
  [[ "${1:-}" == "--gui" ]] && shift
  MODE_ARGS=(--visualizer kit --cloudxr_env none --no-auto_launch_cloudxr)
fi

exec "${ISAAC_LAB}/isaaclab.sh" -p \
  "${ISAAC_LAB}/scripts/environments/teleoperation/teleop_se3_agent.py" \
  --task Isaac-WatoBimanualTeleop-v0 \
  --external_callback quest_isaac_teleop.register_bimanual_tasks \
  --device cpu \
  "${MODE_ARGS[@]}" \
  "$@"
