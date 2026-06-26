#!/usr/bin/env bash
# Launch the quest_teleop_bridge relay inside the *teleop* container.
#
# Run this from the repo root (on the host):
#   ./watod -t teleop
#
# Then inside the teleop container:
#   cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
#   ./run_relay.sh

set -e
source /root/ament_ws/install/setup.bash
exec python3 "$(dirname "$(realpath "$0")")/quest_teleop_bridge.py" "$@"
