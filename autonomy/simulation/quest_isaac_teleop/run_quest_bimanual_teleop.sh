#!/usr/bin/env bash
# Launch Quest armWithStand (wato_arm_v2) teleop (both arms = Differential
# IK/DLS fingertip tracking) inside the simulation_isaac container.
#
# Run this from the repo root (on the host):
#   ./watod -t simulation_isaac_dev
#
# Then inside the container:
#   cd /workspace/humanoid/autonomy/simulation/quest_isaac_teleop
#   ./run_quest_bimanual_teleop.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_LAB="${ISAAC_LAB:-/workspace/isaaclab}"

export PYTHONPATH="${SCRIPT_DIR}:${SCRIPT_DIR}/../Teleop/keyboard_based_teleoperation:${SCRIPT_DIR}/../Humanoid_Wato:${PYTHONPATH}"


# --device MUST stay "cuda": the box grasp holds on GPU PhysX and slips on CPU PhysX. Confirmed
# by A/B -- identical scene, identical friction (1.5/1.2) and enabled_self_collisions=False, only
# this flag differing. GPU and CPU PhysX are separate solver implementations and differ in contact
# /friction handling, so this is a behavioural difference, not a tuning one.
#
# Note this costs latency: min sim.step() is ~41ms on cuda vs ~31ms on cpu, because CPU-side reads
# of GPU-resident physics state (robot.data.*, IK diagnostics, scene.update()) need a device sync
# every step. That tradeoff is deliberate -- a working grasp beats a faster unusable one. Revisit
# only by fixing the grasp on CPU (gripper closed-width / stiffness / finger collision hulls),
# never by flipping this back on its own.
exec "${ISAAC_LAB}/isaaclab.sh" -p \
  "${SCRIPT_DIR}/run_quest_bimanual_teleop.py" \
  --device cuda \
  --enable_cameras \
  "$@"
