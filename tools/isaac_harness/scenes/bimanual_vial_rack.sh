#!/usr/bin/env bash
# Bimanual arm + table + vial rack + 3 vials scene, built through the
# session daemon (isaac_session.sh must already be started). Saved here so
# it can be rebuilt identically later instead of re-typing the spawn
# commands each time.
#
# Usage:
#   tools/isaac_harness/isaac_session.sh start       # if not already running
#   tools/isaac_harness/scenes/bimanual_vial_rack.sh
#
# Geometry matches the real pick_place_bimanual task's table
# (autonomy/simulation/pick_place_gen/wato_constants.py: TABLE_TOP_Z=0.05,
# TABLE_DIMS=(0.9,1.2,0.05), TABLE_X_MIN=0.18) and the so101_vial_task's
# rack/vial assets (assets/lerobot/so101_vial_task/usd/).
#
# Vial spawn Z is 0.12, not the vials' resting height (~0.1) — spawning
# exactly at rest height risks a small initial interpenetration with the
# table collision mesh, which can trigger a violent PhysX correction impulse
# and send the vial flying/tipping instead of settling normally (observed:
# one vial out of three tipped over within 5 physics steps when spawned at
# 0.1; the extra clearance avoids it).

set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
C="python3 $TOOLS_DIR/isaac_session_client.py"

VIAL_ASSETS="/workspace/humanoid/assets/lerobot/so101_vial_task/usd"

# Table top lowered from the real pick_place_bimanual task's TABLE_TOP_Z (0.05)
# to -0.25 for this visualization scene specifically — 0.05 (and an initial
# attempt at -0.10) both sat too high for comfortable manipulation reach
# relative to the robot's shoulder height. This is a demo-scene-only change;
# the real task's wato_constants.py is untouched.
TABLE_TOP_Z=-0.25

echo "[scene] table"
$C spawn_primitive --name Table --shape cuboid --size 0.9,1.2,0.05 \
  --pos 0.63,-0.2,-0.275 --color 0.35,0.25,0.15 --static

echo "[scene] robot (pioneer_bimanual_arm, real actuator config)"
$C spawn_bimanual_arm --name Robot --pos 0,0,0

echo "[scene] vial rack"
$C spawn_usd --name VialRack --usd-path "$VIAL_ASSETS/Vial_rack_simple.usda" --pos 0.5,-0.35,-0.25

echo "[scene] vials"
$C spawn_usd --name Vial1 --usd-path "$VIAL_ASSETS/Vial_opaque.usda" --pos 0.55,-0.05,-0.18
$C spawn_usd --name Vial2 --usd-path "$VIAL_ASSETS/Vial_opaque.usda" --pos 0.55,-0.15,-0.18
$C spawn_usd --name Vial3 --usd-path "$VIAL_ASSETS/Vial_opaque.usda" --pos 0.55,-0.25,-0.18

echo "[scene] settling physics"
$C step --n 60

echo "[scene] ready — spawned: Table, Robot, VialRack, Vial1, Vial2, Vial3"
