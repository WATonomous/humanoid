"""Geometry constants for the generalized pick-and-place task.

Table slab, reachable workspace box, grasp goalset size, tray asset geometry,
and the gripper approach frame. All in the robot base frame (== env-local
frame; the robot sits at the env origin). Pure stdlib — safe to import outside
Isaac Sim.

History: this file was `wato_constants.py`. It used to also carry a copy of the
arm's joint limits / default pose / EE bodies so the (now-removed) cuRobo
pipeline could import them without pulling in isaaclab. cuRobo was archived in
PR #187 and every remaining importer runs inside Isaac Lab, so the arm half now
lives only in `pioneer_humanoid/pioneer_humanoid/bimanual_arm.py`.
"""
import math  # noqa: F401  (kept for downstream configs that do trig on these)
import os

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# --- Gripper approach frame ------------------------------------------------
# Fingertip-center offset in the link6l (wrist) frame. Constant for a
# synchronized gripper pair (q7l = -q8l); derived from the joint7l/joint8l URDF
# origins + the finger distal-tip locals in pioneer_humanoid.bimanual_arm
# (RIGHT_FINGER_DISTAL_TIP_LOCAL). Fingers extend along wrist -Y.
#   x: ((-0.016558 + 0.13211595) + (0.085558 - 0.13211595)) / 2
#   y: -0.10361 - 0.04057075
#   z: 0.004349 - 0.00434997
FINGERTIP_OFFSET_IN_WRIST = (0.0345, -0.14418075, 0.0)
# Tool approach axis in the wrist frame (wrist -> fingertips).
APPROACH_AXIS_IN_WRIST = (0.0, -1.0, 0.0)

# --- Reachable workspace for top-down grasps -----------------------------
# Validated by the (removed) validate_curobo_plan.py at 93.8% plan success over
# this box with 8 yaw candidates, table top at 0.05.
TABLE_TOP_Z = 0.05
WORKSPACE_X = (0.28, 0.43)
WORKSPACE_Y = (-0.32, -0.10)
HOVER_Z = (0.12, 0.22)
NUM_GRASP_YAWS = 8

# --- Table slab (matches the Isaac scene) --------------------------------
# Front edge clear of the robot column at x ~ 0.
TABLE_X_MIN = 0.18
TABLE_DIMS = (0.9, 1.2, 0.05)  # top surface at TABLE_TOP_Z

# --- Tray asset (place-into-tray demo mode) ------------------------------
# tray.usda is an open-top box; its local origin sits at a CORNER (mesh spans
# local x[0,0.20] y[0,0.16] z[0,0.03]), so the interior centre is offset from
# the prim origin by TRAY_LOCAL_CENTER. Geometry read from the asset; scaled
# uniformly by PlaceParams.tray_scale.
TRAY_USDA = os.path.join(REPO_ROOT, "assets", "lerobot", "so101_vial_task", "usd", "tray.usda")
TRAY_FOOTPRINT = (0.20, 0.16)        # local x,y extent at scale 1.0 [m]
TRAY_LOCAL_CENTER = (0.10, 0.08)     # interior centre in tray-local frame [m]
TRAY_INTERIOR_HALF = (0.075, 0.055)  # interior half-extent at scale 1.0 [m]
TRAY_FLOOR_LOCAL_Z = 0.005           # inner floor height above the prim origin [m]
TRAY_WALL_HEIGHT = 0.03              # rim height at scale 1.0 [m]
TRAY_WALL_THICK = 0.004              # wall thickness at scale 1.0 [m]
