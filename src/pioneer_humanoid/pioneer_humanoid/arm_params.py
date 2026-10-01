"""Simulator-neutral pioneer_bimanual_arm parameters: joints, default pose, gripper, actuator gains.

Plain Python (no Isaac / MuJoCo imports) so both bimanual_arm.py (Isaac Lab) and
mujoco_arm.py (MuJoCo) build from the same numbers. Naming: LEFT_* = the L-suffixed
chain (joint1L..joint6l) = physical LEFT arm; RIGHT_* = unsuffixed.
"""
import math


def _deg(degrees: float) -> float:
    return degrees * math.pi / 180.0


# --- Default (spawn) pose: URDF zero, except the elbows flexed to +/-75 deg.
#
# At URDF zero both arms hang straight down at the elbow EXTENSION SINGULARITY (manipulability
# ~2e-06, cond(J) ~2560), with the least-controllable direction almost exactly the +Z that
# _HOME_TIP_Z_OFFSET then commands -- so the first IK step is huge, hikes the shoulder, and
# picks an elbow-bend direction at random. Flexing the elbows raises manipulability ~10,000x
# and starts the arm in the flexion branch.
#
# Signs are OPPOSITE (joint4/joint4l axes are (0,-1,0)/(0,1,0)); both put the forearm forward
# toward +X. Opposite-and-equal is what makes the pose mirror-symmetric. 75 not 90 drops the
# fingertip ~10cm (measured via compute_gripper_tip_pose_b, sweeping theta) -- ~6% manipulability
# cost, still 3 orders clear of the singularity; don't go below ~60 (tip nears the table).
# Both well inside the URDF limits (57.5 deg margin each side).
DEFAULT_JOINT_POS = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": _deg(75.0),
    "joint5": 0.0,
    "joint6": 0.0,
    "joint7": 0.0,
    "joint8": 0.0,
    # Left arm (the L-suffixed chain)
    "joint1L": 0.0,
    "joint2l": 0.0,
    "joint3l": 0.0,
    "joint4l": _deg(-75.0),
    "joint5l": 0.0,
    "joint6l": 0.0,
    "joint7l": 0.0,
    "joint8l": 0.0,
}

LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT_GRIPPER_JOINTS = ["joint7l", "joint8l"]
RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]

# Gripper finger targets, verified empirically in headless Isaac sim: fingertip gap 0.074m
# closed / ~0.16m open at these endpoints. This asset's joint7/8 axis is Y.
LEFT_GRIPPER_OPEN = {"joint7l": 0.0, "joint8l": 0.0}
LEFT_GRIPPER_CLOSED = {"joint7l": -0.05, "joint8l": 0.05}
RIGHT_GRIPPER_OPEN = {"joint7": 0.0, "joint8": 0.0}
RIGHT_GRIPPER_CLOSED = {"joint7": -0.05, "joint8": 0.05}

# Prismatic gripper PD — tuned for hold during arm motion (not from motor datasheet).
# If fingers bounce when the shoulder moves, raise stiffness; if jittery, raise damping.
_GRIPPER = dict(stiffness=400.0, damping=40.0, effort_limit=30.0, velocity_limit=0.2)  # N, m/s

# Joint PD groups: stiffness (Nm/rad or N/m), damping, effort_limit (Nm or N), velocity_limit.
# effort_limit is a sim-only cap at the motor's PEAK torque (rated saturates against gravity
# with the arm extended). Do NOT raise it past the real peak (sim-to-real mismatch for dataset
# collection) -- use stiffness/damping, which command torque harder within the same budget.
ACTUATOR_GROUPS = {
    # AK10-9 V3.0 — shoulder joints 1-2 (18 Nm rated / 53 peak). Stiffness/damping raised ~50%
    # from the original static-hold tuning to stop the shoulder lagging on large reaches; watch
    # for overshoot if pushed further.
    "left_shoulder": dict(joints=["joint1L", "joint2l"], stiffness=2270.0, damping=180.0, effort_limit=53.0, velocity_limit=6.0),
    # AK80-9 V3.0 — elbow joints 3-5 (9 rated / 22 peak); raised with the shoulder for "forearm too slow".
    "left_elbow": dict(joints=["joint3l", "joint4l", "joint5l"], stiffness=1550.0, damping=110.0, effort_limit=22.0, velocity_limit=6.0),
    # GL40 KV70 — wrist joint 6 (0.25 rated / 0.73 peak). Rated saturated instantly in Isaac;
    # don't exceed 0.73 -- raise stiffness instead.
    "left_wrist": dict(joints=["joint6l"], stiffness=341.0, damping=18.0, effort_limit=0.73, velocity_limit=6.0),
    # GL40 KV70 rotary -> linkage -> two prismatic fingers, driven with synchronized targets.
    # effort_limit on a prismatic DOF is a force cap (N) -- tune by grasp, not the motor rating.
    "left_gripper": dict(joints=["joint7l", "joint8l"], **_GRIPPER),
    # Right arm (unsuffixed) mirrors the left: both arms are teleoperated and need equal headroom.
    "right_shoulder": dict(joints=["joint1", "joint2"], stiffness=2270.0, damping=180.0, effort_limit=53.0, velocity_limit=6.0),
    "right_elbow": dict(joints=["joint3", "joint4", "joint5"], stiffness=1550.0, damping=110.0, effort_limit=22.0, velocity_limit=6.0),
    "right_wrist": dict(joints=["joint6"], stiffness=341.0, damping=18.0, effort_limit=0.73, velocity_limit=6.0),
    "right_gripper": dict(joints=["joint7", "joint8"], **_GRIPPER),
}
