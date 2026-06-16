"""Pink IK + action config for the single-articulation bimanual arm.

Robot: Humanoid_Wato/wato_bimanual_arm (armDouble.SLDASM) — ONE articulation
holding both arms, each ending in a 2-finger prismatic gripper.

Unlike the 21-DoF setup (two separate arm assemblies), this is a single robot,
so it maps cleanly onto the G1 reference: one PinkInverseKinematicsAction with
two wrist FrameTasks. URDF link/joint names match the Isaac body/joint names
1:1 (both come from the same SolidWorks export), so there is no name-prefix
mismatch to manage here.
"""

from isaaclab.controllers.pink_ik import (
    DampingTaskCfg,
    LocalFrameTaskCfg,
    NullSpacePostureTaskCfg,
    PinkIKControllerCfg,
)
from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg

# EE frames (wrist link of each arm) and the shared base link.
RIGHT_EE_LINK = "link6"
LEFT_EE_LINK = "link6l"
BASE_LINK = "base_link"

# 12 arm joints solved by IK (6 per arm). Grippers are excluded here.
RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
ARM_JOINTS = RIGHT_ARM_JOINTS + LEFT_ARM_JOINTS

# 4 prismatic gripper joints driven directly from the controller triggers
# (right pair first, then left pair). Order must match the pipeline output.
GRIPPER_JOINTS = ["joint7", "joint8", "joint7l", "joint8l"]

# Gripper target positions. One GL40 motor drives a two-prismatic linkage;
# open/closed are synchronized pairs. On this robot the (0,0) pose is physically
# OPEN and the ±0.05 spread is CLOSED, so the mapping is intentionally the
# reverse of the joint-limit signs.
GRIPPER_OPEN = {"joint7": 0.0, "joint8": 0.0, "joint7l": 0.0, "joint8l": 0.0}
GRIPPER_CLOSED = {"joint7": -0.05, "joint8": 0.05, "joint7l": -0.05, "joint8l": 0.05}


WATO_BIMANUAL_IK_CONTROLLER_CFG = PinkIKControllerCfg(
    articulation_name="robot",
    base_link_name=BASE_LINK,
    num_hand_joints=len(GRIPPER_JOINTS),
    show_ik_warnings=True,
    fail_on_joint_limit_violation=False,
    variable_input_tasks=[
        # Right wrist — task index 0 → action poses[0]
        # Tuning mirrors the snappy Isaac-PickPlace-GR1T2 task (gain 0.5,
        # lm_damping 12, orientation_cost 1.0) rather than the slower G1 example.
        LocalFrameTaskCfg(
            frame=RIGHT_EE_LINK,
            base_link_frame_name=BASE_LINK,
            position_cost=8.0,
            orientation_cost=1.0,
            lm_damping=12,
            gain=0.5,
        ),
        # Left wrist — task index 1 → action poses[1]
        LocalFrameTaskCfg(
            frame=LEFT_EE_LINK,
            base_link_frame_name=BASE_LINK,
            position_cost=8.0,
            orientation_cost=1.0,
            lm_damping=12,
            gain=0.5,
        ),
        NullSpacePostureTaskCfg(
            cost=0.05,
            lm_damping=1,
            controlled_frames=[RIGHT_EE_LINK, LEFT_EE_LINK],
            controlled_joints=ARM_JOINTS,
            gain=0.075,
        ),
        DampingTaskCfg(cost=0.5),
    ],
    fixed_input_tasks=[],
)

WATO_BIMANUAL_IK_ACTION_CFG = PinkInverseKinematicsActionCfg(
    pink_controlled_joint_names=ARM_JOINTS,
    hand_joint_names=GRIPPER_JOINTS,
    # Task order here matches variable_input_tasks: right wrist then left wrist.
    target_eef_link_names={"right_ee": RIGHT_EE_LINK, "left_ee": LEFT_EE_LINK},
    asset_name="robot",
    # GR1T2's teleop leaves this at the default True (it runs on PhysX, like us).
    # Without it the arm sags under gravity and the controller fights it, which
    # reads as sluggish/droopy. Must be False only on the Newton backend.
    enable_gravity_compensation=True,
    controller=WATO_BIMANUAL_IK_CONTROLLER_CFG,
)
