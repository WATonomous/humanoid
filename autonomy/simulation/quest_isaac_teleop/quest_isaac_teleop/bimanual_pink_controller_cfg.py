"""Pink IK action config for the bimanual arm."""

from pink.tasks import DampingTask as DampingTaskCfg
from isaaclab.controllers.pink_ik import NullSpacePostureTask as NullSpacePostureTaskCfg, PinkIKControllerCfg
from isaaclab.controllers.pink_ik.local_frame_task import LocalFrameTask as LocalFrameTaskCfg

RIGHT_EE_LINK = "link6"
LEFT_EE_LINK = "link6l"
BASE_LINK = "base_link"

RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT_ARM_JOINTS = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]
ARM_JOINTS = RIGHT_ARM_JOINTS + LEFT_ARM_JOINTS

# Order must match the teleop pipeline output.
GRIPPER_JOINTS = ["joint7", "joint8", "joint7l", "joint8l"]

GRIPPER_OPEN = {"joint7": 0.0, "joint8": 0.0, "joint7l": 0.0, "joint8l": 0.0}
GRIPPER_CLOSED = {"joint7": -0.05, "joint8": 0.05, "joint7l": -0.05, "joint8l": 0.05}


WATO_BIMANUAL_IK_CONTROLLER_CFG = PinkIKControllerCfg(
    articulation_name="robot",
    base_link_name=BASE_LINK,
    num_hand_joints=len(GRIPPER_JOINTS),
    show_ik_warnings=True,
    fail_on_joint_limit_violation=False,
    variable_input_tasks=[
        LocalFrameTaskCfg(
            frame=RIGHT_EE_LINK,
            base_link_frame_name=BASE_LINK,
            position_cost=8.0,
            orientation_cost=1.0,
            lm_damping=12,
            gain=0.5,
        ),
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

try:
    from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg
    WATO_BIMANUAL_IK_ACTION_CFG = PinkInverseKinematicsActionCfg(
        pink_controlled_joint_names=ARM_JOINTS,
        hand_joint_names=GRIPPER_JOINTS,
        target_eef_link_names={"right_ee": RIGHT_EE_LINK, "left_ee": LEFT_EE_LINK},
        asset_name="robot",
        enable_gravity_compensation=True,
        controller=WATO_BIMANUAL_IK_CONTROLLER_CFG,
    )
except ImportError:
    WATO_BIMANUAL_IK_ACTION_CFG = None
