"""Re-export of the repo's canonical pioneer_bimanual_arm articulation config.

Not vendored -- imports the SAME ``BIMANUAL_ARM_CFG`` (and joint/body name lists)
that teleop and the other Isaac tasks use, so the arm's actuator gains, joint
limits and left/right naming stay in one place: ``src/pioneer_humanoid/``. That
package is editable-installed in the isaac_lab image.

Naming uses the corrected convention: the L-suffixed URDF chain (joint1L,
joint2l..joint6l, link6l) is the physical LEFT arm; the unsuffixed chain is the
RIGHT arm.
"""
from pioneer_humanoid.bimanual_arm import (  # noqa: F401
    BIMANUAL_ARM_CFG,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    LEFT_GRIPPER_JOINTS,
    RIGHT_GRIPPER_JOINTS,
    LEFT_EE_BODY,
    RIGHT_EE_BODY,
)

__all__ = [
    "BIMANUAL_ARM_CFG",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
    "LEFT_GRIPPER_JOINTS",
    "RIGHT_GRIPPER_JOINTS",
    "LEFT_EE_BODY",
    "RIGHT_EE_BODY",
]
