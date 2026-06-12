import math
import os
import sys

import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg

_HUMANOID_WATO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
_WATO_HAND_DIR = os.path.join(_HUMANOID_WATO_ROOT, "wato_hand")
if _WATO_HAND_DIR not in sys.path:
    sys.path.insert(0, _WATO_HAND_DIR)

from wato_hand_cfg import WATO_HAND_CFG  # noqa: E402

# Partial grasp pose for in-hand cube reorientation (~50% of flex range).
# Finger splay at ~50% of ±8.6 deg limit (0.075 rad) so spread is visible at reset.
_INHAND_SPREAD_RAD = 0.075

_INHAND_GRASP_JOINT_POS = {
    "circumduction": 0.0,
    "MCP_A_thumb": 0.5,
    "PIP_thumb": -0.8,
    "DIP_thumb": -0.8,
    "MCP_A_1": _INHAND_SPREAD_RAD,
    "MCP_A_2": _INHAND_SPREAD_RAD,
    "MCP_A_3": _INHAND_SPREAD_RAD,
    "MCP_A_4": _INHAND_SPREAD_RAD,
    "MCP_1": 0.8,
    "PIP_1": -0.8,
    "DIP_1": 0.8,
    "MCP_2": 0.8,
    "PIP_2": -0.8,
    "DIP_2": 0.8,
    "MCP_3": -0.8,
    "PIP_3": -0.8,
    "DIP_3": 0.8,
    "MCP_4": 0.8,
    "PIP_4": -0.8,
    "DIP_4": 0.8,
}

def _quat_y_deg(deg: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) for a pure rotation about world Y."""
    half = math.radians(deg) * 0.5
    return (math.cos(half), 0.0, math.sin(half), 0.0)


# Wato URDF at identity: fingers extend +Y. Palm-up tuned at +160 deg Y.
_INHAND_ROT_Y_POS_160 = _quat_y_deg(160.0)
_INHAND_ROT_Y_NEG_160 = _quat_y_deg(-160.0)
_INHAND_PALM_UP_ROT = _INHAND_ROT_Y_POS_160

# DexCube spawn in env frame, paired with _INHAND_PALM_UP_ROT (palm center ~ y=0.08, z=0.54).
INHAND_CUBE_POS = (0.0, 0.08, 0.54)

INHAND_WATO_HAND_CFG = WATO_HAND_CFG.replace(
    spawn=WATO_HAND_CFG.spawn.replace(
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=False,
            enable_gyroscopic_forces=False,
            angular_damping=0.01,
            max_linear_velocity=1000.0,
            max_angular_velocity=64 / math.pi * 180.0,
            max_depenetration_velocity=1000.0,
            max_contact_impulse=1e32,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        rot=_INHAND_PALM_UP_ROT,
        joint_pos=_INHAND_GRASP_JOINT_POS,
    ),
    soft_joint_pos_limit_factor=1.0,
)
