"""
20-DOF Wato hand articulation config.

Robot model: Humanoid_Wato/wato_hand (hand_urdf.usd / hand_urdf.urdf)
Joint limits: Isaac Sim Physics Inspector (/World/hand_urdf/hand_origin)
Finger splay (MCP_A_1..4): ±8.594 deg (±0.15 rad) per joint.

    import sys, os
    sys.path.insert(0, os.path.join(<repo>, "autonomy/simulation/Humanoid_Wato/wato_hand"))
    from wato_hand_cfg import WATO_HAND_CFG, JOINT_POS_LIMITS, apply_joint_limits
"""
import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_HAND_USD_PATH = os.path.join(_THIS_DIR, "urdf", "hand_urdf", "hand_urdf.usd")
_HAND_URDF_PATH = os.path.join(_THIS_DIR, "urdf", "hand_urdf.urdf")


def _deg(degrees: float) -> float:
    return degrees * math.pi / 180.0


# --- Joint limits from Physics Inspector (degrees -> rad) --------------------
_MCP_A_LIMIT = (_deg(-27.0), _deg(27.0))  # expanded to AllegroHand range; self-collision is off
_MCP_FLEX_LIMIT = (0.0, _deg(85.944))
_PIP_FLEX_LIMIT = (_deg(-85.944), 0.0)
_DIP_FLEX_LIMIT = (0.0, _deg(85.944))
_MCP_FLEX_LIMIT_RING = (_deg(-86.517), 0.0)
_PIP_FLEX_LIMIT_RING = (_deg(-86.517), 0.0)
_DIP_FLEX_LIMIT_RING = (0.0, _deg(86.517))
_MCP_FLEX_LIMIT_PINKY = (0.0, _deg(86.517))
_CIRCUMDUCTION_LIMIT = (_deg(-34.377), _deg(5.73))
_MCP_A_THUMB_LIMIT = (0.0, _deg(114.592))
_THUMB_FLEX_LIMIT = (_deg(-85.944), 0.0)

JOINT_POS_LIMITS = {
    # Thumb chain
    "circumduction": _CIRCUMDUCTION_LIMIT,
    "MCP_A_thumb": _MCP_A_THUMB_LIMIT,
    "PIP_thumb": _THUMB_FLEX_LIMIT,
    "DIP_thumb": _THUMB_FLEX_LIMIT,
    # Index (finger 1)
    "MCP_A_1": _MCP_A_LIMIT,
    "MCP_1": _MCP_FLEX_LIMIT,
    "PIP_1": _PIP_FLEX_LIMIT,
    "DIP_1": _DIP_FLEX_LIMIT,
    # Middle (finger 2)
    "MCP_A_2": _MCP_A_LIMIT,
    "MCP_2": _MCP_FLEX_LIMIT,
    "PIP_2": _PIP_FLEX_LIMIT,
    "DIP_2": _DIP_FLEX_LIMIT,
    # Ring (finger 3) — slightly different flex range
    "MCP_A_3": _MCP_A_LIMIT,
    "MCP_3": _MCP_FLEX_LIMIT_RING,
    "PIP_3": _PIP_FLEX_LIMIT_RING,
    "DIP_3": _DIP_FLEX_LIMIT_RING,
    # Pinky (finger 4)
    "MCP_A_4": _MCP_A_LIMIT,
    "MCP_4": _MCP_FLEX_LIMIT_PINKY,
    "PIP_4": _PIP_FLEX_LIMIT_RING,
    "DIP_4": _DIP_FLEX_LIMIT_RING,
}

# Ordered like config/joint_names_hand_urdf.yaml (excluding the empty root slot).
ALL_JOINT_NAMES = [
    "circumduction",
    "MCP_A_thumb",
    "PIP_thumb",
    "DIP_thumb",
    "MCP_A_1",
    "MCP_1",
    "PIP_1",
    "DIP_1",
    "MCP_A_2",
    "MCP_2",
    "PIP_2",
    "DIP_2",
    "MCP_A_3",
    "MCP_3",
    "PIP_3",
    "DIP_3",
    "MCP_A_4",
    "MCP_4",
    "PIP_4",
    "DIP_4",
]

# All 20 revolute DOF (matches config/joint_names_hand_urdf.yaml).
ACTUATED_JOINT_NAMES = list(ALL_JOINT_NAMES)

_DEFAULT_JOINT_POS = {name: 0.0 for name in ALL_JOINT_NAMES}

# Root link for dex-retargeting / wrist attachment.
WRIST_LINK_NAME = "hand_origin"


def apply_joint_limits(robot) -> None:
    """Apply Physics Inspector joint limits to the articulation."""
    limits = robot.data.joint_pos_limits.clone()
    updated = []

    for joint_idx, joint_name in enumerate(robot.data.joint_names):
        if joint_name not in JOINT_POS_LIMITS:
            continue
        lo, hi = JOINT_POS_LIMITS[joint_name]
        limits[:, joint_idx, 0] = lo
        limits[:, joint_idx, 1] = hi
        updated.append(joint_name)

    if updated:
        robot.write_joint_position_limit_to_sim(limits, warn_limit_violation=False)
        print(f"[INFO] Applied joint limits for {len(updated)} joints.")


WATO_HAND_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_HAND_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos=_DEFAULT_JOINT_POS),
    actuators={
        "thumb_circumduction": ImplicitActuatorCfg(
            joint_names_expr=["circumduction"],
            stiffness=50.0,
            damping=2.0,
            velocity_limit_sim=3.0,
        ),
        "thumb_abduction": ImplicitActuatorCfg(
            joint_names_expr=["MCP_A_thumb"],
            stiffness=30.0,
            damping=1.5,
            velocity_limit_sim=3.0,
        ),
        "thumb_flex": ImplicitActuatorCfg(
            joint_names_expr=["PIP_thumb", "DIP_thumb"],
            stiffness=10.0,
            damping=0.8,
            velocity_limit_sim=3.0,
        ),
        "finger_abduction": ImplicitActuatorCfg(
            joint_names_expr=["MCP_A_1", "MCP_A_2", "MCP_A_3", "MCP_A_4"],
            stiffness=50.0,
            damping=2.0,
            velocity_limit_sim=3.0,
        ),
        "finger_mcp": ImplicitActuatorCfg(
            joint_names_expr=["MCP_1", "MCP_2", "MCP_3", "MCP_4"],
            stiffness=50.0,
            damping=2.0,
            velocity_limit_sim=3.0,
        ),
        "finger_pip": ImplicitActuatorCfg(
            joint_names_expr=["PIP_1", "PIP_2", "PIP_3", "PIP_4"],
            stiffness=30.0,
            damping=1.5,
            velocity_limit_sim=3.0,
        ),
        "finger_dip": ImplicitActuatorCfg(
            joint_names_expr=["DIP_1", "DIP_2", "DIP_3", "DIP_4"],
            stiffness=10.0,
            damping=0.8,
            velocity_limit_sim=3.0,
        ),
    },
)
