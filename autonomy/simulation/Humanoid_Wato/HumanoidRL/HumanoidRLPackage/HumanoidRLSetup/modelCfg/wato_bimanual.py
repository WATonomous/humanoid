"""Wrapper for the existing WATO bimanual arm Isaac Lab config."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import isaaclab.sim as sim_utils

_SIMULATION_DIR = Path(__file__).resolve().parents[5]
_BIMANUAL_CFG_PATH = _SIMULATION_DIR / "Teleop" / "keyboard-based teleoperation" / "bimanual_arm_cfg.py"

_spec = importlib.util.spec_from_file_location("wato_keyboard_bimanual_arm_cfg", str(_BIMANUAL_CFG_PATH))
if _spec is None or _spec.loader is None:
    raise ImportError(f"Could not load bimanual arm config from {_BIMANUAL_CFG_PATH}")

_bimanual_arm_cfg = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _bimanual_arm_cfg
_spec.loader.exec_module(_bimanual_arm_cfg)

_ROBOT_LIMITS_USDA_PATH = (
    _SIMULATION_DIR / "Humanoid_Wato" / "wato_bimanual_arm" / "urdf" / "armDouble.SLDASM" / "armDouble.SLDASM_limits.usda"
)

BIMANUAL_ARM_CFG = _bimanual_arm_cfg.BIMANUAL_ARM_CFG.replace(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(_ROBOT_LIMITS_USDA_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
        ),
    )
)

# The lift task needs a firm simulated pinch, but extreme gains make the cube
# chatter against the tiny finger contacts. Keep this gripper strong enough to
# hold the cube while letting PhysX resolve contact smoothly.
BIMANUAL_ARM_CFG.actuators["right_gripper"].stiffness = 450.0
BIMANUAL_ARM_CFG.actuators["right_gripper"].damping = 55.0
BIMANUAL_ARM_CFG.actuators["right_gripper"].effort_limit_sim = 55.0
BIMANUAL_ARM_CFG.actuators["right_gripper"].velocity_limit_sim = 0.35

RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
RIGHT_GRIPPER_JOINTS = ["joint7", "joint8"]
RIGHT_CONTROLLED_JOINTS = RIGHT_ARM_JOINTS + RIGHT_GRIPPER_JOINTS
RIGHT_EE_BODY = "link6"
RIGHT_FINGER_TIP_BODIES = ("link7", "link8")
RIGHT_FINGER_DISTAL_TIP_LOCAL = {
    "link7": (0.13211595, -0.04057075, -0.00434997),
    "link8": (-0.13211595, -0.04057075, -0.00435003),
}

# The URDF exposes two prismatic finger joints. These values match the keyboard teleop config.
RIGHT_GRIPPER_OPEN = {"joint7": -0.05, "joint8": 0.05}
RIGHT_GRIPPER_CLOSED = {"joint7": 0.0, "joint8": 0.0}
