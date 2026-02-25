# Code for specifying custom model parameters

from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_CURRENT_FILE = Path(__file__).resolve()
# ModelAssets lives under Humanoid_RL/ModelAssets
_HAND_USD_PATH = str(_CURRENT_FILE.parents[4] / "ModelAssets" / "hand.usd")
_ARM_USD_PATH = str(_CURRENT_FILE.parents[7] / "arm_assembly" / "arm_assembly" / "arm_assembly.usd")

HAND_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_HAND_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Revolute_1": 0.0,
            "Revolute_2": 0.0,
            "Revolute_3": 0.0,
            "Revolute_4": 0.0,
            "Revolute_5": 0.0,
            "Revolute_6": 0.0,
            "Revolute_7": 0.0,
            "Revolute_8": 0.0,
            "Revolute_9": 0.0,
            "Revolute_10": 0.0,
            "Revolute_11": 0.0,
            "Revolute_12": 0.0,
            "Revolute_13": 0.0,
            "Revolute_14": 0.0,
            "Revolute_15": 0.0,
        }
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            # velocity_limit=100.0,
            # effort_limit=87.0,
            stiffness=0.5,
            damping=0.5,
        ),
    },
)

ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_ARM_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_flexion_extension": 0.0,
            "shoulder_abduction_adduction": -0.348,
            "shoulder_rotation": -0.14,
            "elbow_flexion_extension": 0.08,
            "forearm_rotation": 0.0,
            "wrist_extension": -0.11,
            "mcp_index": 0.0,
            "pip_index": 1.57,
            "dip_index": 0.0,
            "mcp_middle": 0.0,
            "pip_middle": 1.57,
            "dip_middle": 1.57,
            "mcp_ring": 1.57,
            "pip_ring": 0.0,
            "dip_ring": 0.0,
            "mcp_pinky": 1.57,
            "pip_pinky": 0.0,
            "dip_pinky": 1.57,
            "cmc_thumb": 0.63,
            "mcp_thumb": 2.12,
            "ip_thumb": 0.0,
        }
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            # velocity_limit=100.0,
            # effort_limit=87.0,
            stiffness=0.5,
            damping=0.5,
        ),
    },
)
