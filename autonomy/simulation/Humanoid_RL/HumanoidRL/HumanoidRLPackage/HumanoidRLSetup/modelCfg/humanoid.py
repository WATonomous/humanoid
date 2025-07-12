# Code for specifying custom model parameters

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

HAND_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/hy/Downloads/Humanoid_Wato/ModelAssets/hand.usd",
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
