import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_HUMANOID_WATO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
_LEGR_USD_PATH = os.path.join(_HUMANOID_WATO_ROOT, "UsdModelAssets", "LEGR", "usd", "LEGR.usd")


LEGR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_LEGR_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=True,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "F_R": 0.0,
            "A_R": 0.0,
            "R_R": 0.0,
            "Knee_R": 0.0,
            "U_R_R": 0.0,
            "U_P_R": 0.0,
            "F_L": 0.0,
            "A_L": 0.0,
            "R_L": 0.0,
            "Knee_L": 0.0,
            "U_R_L": 0.0,
            "U_P_L": 0.0,
        },
    ),
    actuators={
        "hip_flexion": ImplicitActuatorCfg(
            joint_names_expr=["F_.*"],
            effort_limit_sim=222.0,
            velocity_limit_sim=2.93,
            stiffness=100.0,
            damping=8.0,
        ),
        "hip_abduction": ImplicitActuatorCfg(
            joint_names_expr=["A_.*"],
            effort_limit_sim=120.0,
            velocity_limit_sim=17.488,
            stiffness=80.0,
            damping=5.0,
        ),
        "hip_rotation": ImplicitActuatorCfg(
            joint_names_expr=["R_.*"],
            effort_limit_sim=60.0,
            velocity_limit_sim=18.85,
            stiffness=60.0,
            damping=4.0,
        ),
        "knee": ImplicitActuatorCfg(
            joint_names_expr=["Knee_.*"],
            effort_limit_sim=222.0,
            velocity_limit_sim=2.93,
            stiffness=100.0,
            damping=8.0,
        ),
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=["U_.*"],
            effort_limit_sim=120.0,
            velocity_limit_sim=18.85,
            stiffness=40.0,
            damping=3.0,
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
