import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

_HUMANOID_WATO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
_WATO_HUMANOID_V1_URDF_PATH = os.path.join(
    _HUMANOID_WATO_ROOT,
    "Wato Humanoid Simultion Model V1",
    "urdf",
    "Wato_Humanoid_V1_isaac.urdf",
)
_WATO_HUMANOID_V1_USD_DIR = os.path.join(
    _HUMANOID_WATO_ROOT,
    "Wato Humanoid Simultion Model V1",
    "urdf",
    "Wato_Humanoid_V1_isaac_usd",
)

# Mild crouch; keep feet near ground after bend.
_SPAWN_HEIGHT = 0.75


WATO_HUMANOID_V1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=_WATO_HUMANOID_V1_URDF_PATH,
        usd_dir=_WATO_HUMANOID_V1_USD_DIR,
        usd_file_name="Wato_Humanoid_V1.usd",
        force_usd_conversion=False,
        make_instanceable=True,
        fix_base=False,
        merge_fixed_joints=True,
        root_link_name="base",
        self_collision=False,
        collider_type="convex_hull",
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=100.0,
                damping=5.0,
            )
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=2.5,
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
        pos=(0.0, 0.0, _SPAWN_HEIGHT),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "Hip_F_L": 0.12,
            "Hip_A_L": 0.04,
            "Hip_R_L": 0.0,
            "Knee_L": -0.22,
            "Ankle_P_L": 0.10,
            "Ankle_R_L": 0.0,
            "Hip_F_R": 0.12,
            "Hip_A_R": 0.04,
            "Hip_R_R": 0.0,
            "Knee_R": -0.22,
            "Ankle_P_R": 0.10,
            "Ankle_R_R": 0.0,
        },
    ),
    actuators={
        "hip_flexion": ImplicitActuatorCfg(
            joint_names_expr=["Hip_F_.*"],
            effort_limit_sim=222.0,
            velocity_limit_sim=3.6652,
            stiffness=100.0,
            damping=8.0,
        ),
        "hip_abduction": ImplicitActuatorCfg(
            joint_names_expr=["Hip_A_.*"],
            effort_limit_sim=120.0,
            velocity_limit_sim=20.944,
            stiffness=80.0,
            damping=5.0,
        ),
        "hip_rotation": ImplicitActuatorCfg(
            joint_names_expr=["Hip_R_.*"],
            effort_limit_sim=60.0,
            velocity_limit_sim=20.42,
            stiffness=60.0,
            damping=4.0,
        ),
        "knee": ImplicitActuatorCfg(
            joint_names_expr=["Knee_.*"],
            effort_limit_sim=222.0,
            velocity_limit_sim=3.6652,
            stiffness=100.0,
            damping=8.0,
        ),
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=["Ankle_.*"],
            effort_limit_sim=60.0,
            velocity_limit_sim=20.42,
            stiffness=80.0,
            damping=6.0,
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
