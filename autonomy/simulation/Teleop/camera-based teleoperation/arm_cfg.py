"""
arm_cfg.py
==========
Local copy of the ARM_CFG articulation config so wato_hand_isaaclab_teleop.py
can be run directly from the camera-based teleoperation/ folder without
requiring PYTHONPATH to include Humanoid_Wato/.

Original source:
  Humanoid_Wato/HumanoidRL/HumanoidRLPackage/HumanoidRLSetup/modelCfg/humanoid.py
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# Resolve ModelAssets relative to THIS file:
# camera-based teleoperation/ -> Teleop/ -> simulation/ -> Humanoid_Wato/ModelAssets/
_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_HUMANOID_WATO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "Humanoid_Wato"))
_MODEL_ASSETS = os.path.join(_HUMANOID_WATO_ROOT, "ModelAssets")

# Hand Arm (6 DOF arm + 15 DOF hand)
_ARM_USD_PATH = os.path.join(_MODEL_ASSETS, "arm.usd")
ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_ARM_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_flexion_extension":    0.0,
            "shoulder_abduction_adduction":  0.0,
            "shoulder_rotation":             0.0,
            "elbow_flexion_extension":       0.0,
            "forearm_rotation":              0.0,
            "wrist_extension":               0.0,
            "mcp_index":  0.0,
            "pip_index":  0.0,
            "dip_index":  0.0,
            "mcp_middle": 0.0,
            "pip_middle": 0.0,
            "dip_middle": 0.0,
            "mcp_ring":   0.0,
            "pip_ring":   0.0,
            "dip_ring":   0.0,
            "mcp_pinky":  0.0,
            "pip_pinky":  0.0,
            "dip_pinky":  0.0,
            "cmc_thumb":  0.0,
            "mcp_thumb":  0.79,   # within limits [0.785, 2.531]
            "ip_thumb":   0.0,
        }
    ),
    actuators={
        # J≈1.20 kg·m², f=4.0 Hz, ζ=1.0 → k=757.6, d=60.3
        "shoulder_fe": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_flexion_extension"],
            stiffness=757.6,
            damping=60.3,
            velocity_limit_sim=3.0,
        ),
        # J≈0.95 kg·m², f=4.0 Hz, ζ=1.0 → k=600.0, d=47.7
        "shoulder_aa": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_abduction_adduction"],
            stiffness=600.0,
            damping=47.7,
            velocity_limit_sim=3.0,
        ),
        # J≈0.77 kg·m², f=4.5 Hz, ζ=1.0 → k=615.5, d=43.5
        "shoulder_rot": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_rotation"],
            stiffness=615.5,
            damping=43.5,
            velocity_limit_sim=3.0,
        ),
        # J≈0.77 kg·m², f=4.5 Hz, ζ=1.0 → k=615.5, d=43.5
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_flexion_extension"],
            stiffness=615.5,
            damping=43.5,
            velocity_limit_sim=3.0,
        ),
        # J≈0.45 kg·m², f=5.0 Hz, ζ=1.0 → k=444.1, d=28.3
        "forearm": ImplicitActuatorCfg(
            joint_names_expr=["forearm_rotation"],
            stiffness=444.1,
            damping=28.3,
            velocity_limit_sim=3.0,
        ),
        # J≈0.12 kg·m², f=6.0 Hz, ζ=1.0 → k=170.5, d=9.0
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_extension"],
            stiffness=170.5,
            damping=9.0,
            velocity_limit_sim=3.0,
        ),
        # J≈0.0012 kg·m², f=8.0 Hz, ζ=1.0 → k=3.0, d=0.12
        "finger_mcp": ImplicitActuatorCfg(
            joint_names_expr=["mcp_index", "mcp_middle", "mcp_ring", "mcp_pinky"],
            stiffness=3.0,
            damping=0.12,
            velocity_limit_sim=3.0,
        ),
        # J≈0.0006 kg·m², f=8.0 Hz, ζ=1.0 → k=1.5, d=0.06
        "finger_pip": ImplicitActuatorCfg(
            joint_names_expr=["pip_index", "pip_middle", "pip_ring", "pip_pinky"],
            stiffness=1.5,
            damping=0.06,
            velocity_limit_sim=3.0,
        ),
        # J≈0.0002 kg·m², f=8.0 Hz, ζ=1.0 → k=0.5, d=0.02
        "finger_dip": ImplicitActuatorCfg(
            joint_names_expr=["dip_index", "dip_middle", "dip_ring", "dip_pinky"],
            stiffness=0.5,
            damping=0.02,
            velocity_limit_sim=3.0,
        ),
        # J≈0.003 kg·m², f=8.0 Hz, ζ=1.0 → k=7.6, d=0.30
        "thumb_cmc": ImplicitActuatorCfg(
            joint_names_expr=["cmc_thumb"],
            stiffness=7.6,
            damping=0.30,
            velocity_limit_sim=3.0,
        ),
        # J≈0.001 kg·m², f=8.0 Hz, ζ=1.0 → k=2.5, d=0.10
        "thumb_mcp": ImplicitActuatorCfg(
            joint_names_expr=["mcp_thumb"],
            stiffness=2.5,
            damping=0.10,
            velocity_limit_sim=3.0,
        ),
        # J≈0.0002 kg·m², f=8.0 Hz, ζ=1.0 → k=0.5, d=0.02
        "thumb_ip": ImplicitActuatorCfg(
            joint_names_expr=["ip_thumb"],
            stiffness=0.5,
            damping=0.02,
            velocity_limit_sim=3.0,
        ),
    },
)
