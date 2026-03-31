import torch
print("Loading DexRetargeting...")
from dex_retargeting.retargeting_config import RetargetingConfig

urdf_path = "/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/arm_assembly.urdf"

# The Cartesian fingertip configuration we extracted from your URDF
config_dict = {
    "type": "position",
    "urdf_path": urdf_path,
    "wrist_link_name": "PALM_GAVIN_1DoF_Hinge_v2_1",
    "target_link_names": [
        "IP_THUMB_v1_1", 
        "DIP_INDEX_v1_1", 
        "DIP_MIDDLE_v1_1", 
        "DIP_RING_v1_1", 
        "DIP_PINKY_v1_1"
    ]
}

print(f"Building Kinematic IK Tree from {urdf_path}...")
try:
    retargeter = RetargetingConfig.from_dict(config_dict).build()
    print("SUCCESS: Pinocchio parsed the URDF and built the IK Solver matrix!")
    print(f"Number of controllable joints found: {len(retargeter.joint_names)}")
    print(f"Joints: {retargeter.joint_names}")
except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"FAILURE: {e}")
