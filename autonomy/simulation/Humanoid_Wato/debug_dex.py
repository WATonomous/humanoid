import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig
import os

urdf_path = "/workspace/isaaclab/humanoid/autonomy/simulation/Humanoid_Wato/arm_assembly/arm_assembly_fixed.urdf"

config_dict = {
    "type": "vector",
    "urdf_path": urdf_path,
    "wrist_link_name": "PALM_GAVIN_1DoF_Hinge_v2_1",
    "target_origin_link_names": [
        "CMC_THUMB_v1_1", "MCP_THUMB_v1_1",
        "MCP_INDEX_v1_1", "PIP_INDEX_v1_1",
        "MCP_MIDDLE_v1_1", "PIP_MIDDLE_v1_1",
        "MCP_RING_v1_1", "PIP_RING_v1_1",
        "MCP_PINKY_v1_1", "PIP_PINKY_v1_1"
    ],
    "target_task_link_names": [
        "MCP_THUMB_v1_1", "IP_THUMB_v1_1",
        "PIP_INDEX_v1_1", "DIP_INDEX_v1_1",
        "PIP_MIDDLE_v1_1", "DIP_MIDDLE_v1_1",
        "PIP_RING_v1_1", "DIP_RING_v1_1",
        "PIP_PINKY_v1_1", "DIP_PINKY_v1_1"
    ],
    "target_link_human_indices": np.array([
        [1, 2], [2, 3],
        [5, 6], [6, 7],
        [9, 10], [10, 11],
        [13, 14], [14, 15],
        [17, 18], [18, 19]
    ])
}

retargeter = RetargetingConfig.from_dict(config_dict).build()

# Get the rest pose directions mathematically determined by Pinocchio
print("--- ROBOT REST POSE VECTORS IN URDF ---")
for i, name in enumerate(config_dict["target_task_link_names"]):
    vec = retargeter.optimizer.target_link_rest_vecs[i]
    print(f"Index {i} ({name} relative to origin): {vec}")

print("\n--- TEST: HAND FLAT (+Y FOR ALL FINGERS) ---")
target_vectors_flat = np.zeros((10, 3))
for i in range(2, 10):
    target_vectors_flat[i] = [0.0, 1.0, 0.0]  # Straight +Y
# Set thumb arbitrarily
target_vectors_flat[0] = [1.0, 0.0, 0.0]
target_vectors_flat[1] = [1.0, 0.0, 0.0]

action_flat = retargeter.retarget(target_vectors_flat)
print("ACTION FOR FLAT HAND:")
for i, j_name in enumerate(retargeter.joint_names):
    if "index" in j_name or "middle" in j_name or "ring" in j_name:
        print(f"{j_name}: {action_flat[i]:.3f} rad ({action_flat[i]*180/np.pi:.1f} deg)")

print("\n--- TEST: HAND CURLED (-Z FOR ALL FINGERS) ---")
target_vectors_curled = np.zeros((10, 3))
for i in range(2, 10):
    target_vectors_curled[i] = [0.0, 0.0, -1.0]  # Curled down -Z
# Set thumb arbitrarily
target_vectors_curled[0] = [1.0, 0.0, 0.0]
target_vectors_curled[1] = [1.0, 0.0, 0.0]

action_curled = retargeter.retarget(target_vectors_curled)
print("ACTION FOR CURLED HAND:")
for i, j_name in enumerate(retargeter.joint_names):
    if "index" in j_name or "middle" in j_name or "ring" in j_name:
        print(f"{j_name}: {action_curled[i]:.3f} rad ({action_curled[i]*180/np.pi:.1f} deg)")
