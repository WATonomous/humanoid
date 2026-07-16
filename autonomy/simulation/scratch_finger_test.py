import sys
sys.path.append("/root/ament_ws/src/simulation/Humanoid_Wato/arm_assembly")
import fingertip_ik
import mujoco
import numpy as np

model = fingertip_ik.load_model()
data = mujoco.MjData(model)

palm_id = model.body("PALM_GAVIN_1DoF_Hinge_v2_1").id

fingers = {
    "index": ["mcp_index", "pip_index", "dip_index", "DIP_INDEX_v1_1"],
    "middle": ["mcp_middle", "pip_middle", "dip_middle", "DIP_MIDDLE_v1_1"],
    "ring": ["mcp_ring", "pip_ring", "dip_ring", "DIP_RING_v1_1"],
    "pinky": ["mcp_pinky", "pip_pinky", "dip_pinky", "DIP_PINKY_v1_1"],
    "thumb": ["cmc_thumb", "mcp_thumb", "ip_thumb", "IP_THUMB_v1_1"]
}

for name, joints in fingers.items():
    tip_body = joints[-1]
    tip_id = model.body(tip_body).id
    j_adrs = [model.joint(j).qposadr[0] for j in joints[:-1]]
    
    print(f"Finger: {name}")
    results = []
    for angle in np.linspace(-1.5, 1.5, 31):
        data.qpos[:] = 0.0
        data.qpos[3] = 0.5  # elbow bend
        for adr in j_adrs:
            data.qpos[adr] = angle
        mujoco.mj_forward(model, data)
        dist = np.linalg.norm(data.xpos[tip_id] - data.xpos[palm_id])
        results.append((angle, dist))
    
    max_angle, max_dist = max(results, key=lambda x: x[1])
    min_angle, min_dist = min(results, key=lambda x: x[1])
    print(f"  Max extension: {max_angle:.2f} rad (dist={max_dist:.4f})")
    print(f"  Min extension (curled): {min_angle:.2f} rad (dist={min_dist:.4f})")
