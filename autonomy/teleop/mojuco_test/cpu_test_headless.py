import mujoco
import time

URDF_PATH = "wato_bimanual_arm/urdf/armDouble.SLDASM.urdf"

try:
    print(f"Loading {URDF_PATH} on CPU...")
    model = mujoco.MjModel.from_xml_path(URDF_PATH)
    data = mujoco.MjData(model)
    
    print("Model compiled successfully! Running 1000 headless physics steps...")
    
    # Step the physics engine 1000 times without rendering a window
    start_time = time.time()
    for _ in range(1000):
        mujoco.mj_step(model, data)
    
    elapsed = time.time() - start_time
    print(f"Success! 1000 steps completed in {elapsed:.4f} seconds.")
    print(f"Final Joint Positions (qpos): {data.qpos}")

except Exception as e:
    print(f"Simulation failed: {e}")