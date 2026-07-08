import mujoco
import time
from pathlib import Path
from viser import ViserServer
from mjviser import ViserMujocoScene

URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / "simulation/Humanoid_Wato/wato_bimanual_arm/urdf/armDouble.SLDASM.urdf"
)


def main():
    print(f"Loading {URDF_PATH}...")
    try:
        model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        return

    print("Starting local web server on port 8080...")
    # host="0.0.0.0" is strictly required to punch through the Docker walls
    server = ViserServer(port=8080, host="0.0.0.0")

    print("Attaching MuJoCo physics to the web viewer...")
    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.create_visualization_gui()

    print("=========================================================")
    print(" SUCCESS! Open your Windows web browser and go to:")
    print(" http://localhost:8080")
    print("=========================================================")

    try:
        # Step the physics engine continuously to animate the web viewer
        while True:
            mujoco.mj_step(model, data)
            scene.update_from_mjdata(data)
            time.sleep(model.opt.timestep)
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")


if __name__ == "__main__":
    main()
