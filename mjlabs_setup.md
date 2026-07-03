# mjlab Setup

## Initial testing folder

`/home/user/humanoid/autonomy/teleop/mojuco_test`

This folder was deleted as it was only used for certain mjlab setup tests:

1. CPU-only webview based tele-operation test
2. CPU-only RL test
3. GPU based RL test and output

## Architectural changes (currently being recommended as of July 3rd 2026)

1. **Added `mjlabs.dockerfile`** → Standalone and separate ROS2 Humble base for physics sims using mjlabs. Uses MuJoCo as the physics engine, mjviser as the web-based viewer, `jax[cuda12]` as the AI computing library, and brax as the RL-specific framework.
2. **Updated `docker-compose.teleop.yaml`** → Added a new service that calls the mjlabs dockerfile setup directly below the teleop service.
3. **GPU Passthrough** → The container now requests nvidia capabilities natively. Most RTX series GPUs running the latest drivers should be able to run training sims locally.
4. **Live WebSocket Visualization** → Container port 8080 is now bound to the host's bridging network for rendering live webview using mjviser.

## How to setup and reproduce

1. Build the new environment:
   ```bash
   ./watod down
   ./watod build mjlabs
   ./watod up mjlabs
   ```
2. Open the environment. Open your IDE/text editor in whatever directory you want using a new terminal tab:
   ```bash
   cd ~/humanoid
   code .
   ```
3. Test hardware on another new terminal tab:
   ```bash
   ./watod exec mjlabs bash
   nvidia-smi   # this should show you your own GPU
   python3 -c 'import mujoco; import jax; print(f"Hardware: {jax.devices()[0]}")'
   ```
   Neither of these should drop a "command not found" error.
4. At this point, `~/ament_ws` should contain the directory `/src/teleop`. This directory correlates to the `humanoid/autonomy/teleop` directory on the repository.

## Using the files in the testing folder

The testing folder has 4 main files:

1. **`cpu_test_headless.py`** → Runs RL training purely on the CPU with an environment count of 1024.
2. **`cpu_test_webview.py`** → Runs a live view of the given URDF (I used the `wato_binaural_arm` for testing).
3. **`train_ant.py`** → Trains a basic quadruped model, the mjlabs "Ant," to move as fast as possible. Generates the `ant_brain.pkl` file containing the optimized neural network weights → <https://gymnasium.farama.org/environments/mujoco/ant/>
4. **`enjoy_ant.py`** → Generates an HTML file after loading `ant_brain.pkl`, containing the video of a single environment using the "best strategy" as trained by `train_ant.py`.
    