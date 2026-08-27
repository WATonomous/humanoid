# Keyboard-Based Teleoperation for WATO Humanoid Bimanual Arm (MuJoCo / Warp)

This directory contains scripts for keyboard-based teleoperation of the bimanual WATO humanoid arm using either MuJoCo or NVIDIA Warp/Newton.

## Prerequisites & Docker Environment Setup

1. **Configure Docker environment:**
   Create or edit the local config file `watod-config.local.sh` in the root of the repository:
   ```bash
   # Use "simulation_mj" if running with an NVIDIA GPU, or "simulation_mj.cpu" for CPU-only systems
   ACTIVE_MODULES="simulation_mj" 
   MODE_OF_OPERATION="develop"
   ```

2. **Build and start the services:**
   Run the following commands in the root of the humanoid workspace directory:
   ```bash
   ./watod down
   ./watod build
   ./watod up -d
   ```

3. **Attach to the `mjlabs` service:**
   Start an interactive bash shell in the running service container:
   ```bash
   ./watod -t mjlabs
   ```

---

## Running the Simulation

### Option A: NVIDIA Warp + Newton Physics Simulation (Recommended)
This uses NVIDIA Warp on CPU/GPU to run an XPBD (Extended Position-Based Dynamics) rigid body solver, using a Viser-based standalone browser visualizer.

1. **Install Python package dependencies** (inside the container shell):
   ```bash
   pip install warp-lang newton viser trimesh
   ```

2. **Launch the teleoperation script:**
   ```bash
   python3 /root/ament_ws/src/simulation/Teleop/mjlabs_keyboard_based_teleoperation/newton_keyboard_teleop.py
   ```

3. **Access the visualizer:**
   Open [http://localhost:8080](http://localhost:8080) (or the port shown in your terminal output) in any web browser.

#### Controls
* **I / K** : Move target marker along X-axis (+/-)
* **J / L** : Move target marker along Y-axis (+/-)
* **U / O** : Move target marker along Z-axis (+/-)
* **C / D** : Rotate right wrist joint roll (+/-) (limit: $[-180^\circ, 180^\circ]$)
* **G**     : Toggle gripper fingers (Open/Closed)
* **R**     : Reset right arm to defaults

---

### Option B: Pure MuJoCo Simulation (Fallback)
This runs the original, pure MuJoCo simulation using the `mjviser` viewer.

1. **Launch the teleoperation script** (inside the container shell):
   ```bash
   python3 /root/ament_ws/src/simulation/Teleop/mjlabs_keyboard_based_teleoperation/mjlabs_keyboard_teleop.py
   ```

#### Controls
* **I / K** : Move target marker along X-axis (+/-)
* **J / L** : Move target marker along Y-axis (+/-)
* **U / O** : Move target marker along Z-axis (+/-)
* **G**     : Toggle gripper fingers (Open/Closed)
* **R**     : Reset left arm to defaults
