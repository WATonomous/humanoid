# Visualizing ROS2 Voxel Data in Isaac Lab (Sim)

This guide explains **what** each piece does, **why** it is needed, and **how** to do it, so you can visualize your dummy publisher → depth → voxel pipeline inside Isaac Lab’s simulation view.

---

## 1. Overview

### What we are doing

- You have a **dummy publisher** that emits depth (and optionally RGB and camera info) on ROS2.
- A **subscriber node** (voxel grid) turns that depth into a **voxel grid** and publishes it as a **PointCloud2** (voxel centers).
- You want to **visualize** that PointCloud2 inside **Isaac Lab (sim)** as 3D cubes (one per voxel).

### Why use the sim for this

- **Visual check**: Confirm that depth → voxels → ROS2 is correct before using real hardware or heavier pipelines.
- **Debugging**: See voxels in a 3D view (frame, scale, density) without extra tools.
- **Reuse**: Same ROS2 topics can later drive planning or other consumers; the sim is just one viewer.

### How it fits together (high level)

```
[Dummy Publisher]  →  /camera/depth/image_raw, /camera/depth/camera_info
        ↓
[Perception Node]  →  /perception/depth_processed, /perception/camera_info   (optional)
        ↓
[Voxel Grid Node]  →  /behaviour/voxel_grid (sensor_msgs/PointCloud2)
        ↓
[Isaac Lab Script] ←  subscribes to /behaviour/voxel_grid
        ↓
        →  spawns cuboids in the Isaac Sim stage (one per voxel center)
```

You can also use a **simplified pipeline** where the voxel node subscribes directly to `/camera/*` and publishes e.g. `/voxel_grid` (see Step 2). The rest of the guide still applies; only topic names and which nodes you launch change.

---

## 2. Prerequisites

### What you need

- **Isaac Lab** cloned in your monorepo (e.g. `IsaacLab/` next to `humanoid/`).
- **Isaac Sim** installed and working (via Isaac Lab’s `isaaclab.sh`, conda, or Docker).
- **ROS2** (Humble or later) so that:
  - The dummy publisher and voxel node can run.
  - The Isaac Lab visualization script can subscribe to PointCloud2 (same machine or same ROS2 domain over network).
- Your **humanoid** workspace built (`colcon build` for `perception`, `voxel_grid`, etc.).

### Why each is needed

- **Isaac Lab**: Provides the app launcher and sim utilities (scene, spawners) we use to show voxels.
- **Isaac Sim**: The actual simulator and renderer; Isaac Lab scripts run inside its Python.
- **ROS2**: Carries depth, camera info, and the voxel PointCloud2 between nodes and into the visualization script.

### How to check

- **Isaac Lab**: From repo root, `./IsaacLab/isaaclab.sh -h` should print help.
- **Isaac Sim**: Run `./IsaacLab/isaaclab.sh -p -c env_isaaclab` once if you use conda; then `conda activate env_isaaclab` and `isaaclab -p -c env_isaaclab` (or use the script path you use for other Isaac Lab scripts).
- **ROS2**: `source /opt/ros/<distro>/setup.bash` and `ros2 topic list` should work.
- **Humanoid packages**: From `humanoid/`, run `colcon build --packages-select perception voxel_grid` (or your actual package list) and `source install/setup.bash`.

---

## 3. Step 1: Run the data pipeline (dummy → voxels)

### What this step does

Starts the nodes that produce **depth** and then **voxel centers** on ROS2. No Isaac Lab yet; this is pure ROS2.

### Why it comes first

The visualization script will **subscribe** to the voxel topic. If nothing is publishing, there is nothing to draw. Running the pipeline first (or in parallel) ensures messages are available.

### How to do it

You have two options: **full pipeline** (dummy → perception → voxel_grid) or **short pipeline** (dummy → one voxel node that reads `/camera/*`).

**Option A – Full pipeline (current humanoid design)**

- **Dummy publisher** publishes:
  - `/camera/depth/image_raw` (Image, mono16, mm)
  - `/camera/color/image_raw` (Image, bgr8)
  - `/camera/depth/camera_info` (CameraInfo)
- **Perception node** subscribes to `/camera/*`, does minimal processing, and publishes:
  - `/perception/depth_processed`, `/perception/rgb_processed`, `/perception/camera_info`
- **Voxel grid node** subscribes to `/perception/*` and publishes:
  - `/behaviour/voxel_grid` (PointCloud2, voxel centers as x,y,z)

Launch order (three terminals or one launch file):

```bash
# Terminal 1 – Dummy camera
source /opt/ros/humble/setup.bash
source /path/to/humanoid/install/setup.bash
ros2 run perception dummy_publisher_node
```

```bash
# Terminal 2 – Perception (bridges /camera/* → /perception/*)
ros2 run perception perception_node
```

```bash
# Terminal 3 – Voxel grid (subscribes to /perception/*, publishes /behaviour/voxel_grid)
ros2 run voxel_grid voxel_grid_node
```

**Option B – Short pipeline (dummy → voxel only)**

If you use a voxel node that subscribes **directly** to `/camera/depth/image_raw` and `/camera/depth/camera_info` (e.g. a small script or `humanoid/utils/voxel/ros2_voxel.py` adapted as a node), then you only need:

- Terminal 1: dummy publisher (same as above).
- Terminal 2: voxel node that publishes e.g. `/voxel_grid` (PointCloud2).

In that case, the visualization script (Step 4) should subscribe to **that** topic (e.g. `/voxel_grid`) instead of `/behaviour/voxel_grid`.

**Check that voxels are published**

```bash
ros2 topic echo /behaviour/voxel_grid --once
# or
ros2 topic echo /voxel_grid --once
```

You should see a `sensor_msgs/PointCloud2` with `height=1`, `width=N`, and `data` containing x,y,z (and possibly padding) for each voxel center.

---

## 4. Step 2: Make sure Isaac Lab can see ROS2

### What we need

The Isaac Lab visualization script runs **inside** Isaac Sim’s Python process. To subscribe to the voxel topic, that process must:

- Have **rclpy** (and ROS2) available, and
- Use the **same ROS2 domain** as the dummy/voxel nodes (same `ROS_DOMAIN_ID` and reachable network).

### Why it matters

If ROS2 and Isaac Sim use different domains or different machines without proper DDS config, the script will never receive PointCloud2 messages and no voxels will appear.

### How to do it

- **Same machine**: Use the same `ROS_DOMAIN_ID` for all terminals (e.g. leave unset so everyone uses 0).
- **Docker**: If the pipeline runs in one container and Isaac Lab in another, either:
  - Run them on the same host with the same `ROS_DOMAIN_ID` and host network, or
  - Configure DDS (e.g. Cyclone DDS or Fast DDS) so that both containers use the same discovery and multicast/unicast.
- **Isaac Sim + ROS2 in one env**: Isaac Lab’s Docker image (e.g. with `.env.ros2`) or a conda env that has both Isaac Sim and `ros-humble-*` (and thus `rclpy`) allows a **single** script to start the sim and subscribe to ROS2 in the same process. That is the intended setup for the visualization script below.

---

## 5. Step 3: Isaac Lab script (subscribe + spawn cuboids)

### What the script does

1. **Launch** Isaac Sim via Isaac Lab’s `AppLauncher` (so the sim and rendering backend are ready).
2. **Create a minimal scene**: ground plane and a light.
3. **Subscribe** to the voxel PointCloud2 topic (e.g. `/behaviour/voxel_grid` or `/voxel_grid`) using **rclpy**.
4. In the **simulation loop** (each frame):
   - Call `rclpy.spin_once()` (or a short timeout) to get the latest message.
   - If a new PointCloud2 was received, **clear** previously spawned voxel prims (optional, or update in place).
   - **Spawn** one **cuboid** per voxel center using Isaac Lab’s `sim_utils.CuboidCfg` (or `MeshCuboidCfg`) so voxels appear as small boxes in the scene.

### Why it is structured this way

- **AppLauncher first**: Isaac Sim must be the first thing imported/started; all other imports (e.g. `rclpy`, `isaaclab.sim`) come after so the Omniverse kernel is initialized correctly.
- **Minimal scene**: So you have a ground and light; otherwise the view is dark and you may not see the voxels.
- **ROS2 in the same process**: Keeps one executable: one script that both runs the sim and consumes ROS2. Alternatively you could run a separate ROS2 node that writes voxels to a file/socket and have the script read that; same idea, more moving parts.
- **Cuboids per voxel**: Isaac Lab’s spawners give you prims (e.g. cuboids) at given positions; one small cube per PointCloud2 point is the natural mapping for “voxel visualization”.

### How to implement it (conceptually)

- **Imports (after AppLauncher)**  
  Use `isaaclab.sim` as `sim_utils`, `rclpy`, and `sensor_msgs.msg.PointCloud2`. Decode PointCloud2 (e.g. `sensor_msgs_py.point_cloud2` or manual parsing of `data` with `point_step` and `fields`) to get an N×3 array of positions in meters.

- **Scene setup**  
  Create `SimulationContext(sim_utils.SimulationCfg(dt=...))`, then spawn:
  - `sim_utils.GroundPlaneCfg()` for the ground.
  - `sim_utils.DistantLightCfg(...)` or `DomeLightCfg(...)` for lighting.
  Call `sim.reset()` once after design.

- **ROS2**  
  `rclpy.init()`, create a node, subscribe to `/behaviour/voxel_grid` (or `/voxel_grid`) with `PointCloud2`. In a callback, store the latest point cloud (e.g. decode to numpy N×3) and set a “dirty” flag. Do **not** block in the callback; keep it short.

- **Voxel prims**  
  Keep a list (or a fixed max count) of prim paths you created, e.g. `/World/Voxels/voxel_000`, … . When you have new data:
  - Delete existing voxel prims (e.g. `prim_utils.delete_prim(path)` for each, or delete the parent Xform and recreate it).
  - For each point in the new point cloud (optionally subsample if N is huge), call:
    - `cfg = sim_utils.CuboidCfg(size=(voxel_size, voxel_size, voxel_size), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 1.0)))`
    - `cfg.func(prim_path, cfg, translation=(x, y, z))`
  Use the same `voxel_size` as in your voxel grid node (e.g. 0.04 m) so the cubes match the logical voxel size.

- **Main loop**  
  `while simulation_app.is_running():`  
  - `rclpy.spin_once(node, timeout_sec=0.01)`  
  - If the latest point cloud is updated, clear old voxels and spawn new ones (as above).  
  - `sim.step()`  
  - Optionally `sim.set_camera_view(...)` once so the camera looks at the voxel cloud.

- **Shutdown**  
  After the loop, `node.destroy_node()`, `rclpy.shutdown()`, then `simulation_app.close()`.

Important: **AppLauncher and `simulation_app`** must be created at the top of the script (before any other Isaac/Omniverse imports), and the rest of the script (including `import rclpy`) must be **after** that block, as in existing Isaac Lab tutorials (e.g. `launch_app.py`, `spawn_prims.py`).

---

## 6. Step 4: Run the visualization

### What you are doing

Running the script from Step 3 with Isaac Lab’s launcher so it uses Isaac Sim’s Python and environment.

### Why use `isaaclab.sh -p`

Isaac Sim expects to be started in a specific way; `isaaclab.sh -p` uses the correct Python and environment (conda or Isaac Sim’s Python) and runs your script in that context. That way `isaaclab.sim` and the simulator backend are available.

### How to run

From your monorepo root (or the directory that contains your script):

```bash
# If your script is inside humanoid and uses humanoid packages, set PYTHONPATH so imports resolve
export PYTHONPATH=/path/to/humanoid/autonomy/simulation:$PYTHONPATH

# Run the voxel visualization script with Isaac Lab’s Python
/path/to/IsaacLab/isaaclab.sh -p /path/to/humanoid/autonomy/simulation/Wato_additional_scripts/voxel_viz/isaaclab_voxel_viz.py
```

If you use conda:

```bash
conda activate env_isaaclab
PYTHONPATH=/path/to/humanoid/autonomy/simulation $ISAACLAB_PATH/isaaclab.sh -p /path/to/.../isaaclab_voxel_viz.py
```

Ensure the **dummy publisher** and **voxel node** are already running (Step 1) and that **ROS_DOMAIN_ID** (and network) are consistent (Step 2). Then you should see the sim window and, after a short delay, voxels appearing as small cubes.

---

## 7. Summary table

| Step | What | Why | How |
|------|------|-----|-----|
| 1 | Run dummy + (optionally perception) + voxel node | Produce PointCloud2 voxel data on ROS2 | Launch nodes; verify with `ros2 topic echo` |
| 2 | Same ROS2 domain / network for Isaac Lab | So the viz script receives messages | Same machine + same ROS_DOMAIN_ID; or configured DDS across containers |
| 3 | Script: AppLauncher → scene → subscribe → spawn cuboids in loop | Show voxels in sim | One script: sim + rclpy + decode PointCloud2 + spawn/clear CuboidCfg each update |
| 4 | Run script with `isaaclab.sh -p` | Use correct Isaac Sim Python/env | `isaaclab.sh -p path/to/isaaclab_voxel_viz.py` and suitable PYTHONPATH |

---

## 8. Troubleshooting

- **No voxels in the sim**
  - Confirm the voxel topic is published: `ros2 topic hz /behaviour/voxel_grid` (or your topic).
  - Confirm the script is in the same ROS2 domain and can see the topic: `ros2 topic list` from the same host/container.
  - Check that the script actually receives messages (e.g. log `len(points)` when you get a new PointCloud2).
  - Ensure **frame_id** and **units**: your voxel node publishes in **meters** and the script uses the same; if the cloud is in `camera_link`, the cubes will appear in sim in that frame (you may need to add a static TF or spawn under a parent Xform that matches your robot/camera frame).

- **Voxels in the wrong place**
  - PointCloud2 is usually in the **camera** or **sensor** frame. The script spawns cubes at (x,y,z) from the message; if your sim world is different (e.g. Y-up, different origin), you may need to transform the points (e.g. flip axes or apply a fixed transform) before spawning.

- **Too many voxels / slow**
  - Subsample: e.g. use every 2nd or 4th point, or cap at 500–2000 cubes.
  - Reuse prims: instead of delete-all and re-spawn, update positions of a fixed pool of cuboids (more complex, but avoids creating/destroying thousands of prims every frame).

- **rclpy not found in Isaac Sim**
  - Your Isaac Sim Python may not have ROS2. Use an environment that has both (e.g. Isaac Lab Docker image with ROS2 support, or a conda env where you installed `ros-humble-*` and sourced it before running `isaaclab.sh -p`), or implement a small bridge: a separate ROS2 node that writes voxels to a file or socket, and the Isaac Lab script reads from that file/socket instead of ROS2.

---

## 9. Next steps

- **Parameterize** voxel size and topic name (e.g. via `argparse` or a config file) so you can match your voxel node and tune visualization.
- **Add a TF or fixed offset** so the voxel cloud appears in the right place relative to your robot or world in the sim.
- **Reuse** the same pattern for OctoMap or other world representations by subscribing to their visualization topics (e.g. also PointCloud2 or MarkerArray) and spawning corresponding prims in Isaac Lab.

This gives you a full path: **dummy publisher → depth → voxel node → PointCloud2 → Isaac Lab sim** with clear what/why/how for each step.
