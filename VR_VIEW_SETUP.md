# VR View Path for Isaac Sim / Isaac Lab

This note is only about seeing the Isaac simulation from inside a VR headset.
It is not about the ROS finger-curl control path.

In the monorepo setup, the existing architecture can stay split like this:

```text
teleop container
  -> publishes finger curl / hand data over ROS

simulation container
  -> subscribes to those ROS topics
  -> bends the simulated fingers
  -> renders the Isaac world

XR / VR streaming path
  -> takes the rendered Isaac world
  -> streams it into the headset
```

The key point: the VR view is a rendering/streaming problem, not a ROS-control
problem. If your finger curls already bend the simulated hand correctly, you do
not need to redesign that part to get VR viewing.

## What This Folder Shows

This folder has the useful pattern for launching an Isaac Lab simulation that can
be driven from external input.

Relevant files:

- `run_quest2_so100_rosbridge_teleop.sh`
- `quest2_so100_rosbridge_teleop.py`
- `so100_marker_pick_place/tasks/marker_env_cfg.py`

The launcher starts Isaac Lab through `isaaclab.sh -p` and runs the Python
simulation script:

```bash
./isaaclab.sh -p quest2_so100_rosbridge_teleop.py
```

Inside `quest2_so100_rosbridge_teleop.py`, the app is created with:

```python
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
```

The important line for camera/rendering support is:

```python
args_cli.enable_cameras = True
```

That makes this folder a good reference for how to run an Isaac Lab scene that
has cameras/rendering enabled while the sim loop is controlled externally.

## What WebXR / CloudXR Adds

For headset viewing, Isaac needs to render stereo XR frames and send them to the
headset. The usual NVIDIA path is:

```text
Isaac Sim / Isaac Lab
  -> OpenXR output
  -> CloudXR runtime on the workstation
  -> WebXR / CloudXR client in the headset browser
  -> immersive VR view
```

So when you open the WebXR client in the headset, the browser is not running the
simulation. The sim is still running on the workstation or inside the simulation
container. The headset receives a low-latency stereo stream and sends headset
tracking data back through the XR runtime.

For your use case, the ROS finger-curl topics can remain separate:

```text
finger curl ROS topic -> simulation container -> simulated hand
VR headset view       -> CloudXR/WebXR stream -> user sees the sim from inside
```

## Target Integration in the Monorepo

After inspecting `~/Wato/humanoid`, the best place to add the immersive Isaac
experience is the simulation side:

- Compose file: `~/Wato/humanoid/modules/docker-compose.simulation.yaml`
- Image: `~/Wato/humanoid/docker/simulation/isaac_sim/isaac_sim.Dockerfile`
- Sim package: `~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop`
- Current Isaac entrypoint:
  `~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py`

Do not put the immersive Isaac view in the teleop container. The teleop container
is already doing the right job: it serves the Quest/WebXR hand-tracking page and
publishes `/quest_teleop` as `common_msgs/QuestHandPose`.

The simulation container should do three things:

1. Start Isaac Sim / Isaac Lab with XR support enabled.
2. Run `quest_Isaac.py`, which already subscribes to `/quest_teleop`.
3. Expose the rendered Isaac world through OpenXR/CloudXR so the headset can
   view it.

The teleop container can keep doing what it already does:

1. Read hand/finger data.
2. Publish finger curls over ROS.
3. Avoid caring about rendering or headset display.

This keeps the system clean:

```text
teleop container:
  autonomy/teleop/quest_teleop
  -> browser/WebXR hand tracking
  -> publishes /quest_teleop

simulation container:
  autonomy/simulation/quest_isaac_teleop/quest_Isaac.py
  -> subscribes /quest_teleop
  -> bends simulated fingers
  -> renders Isaac world
  -> should own XR/OpenXR/CloudXR output

headset:
  -> connects to teleop WebXR page for hand input
  -> connects to Isaac XR/CloudXR/WebXR stream for immersive sim view
```

## Practical Startup Shape

A working session should look roughly like this:

```bash
# 1. Start from the monorepo.
cd ~/Wato/humanoid

# 2. Start teleop and simulation modules.
# Depending on watod-config.local.sh, ACTIVE_MODULES should include:
#   teleop simulation
./watod up

# 3. Enter the teleop container and start the hand input bridge.
./watod -t teleop
ros2 run quest_teleop quest_teleop_node

# 4. Enter the simulation container and launch the Isaac hand sim.
./watod -t simulation_dev
cd /isaac-lab
./isaaclab.sh -p \
  /root/ament_ws/src/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py

# 5. Start/connect the XR client from the headset browser.
# The headset should connect to the workstation/container host running the XR stream.
```

The important missing change is that step 4 must become an XR-capable Isaac
launch. Today `quest_Isaac.py` is a normal Isaac Lab app. For immersive viewing,
that same app needs to run with Isaac's XR/OpenXR extensions enabled.

## Isaac Launch Requirement

For normal sim control, launching with `isaaclab.sh -p your_script.py` is enough.
For VR viewing, the same simulation must be launched with Isaac's XR/OpenXR
extensions enabled.

NVIDIA provides an Isaac Sim launcher named:

```bash
isaac-sim.xr.vr.sh
```

That launcher is the reference for "Isaac Sim with XR and VR enabled". In an
Isaac Lab workflow, the equivalent goal is:

```text
run your Isaac Lab simulation script
with OpenXR/XR extensions enabled
and with the CloudXR runtime selected as the OpenXR runtime
```

If this is containerized, the simulation container needs access to:

- the NVIDIA GPU
- Vulkan/OpenGL rendering support
- the XR/OpenXR runtime
- CloudXR runtime or equivalent streaming service
- network ports required by the headset client

Your existing `simulation` / `simulation_dev` services are already closer to
this than `teleop` because they have:

- `gpus: all`
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=all`
- `network_mode: host`
- X11 mounts
- Isaac Sim base image `nvcr.io/nvidia/isaac-sim:4.2.0`
- Isaac Lab installed at `/isaac-lab`

That means the likely implementation work belongs in:

1. `modules/docker-compose.simulation.yaml`
2. `docker/simulation/isaac_sim/isaac_sim.Dockerfile`
3. `autonomy/simulation/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py`
4. Maybe a helper launcher script under `autonomy/simulation/quest_isaac_teleop/scripts/`

The teleop compose file should usually stay unchanged except for normal ROS
networking variables.

## What To Copy From This Folder

From this folder, copy the launch pattern, not the SO100-specific robot code.

Useful pieces:

- Use an Isaac Lab Python entrypoint similar to `quest2_so100_rosbridge_teleop.py`.
- Set `args_cli.enable_cameras = True`.
- Create the Isaac app through `AppLauncher`.
- Keep the simulation stepping in a loop while ROS callbacks update robot state.
- Keep ROS input transport separate from XR rendering.

In Wato, the matching file is already:

```text
~/Wato/humanoid/autonomy/simulation/quest_isaac_teleop/quest_isaac_teleop/quest_Isaac.py
```

That file already has the right structure:

- `AppLauncher`
- `SimulationContext`
- `InteractiveScene`
- ROS subscription to `/quest_teleop`
- sim loop that applies finger targets and steps Isaac

So the first code-level change should probably be small:

```python
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
```

Then the bigger runtime change is launching that app with XR/OpenXR enabled in
the simulation container.

Do not copy the SO100 IK logic unless the target robot also needs that exact
end-effector IK behavior. For the hand/finger case, your existing subscriber
that bends fingers should remain the control path.

## Minimal Mental Model

When everything is working, this is what is happening:

```text
You move your hand
  -> teleop container publishes finger curls
  -> simulation container bends the Isaac hand
  -> Isaac renders the updated world
  -> CloudXR/WebXR streams the rendered world to the headset
  -> you see the simulated hand moving while standing inside the scene
```

That is the goal: keep the working ROS hand control, and add the XR rendering
path around the simulation container.

## References

- Isaac Lab CloudXR teleoperation:
  https://isaac-sim.github.io/IsaacLab/main/source/how-to/cloudxr_teleoperation.html
- Isaac Sim launch modes, including XR/VR launcher:
  https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html
