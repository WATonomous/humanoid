# mjlab Setup

MuJoCo / mjlab runs via the **`simulation_mj`** watod module (service `simulation_mj`).

## Architectural notes

1. **`docker/simulation/mjlabs/mjlabs.Dockerfile`** — ROS 2 Humble base for mjlab (MuJoCo, mjviser, `jax[cuda12]`, brax).
2. **`modules/docker-compose.simulation_mj.yaml`** — `simulation_mj` service with GPU passthrough and port 8080 for mjviser.
3. Mounts `src/simulation`, `src/teleop`, `common_msgs` and `joint_command` into `/root/ament_ws/src`, and the whole repo at `/workspace/humanoid`.

## How to setup and reproduce

1. In `watod-config.local.sh`:

```bash
ACTIVE_MODULES="simulation_mj"
MODE_OF_OPERATION="develop"
```

2. Build and start:

```bash
./watod down
./watod build
./watod up -d
./watod -t simulation_mj
```

3. Inside the container, smoke-check GPU / JAX:

```bash
nvidia-smi
python3 -c 'import mujoco; import jax; print(f"Hardware: {jax.devices()[0]}")'
```
