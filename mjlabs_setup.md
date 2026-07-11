# mjlab Setup

MuJoCo / mjlab runs via the **`simulation_mj`** watod module (`mjlabs` service).

## Architectural notes

1. **`docker/simulation/mjlabs/mjlabs.Dockerfile`** — ROS 2 Humble base for mjlab (MuJoCo, mjviser, `jax[cuda12]`, brax).
2. **`modules/docker-compose.simulation_mj.yaml`** — `mjlabs` service with GPU passthrough and port 8080 for mjviser.
3. Mounts `autonomy/simulation`, `autonomy/teleop`, and `common_msgs` into the container.

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
./watod -t mjlabs
```

3. Inside the container, smoke-check GPU / JAX:

```bash
nvidia-smi
python3 -c 'import mujoco; import jax; print(f"Hardware: {jax.devices()[0]}")'
```

## Historical note

An early `autonomy/teleop/mojuco_test` folder and a `docker-compose.teleop.yaml` entry were used for CPU/GPU mjlab experiments; both are gone. Use `simulation_mj` / `mjlabs` instead.
