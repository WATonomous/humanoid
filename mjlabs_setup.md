# mjlab Setup

MuJoCo / mjlab runs via the **`simulation_mj`** (or **`simulation_mj.cpu`** for CPU-only systems) watod module (`mjlabs` service).

## Architectural notes

1. **`docker/simulation/mjlabs/mjlabs.Dockerfile`** — ROS 2 Humble base for mjlab (MuJoCo, mjviser, `jax[cuda12]`, brax).
2. **`modules/docker-compose.simulation_mj.yaml`** (or **`modules/docker-compose.simulation_mj.cpu.yaml`** for CPU-only) — `mjlabs` service. The GPU version includes GPU passthrough.
3. Mounts `autonomy/simulation`, `autonomy/teleop`, and `common_msgs` into the container.

## How to setup and reproduce

1. In `watod-config.local.sh`:

```bash
ACTIVE_MODULES="simulation_mj" # or "simulation_mj.cpu" if running without an NVIDIA GPU
MODE_OF_OPERATION="develop"
```

2. Build and start:

```bash
./watod down
./watod build
./watod up -d
./watod -t mjlabs
```

3. Inside the container, smoke-check GPU / JAX (if on a GPU machine, `nvidia-smi` should run; if on CPU, you can skip `nvidia-smi` and check JAX devices):

```bash
# Optional (GPU only)
nvidia-smi

# Check JAX devices (should print CPU or GPU depending on hardware config)
python3 -c 'import mujoco; import jax; print(f"Hardware: {jax.devices()[0]}")'
```

## Historical note

An early `autonomy/teleop/mojuco_test` folder and a `docker-compose.teleop.yaml` entry were used for CPU/GPU mjlab experiments; both are gone. Use `simulation_mj` / `mjlabs` instead.
