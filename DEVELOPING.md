# Developing

How to get a dev environment running and work on a module. For what the repo
contains and the high-level layout, see [README.md](README.md).

## Prerequisites

- Ubuntu ≥ 22.04 (WSL / macOS work for the non-GPU modules only)
- Docker + Docker Compose v2
- NVIDIA GPU + drivers + container toolkit for `perception`, `simulation_isaac`,
  `simulation_mj`
- `pre-commit` (`pipx install pre-commit` or `pip install --user pre-commit`)

## First-time setup

Follow [Quick start](README.md#quick-start) in the README to create
`watod-config.local.sh` and start your containers, then once per clone:

```bash
pre-commit install
```

`watod-config.sh` is shared defaults and is CI-guarded — never commit personal
changes to it. Everything local goes in `watod-config.local.sh` (gitignored).

Notes:

- Dev containers run as your host user (UID/GID), so bind-mounted files under
  `src/` are never root-owned.
- The first `simulation_isaac` image build pulls a large base image and takes a
  while. Later builds are cached.
- Isaac Lab / perception need X11 access: `xhost +local:docker`.

## Inside the container

Build and launch ROS packages by hand, e.g.:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch joint_command joint_command.launch.py
```

## CI and things to know

- `pre-commit` runs the same checks locally and in CI.
- `clang-format` runs on all C/C++ in CI, except `src/simulation/**` and
  `src/embedded/STM32/lib/**`.
- CI builds and unit-tests changed non-GPU modules only. `simulation_*` and
  `embedded` are **not** built in CI — test those yourself.
- **`ROS_DOMAIN_ID`** — set a unique value (0–232) in `watod-config.local.sh` if
  someone else runs ROS on the same subnet, or you will see each other's topics.
- **New ROS package** — copy `src/interfacing/joint_command/` (C++) or
  `src/perception/voxel_grid/` (Python) and rename.
