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

```bash
cp watod-config.sh watod-config.local.sh
# edit watod-config.local.sh: set ACTIVE_MODULES, e.g.
#   ACTIVE_MODULES="interfacing simulation_isaac"

pre-commit install

./watod up -d
```

`watod-config.sh` is shared defaults and is CI-guarded — never commit personal
changes to it. Everything local goes in `watod-config.local.sh` (gitignored).

Notes:

- Dev containers run as your host user (UID/GID), so bind-mounted files under
  `src/` are never root-owned.
- The first `simulation_isaac` image build pulls a large base image and takes a
  while. Later builds are cached.
- Isaac Lab / perception need X11 access: `xhost +local:docker`.

## Per-module dev loop

One editable container per active module. `src/<module>/` is bind-mounted in, so
edits on the host are live in the container.

```bash
./watod up -d                 # start active modules
./watod -t <service>          # shell into a service (interfacing, perception,
                              #   voxel_grid, joint_command, simulation_mj, ...)
./watod build <service>       # rebuild after a Dockerfile / dependency change
./watod down                  # stop
```

Inside the container, build and launch ROS packages by hand, e.g.:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch joint_command joint_command.launch.py
```

## CI / linting

- `pre-commit` runs the same checks locally and in CI (large files, merge
  conflicts, symlinks, AST/TOML syntax).
- `clang-format` runs on all C/C++ in CI. `src/simulation/**` and
  `src/embedded/STM32/lib/**` are excluded.
- `build_and_unitest.yml` builds and unit-tests every changed non-GPU module on
  each PR. GPU modules (`simulation_*`, `embedded`) are not built in CI.
- Don't merge with red checks.

## Notes

- **`ROS_DOMAIN_ID`** — if you and someone else run ROS nodes on the same
  machine or subnet at once, set a unique `ROS_DOMAIN_ID` (0–232) in your
  `watod-config.local.sh`. Otherwise your ROS graphs merge and you'll see each
  other's topics.
- **New ROS package** — copy an existing one (`src/interfacing/joint_command/`
  for C++, `src/perception/voxel_grid/` for Python) and rename.

## Per-area guides

| Area | Start here |
|------|------------|
| Isaac Lab sim / imitation learning | [docker/simulation/isaac_lab/QUICKSTART.md](docker/simulation/isaac_lab/QUICKSTART.md) · [src/il/README.md](src/il/README.md) |
| MuJoCo / mjlab | [docker/simulation/mjlabs/README.md](docker/simulation/mjlabs/README.md) |
| Teleop | [src/teleop/README.md](src/teleop/README.md) |
| CAN / hardware, arm bring-up | [src/interfacing/can/README.md](src/interfacing/can/README.md) · [ARM_BRINGUP.md](ARM_BRINGUP.md) |
| Messages | [src/common_msgs/README.md](src/common_msgs/README.md) |
| Design / subsystem docs | [docs/README.md](docs/README.md) → humanoid-docs site |
