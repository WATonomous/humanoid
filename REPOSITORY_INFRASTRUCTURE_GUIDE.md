# WATonomous Humanoid Project - Repository Structure & Infrastructure Guide

## Overview

This repository contains a Dockerized ROS2 monorepo for the WATonomous + UWRL Humanoid Project, implementing a complete autonomous robotics system with perception, control, and simulation capabilities. The entire infrastructure is containerized using Docker and orchestrated via Docker Compose.

## Repository Structure

```
humanoid/
├── LICENSE
├── README.md                           # Main project README
├── watod                               # Main orchestration script
├── watod-config.sh                     # Main configuration file
├── watod-config.local.sh               # Local configuration overrides (gitignored)
├── watod_scripts/                      # Setup and utility scripts
│   ├── watod-completion.bash          # Bash completion for watod command
│   ├── watod-setup-dev-env.sh         # Development environment setup
│   └── watod-setup-docker-env.sh      # Docker environment setup
├── autonomy/                          # Core ROS2 packages and nodes
│   ├── perception_pkg/                # Perception system components
│   │   ├── config/                    # Configuration files
│   │   ├── launch/                    # ROS2 launch files
│   │   └── perception_pkg/            # Main perception package
│   ├── samples/                       # Sample ROS2 nodes for reference
│   │   ├── README.md                  # Samples documentation
│   │   ├── cpp/                       # C++ sample implementations
│   │   │   ├── aggregator/            # Data aggregation sample
│   │   │   ├── producer/              # Data production sample
│   │   │   └── transformer/           # Data transformation sample
│   │   ├── python/                    # Python sample implementations
│   │   │   ├── aggregator/
│   │   │   ├── producer/
│   │   │   └── transformer/
│   │   ├── sample_msgs/               # Custom ROS2 message definitions
│   │   └── samples_diagram.svg        # Architecture diagram
│   ├── simulation/                    # Simulation components
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── Humanoid_RL/               # Reinforcement learning components
│   │   └── Wato_additional_scripts/   # Additional simulation scripts
│   └── wato_msgs/                     # WATonomous custom messages
│       ├── common_msgs/               # Common message types
│       └── sample_msgs/               # Sample-specific messages
├── config/                            # Configuration files
│   └── wato_asd_training_foxglove_config .json  # Foxglove visualization config
├── docker/                            # Docker infrastructure
│   ├── base/                          # Base Docker images
│   │   ├── inject_ros2.Dockerfile     # ROS2 injection base
│   │   └── inject_wato_base.Dockerfile # WATonomous base
│   ├── controller/                    # Controller-specific Dockerfiles
│   │   └── voxel_grid/                # Voxel grid controller
│   ├── infrastructure/                # Infrastructure services
│   │   └── foxglove/                  # Foxglove visualization
│   ├── samples/                       # Sample service Dockerfiles
│   │   ├── cpp_aggregator.Dockerfile
│   │   ├── cpp_producer.Dockerfile
│   │   ├── cpp_transformer.Dockerfile
│   │   ├── py_aggregator.Dockerfile
│   │   ├── py_producer.Dockerfile
│   │   └── py_transformer.Dockerfile
│   ├── simulation/                    # Simulation Dockerfiles
│   │   └── isaac_sim/                 # Isaac Sim integration
│   ├── voxel/                         # Voxel processing utilities
│   └── wato_ros_entrypoint.sh         # ROS2 environment setup script
├── docs/                              # Documentation
│   ├── Architecture_Map.odg          # System architecture diagram
│   ├── OctoMap_Implementation_Guide.md # OctoMap implementation guide
│   └── README.md                      # Documentation guide
├── embedded/                          # Embedded systems code
│   ├── README.md
│   ├── ESP32S3/                       # ESP32 microcontroller code
│   │   └── gimbal_motor_testing/      # Gimbal motor control
│   └── STM32/                         # STM32 microcontroller code
│       ├── platformio.ini            # PlatformIO configuration
│       ├── app/                       # Application code
│       ├── bootloader/                # Bootloader code
│       ├── include/                   # Header files
│       ├── lib/                       # Libraries
│       ├── src/                       # Source files
│       └── test/                      # Test files
├── modules/                           # Docker Compose modules (current)
│   ├── .env                           # Environment variables (auto-generated)
│   ├── docker-compose.controller.yaml # Controller services
│   ├── docker-compose.infrastructure.yaml # Infrastructure services
│   └── docker-compose.samples.yaml    # Sample services
├── modules_future/                    # Future Docker Compose modules
│   ├── docker-compose.controller.yaml
│   ├── docker-compose.interfacing.yaml
│   ├── docker-compose.perception.yaml
│   └── docker-compose.simulation.yaml
├── STM32/                             # Legacy STM32 code (deprecated)
├── utils/                             # Utility scripts and tools
│   ├── README.md                      # Utils documentation
│   ├── boilerplate-files/             # ROS2 package templates
│   │   ├── foo_core.cpp              # C++ core template
│   │   ├── foo_core.hpp              # C++ core header template
│   │   ├── foo_core.py               # Python core template
│   │   ├── foo_node.cpp              # C++ node template
│   │   ├── foo_node.hpp              # C++ node header template
│   │   ├── foo_node.py               # Python node template
│   │   └── foo_test.cpp              # C++ test template
│   ├── create_package.bash           # ROS2 package creation script
│   └── voxel/                        # Voxel processing utilities
│       ├── bunnyData.pts             # Test point cloud data
│       ├── readme.md                 # Voxel utils documentation
│       ├── ros2_octcloud.py          # ROS2 octree cloud utilities
│       ├── ros2_voxel.py             # ROS2 voxel grid implementation
│       ├── test_voxel.py             # Voxel testing script
│       └── spconv/                   # Sparse convolution utilities
└── watod                             # Main orchestration script (symlink)
```

## Infrastructure Components

### 1. WATOD Orchestration System

#### Main Script (`watod`)
- **Location**: `/watod`
- **Purpose**: Main command-line interface for managing the entire Docker infrastructure
- **Functionality**:
  - Wraps Docker Compose commands with project-specific configuration
  - Manages multiple modules and profiles
  - Provides terminal access to running containers
  - Handles environment variable injection

**Key Commands**:
```bash
watod up                    # Start all active modules
watod down                  # Stop and remove containers
watod ps                    # List container status
watod -t <service_name>     # Open terminal in container
watod logs <service_name>   # View container logs
```

#### Configuration Files
- **`watod-config.sh`**: Main configuration file with default settings
- **`watod-config.local.sh`**: Local overrides (gitignored) for personal configuration

**Key Configuration Options**:
```bash
ACTIVE_MODULES="samples infrastructure"  # Modules to run
MODE_OF_OPERATION="develop"              # "develop" or "deploy"
COMPOSE_PROJECT_NAME="watod_username"    # Docker project name
TAG="main"                              # Docker image tag
PLATFORM="amd64"                        # Build platform
```

### 2. Docker Infrastructure

#### Base Images (`docker/base/`)
- **`inject_ros2.Dockerfile`**: Minimal ROS2 environment
- **`inject_wato_base.Dockerfile`**: WATonomous base with additional dependencies

#### Service Dockerfiles
Located in `docker/` subdirectories, each service has its own Dockerfile:
- **Multi-stage builds**: Source → Dependencies → Build
- **ROS2 integration**: Uses `wato_ros_entrypoint.sh` for environment setup
- **Dependency management**: Automatic rosdep resolution

#### Entrypoint Script (`docker/wato_ros_entrypoint.sh`)
```bash
#!/bin/bash
set -e
source /opt/watonomous/setup.bash
exec "$@"
```
- Sets up ROS2 environment before running container commands
- Sources WATonomous-specific ROS2 workspace

### 3. Module System

#### Current Modules (`modules/`)
- **`docker-compose.controller.yaml`**: Control systems (motor control, trajectory planning)
- **`docker-compose.infrastructure.yaml`**: Supporting services (visualization, logging)
- **`docker-compose.samples.yaml`**: Example ROS2 nodes for reference

#### Future Modules (`modules_future/`)
- **`docker-compose.controller.yaml`**: Advanced control systems
- **`docker-compose.interfacing.yaml`**: Hardware interface nodes
- **`docker-compose.perception.yaml`**: Computer vision and sensor processing
- **`docker-compose.simulation.yaml`**: Simulation environments

#### Environment Configuration (`modules/.env`)
Auto-generated file containing:
- Docker image names and tags
- Port mappings
- Project-specific variables
- Registry URLs

### 4. Development Environment Setup

#### Setup Scripts (`watod_scripts/`)
- **`watod-setup-dev-env.sh`**: Creates development environment
  - Mounts ROS2 dependencies from containers to host
  - Generates VS Code configuration for IntelliSense
  - Enables IDE support for ROS2 development

- **`watod-setup-docker-env.sh`**: Configures Docker environment
  - Sets up registry authentication
  - Configures build parameters
  - Generates `.env` files

#### Development Workflow
1. Run `watod-setup-dev-env.sh` to mount dependencies
2. Edit code on host (volumes mounted in containers)
3. Use `watod -t <service>` to access container terminals
4. Changes reflect immediately in running containers

## Core Components

### Autonomy Stack (`autonomy/`)

#### Perception Package (`autonomy/perception_pkg/`)
- **Purpose**: Computer vision and sensor processing
- **Structure**:
  - `config/`: Parameter files
  - `launch/`: ROS2 launch configurations
  - `perception_pkg/`: Main perception algorithms

#### Samples (`autonomy/samples/`)
- **Purpose**: Reference implementations following WATonomous conventions
- **Architecture**: Producer → Transformer → Aggregator pipeline
- **Languages**: Both C++ and Python implementations available
- **Paradigm**: "Core Logic + Node Logic" separation
  - **Core Logic**: Algorithm implementations (no ROS2 dependencies)
  - **Node Logic**: ROS2 interfaces (publishers, subscribers, etc.)

#### Simulation (`autonomy/simulation/`)
- **Isaac Sim Integration**: NVIDIA Isaac Sim environment
- **Humanoid RL**: Reinforcement learning components
- **Additional Scripts**: ROS2 bridge and hand controller scripts

#### Custom Messages (`autonomy/wato_msgs/`)
- **`common_msgs/`**: Shared message types across the system
- **`sample_msgs/`**: Message definitions for sample nodes

### Embedded Systems (`embedded/`)

#### ESP32S3 (`embedded/ESP32S3/`)
- **Purpose**: Microcontroller code for peripheral devices
- **Applications**: Gimbal motor control, sensor interfaces

#### STM32 (`embedded/STM32/`)
- **Purpose**: Main microcontroller firmware
- **Structure**:
  - `platformio.ini`: PlatformIO configuration
  - `src/`: Main application code
  - `include/`: Header files
  - `lib/`: External libraries
  - `test/`: Unit tests
  - `bootloader/`: Custom bootloader

### Utilities (`utils/`)

#### Package Creation (`utils/create_package.bash`)
- **Usage**: `./create_package.bash <package_name> <module_name> [-p]`
- **Functionality**:
  - Creates ROS2 packages with proper structure
  - Generates boilerplate code (C++/Python)
  - Updates Docker Compose files automatically
  - Supports both C++ and Python packages

#### Boilerplate Files (`utils/boilerplate-files/`)
Templates for:
- ROS2 nodes (C++/Python)
- Core logic classes
- Launch files
- Configuration files
- Test files

#### Voxel Utilities (`utils/voxel/`)
- **Purpose**: 3D spatial processing utilities
- **`ros2_voxel.py`**: ROS2 voxel grid implementation (your current file)
- **`ros2_octcloud.py`**: Octree point cloud processing
- **Dependencies**: PyTorch, spconv (sparse convolutions)

## Configuration and Deployment

### Configuration Hierarchy
1. **Global defaults**: `watod-config.sh`
2. **Local overrides**: `watod-config.local.sh` (recommended)
3. **Environment variables**: `modules/.env` (auto-generated)
4. **Runtime parameters**: ROS2 parameter server

### Deployment Modes

#### Development Mode
- **Profile**: `develop`
- **Characteristics**:
  - Source code mounted as volumes
  - Live code reloading
  - Debug builds
  - Full development tools available

#### Deploy Mode
- **Profile**: `deploy`
- **Characteristics**:
  - Pre-built optimized images
  - Production configurations
  - Minimal container sizes
  - Release builds

### Service Architecture

Each service follows a consistent pattern:
1. **Base Image**: ROS2 + WATonomous dependencies
2. **Source Copy**: Application code copied into container
3. **Dependency Resolution**: Automatic rosdep installation
4. **Build Process**: Colcon build with optimized flags
5. **Runtime**: Entrypoint sets up ROS2 environment

## Development Workflow

### Setting Up Development Environment
```bash
# 1. Configure active modules
cp watod-config.sh watod-config.local.sh
# Edit watod-config.local.sh to set ACTIVE_MODULES

# 2. Setup Docker environment
./watod_scripts/watod-setup-docker-env.sh

# 3. Setup development environment
./watod_scripts/watod-setup-dev-env.sh

# 4. Start services
watod up
```

### Creating New Packages
```bash
# Create a new Python package in perception module
./utils/create_package.bash my_perception_node perception -p

# Create a new C++ package in controller module
./utils/create_package.bash my_controller_node controller
```

### Working with Containers
```bash
# Access a running container
watod -t perception

# View logs
watod logs perception

# Rebuild and restart a service
watod up --build perception
```

## Key Design Principles

### 1. Containerization First
- Everything runs in containers for consistency
- Development and production use same images
- Easy scaling and deployment

### 2. Modular Architecture
- Services are loosely coupled via ROS2
- Modules can be enabled/disabled independently
- Clear separation of concerns

### 3. Development Experience
- Live editing with volume mounts
- IDE integration with proper tooling
- Consistent development environments

### 4. ROS2 Best Practices
- Core/Node logic separation for testability
- Proper message definitions
- Launch file configurations
- Parameter management

### 5. Infrastructure as Code
- All infrastructure defined in code
- Reproducible builds
- Version-controlled configurations

This infrastructure provides a robust, scalable foundation for developing autonomous robotics systems with proper separation between development and deployment environments.</content>
<parameter name="filePath">/home/hassaan141/humanoid/REPOSITORY_INFRASTRUCTURE_GUIDE.md