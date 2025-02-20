# WATonomous + UWRL Humanoid Project
Dockerized ROS2 setup controlling and interfacing with Humanoid Hand

## Repo Structure
```
humanoid
├── watod-setup-env.sh
├── docker
│   ├── samples
│   │   └── cpp
│   │       ├── Dockerfile.aggregator
│   │       ├── Dockerfile.producer
│   │       └── Dockerfile.transformer
│   └── wato_ros_entrypoint.sh
├── docs
├── modules
│   └── docker-compose.samples.yaml
├── scripts
├── src
│   ├── perception
│   ├── wato_msgs
│   │   └── sample_msgs
│   │       ├── CMakeLists.txt
│   │       ├── msg
│   │       └── package.xml
│   ├── samples
│   │   ├── README.md
│   │   └── cpp
│   │       ├── aggregator
│   │       ├── image
│   │       ├── producer
│   │       ├── README.md
│   │       └── transformer
│   ├── controller
│   ├── interfacing
│   ├── simulation
└── watod
```
## Documentation
Documentation structure of this repo can be found [docs/README.md](docs/README.md)

Before developing please read these documents.

1. [Documentation Structure of Repo](docs/README.md)
2. [WATO Infrastructure Development Docs](https://github.com/WATonomous/wato_monorepo/tree/main/docs/dev/)
3. [ROS Node/Core Structure Docs](src/samples/README.md)

## Dependencies:
- Ubuntu >= 22.04, Windows (WSL), and MacOS.


