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
## Infra Documentation
1. [Project Infrastructure Development Docs](https://github.com/WATonomous/wato_monorepo/tree/main/docs/dev/)
2. [ROS Structure Docs](src/samples/README.md)

## Dependencies:
- Ubuntu >= 22.04, Windows (WSL), and MacOS.


