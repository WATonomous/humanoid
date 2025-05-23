## CAN Interfacing Package Documentation

The documentation for the CAN interfacing package follows a standard WATOnomous package level scheme. As shown below:
```
can/
├── CMakeLists.txt
├── README.md
├── package.xml
├── config/
│   └── params.yaml
├── include/
│   ├── can_core.hpp
│   └── can_node.hpp
├── launch/
│   └── can.launch.py
├── src/
│   ├── can_core.cpp
│   └── can_node.cpp
└── test/
    └── test_can.cpp
```

### Package Level Overview

#### Purpose
This ROS 2 package serves as a communication bridge between a high-level controller and an embedded hand system via CAN bus. The package enables:
- Bidirectional communication between ROS 2 nodes and embedded systems
- Protocol translation between ROS 2 messages and CAN Bus frames
- Real-time feedback loop for hand control 
- Integration of embedded hand systems within the WATonomous ROS2 ecosystem

The package implements the following workflow:
1. Receive ROS 2 movement messages from the controller/behaviour node
2. Convert and send these messages as CAN frames through a USB CAN transceiver (accessible within a Docker container)
3. Receive CAN frames from the embedded system containing hand odometry
4. Convert and publish arm odometry as ROS2 messages for feedback to the controller

#### Inputs & Outputs
- Data flow, including message types, service calls, or file interactions

#### Key Features
- Key classes, nodes, or scripts, along with their relationships

#### Usage
- How to build, run, and test the package

#### Configuration
- Relevant parameters, environment variables, or dependencies

### Package Architecture
<!-- TODO: Create the architecture diagram of how messages are transmitted and received -->

### Infrastructure Documentation
1. [Documentation Structure of Repo](docs/README.md) 
2. [Project Infrastructure Development Docs](https://github.com/WATonomous/wato_monorepo/tree/main/docs/dev/)
3. [ROS Structure Docs](src/samples/README.md)