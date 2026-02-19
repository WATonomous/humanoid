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
├── scripts/
│   └── setup_can.sh
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

**Architecture Design Pattern: Core vs Node Separation**

This package follows a clean architecture pattern that separates concerns between hardware abstraction and ROS integration:

- **CanCore** (`can_core.hpp/.cpp`): 
  - Pure C++ class handling low-level CAN bus operations
  - Hardware abstraction layer for CAN communication
  - Reusable component independent of ROS 
  - Responsibilities: socket management, frame transmission/reception, interface setup
  - Could be used in non-ROS applications or embedded systems

- **CanNode** (`can_node.hpp/.cpp`):
  - ROS 2 Node class managing ROS ecosystem integration
  - Message translation between ROS and CAN protocols
  - ROS-specific functionality: publishers, subscribers, parameters, timers
  - Responsibilities: ROS message handling, topic management, service calls

This separation provides:
- **Single Responsibility**: Each class has one clear purpose
- **Testability**: CanCore can be unit tested without ROS overhead  
- **Reusability**: CanCore can be used in other projects
- **Maintainability**: CAN protocol changes only affect CanCore, ROS changes only affect CanNode

#### Usage

**Building the Package**
Build the package using the standard ROS 2 build process within the interfacing container.

**Running the CAN Node**
1. Start the interfacing container
2. Access the container: `sudo ./watod -t interfacing`
3. Source the environment: `source ./watod_ros_entrypoint.sh`
4. Launch the CAN node: `ros2 launch can can.launch.py`

**Note**: The launch file includes a test controller node for development/testing. To disable it for production use, comment out the test controller section in `launch/can.launch.py` (lines marked with "TEST CONTROLLER NODE" comments).

#### Configuration
- Relevant parameters, environment variables, or dependencies

#### Dependencies
- List of required packages, libraries, or tools
  - `rclcpp`: ROS 2 C++ client library
  - `can_msgs`: Custom message definitions for CAN frames
  - `socketcan`: Linux socket CAN interface
  - `ament_cmake`: Build system for ROS 2 packages

#### Config 
- Configuration files, environment variables, or command-line arguments
  - `config/params.yaml`: Contains parameters for CAN interface setup, such as:
    - `can_interface`: Name of the CAN interface (e.g., `can0`)
    - `baud_rate`: CAN bus speed (e.g., `500000`)
    - `frame_timeout`: Timeout for receiving frames in milliseconds

### Package Architecture
<!-- TODO: Create the architecture diagram of how messages are transmitted and received -->

### Infrastructure Documentation
1. [Documentation Structure of Repo](docs/README.md) 
2. [Project Infrastructure Development Docs](https://github.com/WATonomous/wato_monorepo/tree/main/docs/dev/)
3. [ROS Structure Docs](src/samples/README.md)