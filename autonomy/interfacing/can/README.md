## CAN Interfacing Package Documentation

The documentation for the CAN interfacing package follows a standard WATonomous package level scheme. As shown below:
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

TODO: Move hardware_mapping to behaviour
TODO: Define can messages for hand - ideally reuse 

#### Inputs & Outputs
- DBC defining all of the can bus messages
- config/params.yaml file defining can bus parameters

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

- **CAN Format - IMPORTANT**
  - Can messages are defined in the dbc
  - All messages are extended - meaning that their can_id starts with a 8 to signify a larger id
  - DBC file can be edited using SavyCAN or VectorCAN data base editors
  - CAN ID 6 byte can message id - 2 byte device id `MMMMMMDD#DATADATA`

#### Usage

**Building the Package**
Build the package using the standard ROS 2 build process within the interfacing container.

**Running the CAN Node**
1. Start the interfacing container
2. Access the container: `sudo ./watod -t interfacing`
3. Source the environment: `source ./watod_ros_entrypoint.sh`
4. `./watod up`

**Note**: The launch file includes a test controller node for development/testing. To disable it for production use, comment out the test controller section in `launch/can.launch.py` (lines marked with "TEST CONTROLLER NODE" comments).

#### Configuration
- Relevant parameters, environment variables, or dependencies

#### Config 
- Configuration files, environment variables, or command-line arguments
  - `config/params.yaml`: Contains parameters for CAN interface setup, such as:
    - `can_interface`: Name of the CAN interface (e.g., `can0`)
    - `baud_rate`: CAN bus speed (e.g., `500000`)
    - `frame_timeout`: Timeout for receiving frames in milliseconds
