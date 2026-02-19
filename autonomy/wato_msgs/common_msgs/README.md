# Common Messages (common_msgs)

## File Tree/Package Structure
This message package follows standard ROS2 package scheme:
```
common_msgs/
├── CMakeLists.txt
├── package.xml
├── README.md
└── msg/
    ├── ArmPose.msg
    ├── HandPose.msg
    └── JointState.msg
```

## Purpose
The `common_msgs` package serves as a global package for message definitions that are commonly used across various components and packages within the humanoid project. 

## Inputs & Outputs
This package primarily defines message structures. Therefore:
-   **Outputs**: Provides `.msg` definitions that other ROS 2 packages can import and use.
-   **Inputs**: Does not process data or subscribe to topics; it only defines data structures.

## Key Features
-   **Message Definitions**: Contains a collection of `.msg` files, each defining a specific data structure. These messages are designed to be generic and applicable in multiple contexts.

### Message Definitions
-   `ArmPose.msg`: Describes the pose of an arm, including its major joints and optionally the hand.
-   `HandPose.msg`: Details the pose of a hand, including individual finger joint states. 
-   `JointState.msg`: A custom message (similar message structure to [sensor_msgs/JointState](https://docs.ros.org/en/humble/p/sensor_msgs/msg/JointState.html)) to represent a single joint.

## Usage
To use the messages defined in this package within another ROS 2 package:

1.  **Copy the package into your Docker image's source workspace**: Add the following line to your Dockerfile (typically `root/docker/MODULE_NAME/MODULE_NAME.Dockerfile`), typically in the section where you copy your source code:
    ```dockerfile
    # # Copy in source code
    # # COPY autonomy/wato_msgs/sample_msgs sample_msgs
    COPY autonomy/wato_msgs/common_msgs common_msgs
    ```

2.  **Add Dependency**: Ensure that `common_msgs` is listed as a dependency in the `package.xml` of your consuming package:
    ```xml
    <depend>common_msgs</depend>
    ```
3.  **Include in CMakeLists.txt** (for C++ packages):
    ```cmake
    find_package(common_msgs REQUIRED)
    ament_target_dependencies(your_target_name common_msgs)
    ```
4.  **Import in Python scripts**:
    ```python
    from common_msgs.msg import YourMessageName
    ```
5.  **Include in C++ code**:
    ```cpp
    #include "common_msgs/msg/your_message_name.hpp"
    ```

### Testing
Currently, this package primarily contains message definitions, so testing focuses on ensuring they can be correctly compiled and used by dependent packages. 

## Configuration
-   **Parameters**: No configurable parameters are defined within this package itself.
