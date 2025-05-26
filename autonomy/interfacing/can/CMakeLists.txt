cmake_minimum_required(VERSION 3.10)
project(can)

# Set compiler to C++ 17 Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED) # ROS2 C++ package
find_package(std_msgs REQUIRED) # ROS2 message package

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(can_lib
  src/can_core.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(can_lib
  PUBLIC
  include
)

# Add ROS2 dependencies required by package 
ament_target_dependencies(can_lib
  rclcpp
  std_msgs
)

# Link system libraries for CAN support
# Note: Using system CAN libraries instead of libsocketcan for broader compatibility
target_link_libraries(can_lib)

# Create the can_node executable
add_executable(can_node src/can_node.cpp src/can_core.cpp)
ament_target_dependencies(can_node rclcpp std_msgs)
# Link to the previously built libraries 
target_link_libraries(can_node can_lib)

# Copy executable to installation location
install(TARGETS
  can_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add tests
  # ament_add_gtest(can_test test/can_test.cpp)
  # target_include_directories(can_test PUBLIC include)
  # target_link_libraries(can_test)
endif()

ament_package()
