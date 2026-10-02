cmake_minimum_required(VERSION 3.10)
project(joint_command)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(joint_command_node
  src/joint_command_node.cpp
  src/joint_command_core.cpp)
ament_target_dependencies(joint_command_node rclcpp common_msgs ament_index_cpp)
target_include_directories(joint_command_node PUBLIC include)
target_link_libraries(joint_command_node yaml-cpp)

install(TARGETS joint_command_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Runs against the SHIPPED config (CONFIG_DIR): an unsafe config fails the build, not the arm.
  ament_add_gtest(test_joint_command_core
    test/test_joint_command_core.cpp
    src/joint_command_core.cpp)
  ament_target_dependencies(test_joint_command_core rclcpp common_msgs)
  target_include_directories(test_joint_command_core PRIVATE include)
  target_link_libraries(test_joint_command_core yaml-cpp)
  target_compile_definitions(test_joint_command_core
    PRIVATE CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_package()
