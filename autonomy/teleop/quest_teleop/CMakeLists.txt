cmake_minimum_required(VERSION 3.8)
project(quest_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuestHandPose.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(quest_teleop_node
  src/quest_teleop_node.cpp
  src/wss_server.cpp
  src/quest_message_parser.cpp
)

target_include_directories(quest_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(quest_teleop_node
  rclcpp
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(quest_teleop_node
  "${cpp_typesupport_target}"
  Boost::system
  OpenSSL::SSL
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
)

install(TARGETS quest_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts static
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()