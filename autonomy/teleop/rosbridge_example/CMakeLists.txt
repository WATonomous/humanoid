cmake_minimum_required(VERSION 3.8)
project(rosbridge_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sample_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(rosbridge_listener_lib src/rosbridge_listener_core.cpp)
add_library(rosbridge_publisher_lib src/rosbridge_publisher_core.cpp)

target_include_directories(rosbridge_listener_lib PUBLIC include)

add_executable(rosbridge_listener_node src/rosbridge_listener_node.cpp)
add_executable(rosbridge_publisher_node src/rosbridge_publisher_node.cpp)

ament_target_dependencies(rosbridge_listener_lib rclcpp std_msgs sample_msgs)
ament_target_dependencies(rosbridge_publisher_lib rclcpp std_msgs sample_msgs)

target_link_libraries(rosbridge_listener_node rosbridge_listener_lib)
target_link_libraries(rosbridge_publisher_node rosbridge_publisher_lib)

target_include_directories(rosbridge_listener_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(rosbridge_publisher_lib PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(rosbridge_listener_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(rosbridge_publisher_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS rosbridge_listener_node
  DESTINATION lib/${PROJECT_NAME})
  install(TARGETS rosbridge_publisher_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
