cmake_minimum_required(VERSION 3.5)
project(impedance_controller_framework)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# headers
include_directories(
  include
  ../../../../unitree_legged_sdk/include
)

# build lib
add_library(impedance_controllers
  src/impedance_controller_node.cpp
)

ament_target_dependencies(impedance_controllers
  rclcpp
  sensor_msgs
  std_msgs
)

# main executable
add_executable(impedance_controller_main src/main.cpp)

target_link_libraries(impedance_controller_main impedance_controllers)

ament_target_dependencies(impedance_controller_main
  rclcpp
  sensor_msgs
  std_msgs
)

# installation
install(TARGETS
  impedance_controller_main
  impedance_controllers
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
