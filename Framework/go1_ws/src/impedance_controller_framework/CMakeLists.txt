cmake_minimum_required(VERSION 3.8)
project(impedance_controller_framework)

# Compiler Settings
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Core Library (not ROS2)
add_library(impedance_core
  src/common/controller_manager.cpp
  src/impedance_interface/prototype_impedance_controller.cpp
)

target_include_directories(impedance_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS2 Node Exectutable
add_executable(impedance_controller_node
  src/main.cpp
  src/impedance_controller_node.cpp
  src/torque_bridge.cpp
  src/ros2_buffer_node.cpp
  src/ros2_command_node.cpp
)

target_link_libraries(impedance_controller_node
  impedance_core
)

ament_target_dependencies(impedance_controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Non-ROS2 Test
add_executable(test_step1
  src/tools/test_step1.cpp
)

target_link_libraries(test_step1
  impedance_core
)

# Install targets
install(
  DIRECTORY include/impedance_controller_framework/
  DESTINATION include/${PROJECT_NAME} 
)

install(
  TARGETS
    impedance_core
    impedance_controller_node
    test_step1
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Package export
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
