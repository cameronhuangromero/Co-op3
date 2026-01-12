cmake_minimum_required(VERSION 3.5)
project(joint_state_buffer_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)



include_directories(
  ../../../unitree_legged_sdk/include
)

add_executable(joint_state_buffer_node src/ros2_buffer_node.cpp)
ament_target_dependencies(joint_state_buffer_node rclcpp sensor_msgs)

add_executable(torque_command_node src/ros2_command_node.cpp)
ament_target_dependencies(torque_command_node rclcpp std_msgs)

install(TARGETS
  joint_state_buffer_node
  torque_command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()