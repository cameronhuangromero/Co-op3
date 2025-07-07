cmake_minimum_required(VERSION 3.8)
project(robotic_control)


# # Use C++14 or C++17 depending on your needs
# set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



# Include unitree_legged_sdk headers manually
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include
)
# include_directories(/opt/ros/humble/include/std_msgs)

# Link static SDK library
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib)



add_executable(go1_jacobian_node src/example_jacobian_sim.cpp src/unitreeLeg.cpp)




target_link_libraries(go1_jacobian_node
  pthread
  ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a
)
ament_target_dependencies(go1_jacobian_node rclcpp std_msgs sensor_msgs)






install(TARGETS go1_jacobian_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
