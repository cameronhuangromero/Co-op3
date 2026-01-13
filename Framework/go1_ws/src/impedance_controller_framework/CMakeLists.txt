cmake_minimum_required(VERSION 3.5)
project(impedance_controller_framework)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(impedance_controller_lib
  src/impedance_controller_factory.cpp
  src/impedance_controller_manager.cpp
  test/test_dummy.cpp
)

target_include_directories(impedance_controller_lib PUBLIC include)
target_link_libraries(impedance_controller_lib Eigen3::Eigen)
ament_target_dependencies(impedance_controller_lib rclcpp std_msgs)

add_executable(test_dummy test/test_dummy.cpp)
target_link_libraries(test_dummy impedance_controller_lib Eigen3::Eigen)
target_include_directories(test_dummy PRIVATE include)
ament_target_dependencies(test_dummy rclcpp std_msgs)

ament_add_gtest(test_impedance_controller test/test_impedance_controller.cpp)
if(TARGET test_impedance_controller)
  target_link_libraries(test_impedance_controller impedance_controller_lib Eigen3::Eigen)
  target_include_directories(test_impedance_controller PRIVATE include)
endif()

install(TARGETS
  impedance_controller_lib
  test_dummy
  test_impedance_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
