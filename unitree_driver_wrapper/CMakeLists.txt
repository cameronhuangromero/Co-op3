cmake_minimum_required(VERSION 3.8)
project(unitree_driver_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(unitree_ros REQUIRED)

include_directories(
  include
  ../unitree_ros/src  # or wherever UnitreeDriver is defined
)

include_directories(
  include
  ../unitree_ros/include
  ../unitree_ros/utils/faceLightSDK_Nano/include
)

include_directories(
  include
  ../unitree_ros/include
  ../unitree_ros/utils/faceLightSDK_Nano/include
  ../unitree_ros/utils/include      # ✅ Add this — it contains unitree_legged_sdk/
)


include_directories(include)

include_directories(
  include
  ../unitree_ros/include  # Only works if both packages are in the same workspace
)

add_executable(unitree_driver_node src/unitree_driver_node.cpp)
ament_target_dependencies(unitree_driver_node rclcpp geometry_msgs std_srvs sensor_msgs)


target_link_libraries(unitree_driver_node unitree_driver_library)  # if needed
install(TARGETS unitree_driver_node DESTINATION lib/${PROJECT_NAME})

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











