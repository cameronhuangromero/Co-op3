cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

include_directories(include)
link_directories(lib/cpp/${ARCH})



include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/include)



option(PYTHON_BUILD "build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)
endif()

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  )
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# one pc one process
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})

add_executable(example_position_hardware example/example_position_hardware.cpp)
target_link_libraries(example_position_hardware ${EXTRA_LIBS})

add_executable(example_position_databack example/example_position_databack.cpp)
target_link_libraries(example_position_databack ${EXTRA_LIBS})

add_executable(example_position_databackMATLAB example/example_position_databackMATLAB.cpp)
target_link_libraries(example_position_databackMATLAB ${EXTRA_LIBS})

# add_executable(jacobian_calculation example/jacobian_calculation.cpp)
# target_link_libraries(jacobian_calculation ${EXTRA_LIBS})

add_executable(example_jacobian example/example_jacobian.cpp     common/unitreeLeg.cpp)
target_link_libraries(example_jacobian ${EXTRA_LIBS})

# add_executable(unitreeLeg example/common/unitreeLeg.cpp)
# target_link_libraries(unitreeLeg ${EXTRA_LIBS})


add_executable(example_torque_databack example/example_torque_databack.cpp)
target_link_libraries(example_torque_databack ${EXTRA_LIBS})

add_executable(example_torque_databackMATLAB example/example_torque_databackMATLAB.cpp)
target_link_libraries(example_torque_databackMATLAB ${EXTRA_LIBS})

add_executable(example_12_joints example/example_12_joints.cpp)
target_link_libraries(example_12_joints ${EXTRA_LIBS})

add_executable(example_12_joints_databackMATLAB example/example_12_joints_databackMATLAB.cpp)
target_link_libraries(example_12_joints_databackMATLAB ${EXTRA_LIBS})




# install
install(TARGETS
  example_position example_velocity example_torque example_walk example_joystick
  DESTINATION bin/unitree)
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/unitree
  USE_SOURCE_PERMISSIONS)
