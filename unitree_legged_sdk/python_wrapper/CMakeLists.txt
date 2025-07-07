cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk_python_wrapper)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    set(ARCH arm64)
endif()

# Set output directory for built .so Python module
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../lib/python/${ARCH})

# Include and link paths
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/cpp/${ARCH})

# Add pybind11
add_subdirectory(third-party/pybind11)

# Python binding source
set(SRC python_interface.cpp)

# Define pybind11 module
pybind11_add_module(robot_interface ${SRC})

# Use position-independent code
set_target_properties(robot_interface PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link the static Unitree SDK
target_link_libraries(robot_interface PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/cpp/${ARCH}/libunitree_legged_sdk.a
    pthread rt m
)
