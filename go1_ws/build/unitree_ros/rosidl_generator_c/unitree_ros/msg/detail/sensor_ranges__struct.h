// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_H_
#define UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SensorRanges in the package unitree_ros.
typedef struct unitree_ros__msg__SensorRanges
{
  float front;
  float left;
  float right;
} unitree_ros__msg__SensorRanges;

// Struct for a sequence of unitree_ros__msg__SensorRanges.
typedef struct unitree_ros__msg__SensorRanges__Sequence
{
  unitree_ros__msg__SensorRanges * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_ros__msg__SensorRanges__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_H_
