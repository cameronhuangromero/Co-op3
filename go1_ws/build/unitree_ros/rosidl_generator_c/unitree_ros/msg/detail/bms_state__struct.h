// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_ros:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__BMS_STATE__STRUCT_H_
#define UNITREE_ROS__MSG__DETAIL__BMS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BmsState in the package unitree_ros.
/**
  * This message is a fork from https://github.com/unitreerobotics/unitree_ros_to_real/tree/master/unitree_legged_msgs
 */
typedef struct unitree_ros__msg__BmsState
{
  uint8_t version_h;
  uint8_t version_l;
  uint8_t bms_status;
  /// SOC 0-100%
  uint8_t soc;
  /// mA
  int32_t current;
  uint16_t cycle;
  /// x1 degrees centigrade
  int8_t bq_ntc[2];
  /// x1 degrees centigrade
  int8_t mcu_ntc[2];
  /// cell voltage mV
  uint16_t cell_vol[10];
} unitree_ros__msg__BmsState;

// Struct for a sequence of unitree_ros__msg__BmsState.
typedef struct unitree_ros__msg__BmsState__Sequence
{
  unitree_ros__msg__BmsState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_ros__msg__BmsState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_ROS__MSG__DETAIL__BMS_STATE__STRUCT_H_
