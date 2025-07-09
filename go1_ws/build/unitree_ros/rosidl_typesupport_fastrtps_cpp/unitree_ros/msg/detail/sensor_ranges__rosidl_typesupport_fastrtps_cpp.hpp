// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_ros/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "unitree_ros/msg/detail/sensor_ranges__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace unitree_ros
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_ros
cdr_serialize(
  const unitree_ros::msg::SensorRanges & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_ros
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_ros::msg::SensorRanges & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_ros
get_serialized_size(
  const unitree_ros::msg::SensorRanges & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_ros
max_serialized_size_SensorRanges(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace unitree_ros

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_ros
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_ros, msg, SensorRanges)();

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
