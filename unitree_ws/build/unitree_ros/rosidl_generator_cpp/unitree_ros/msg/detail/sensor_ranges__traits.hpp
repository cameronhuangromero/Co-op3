// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__TRAITS_HPP_
#define UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "unitree_ros/msg/detail/sensor_ranges__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace unitree_ros
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorRanges & msg,
  std::ostream & out)
{
  out << "{";
  // member: front
  {
    out << "front: ";
    rosidl_generator_traits::value_to_yaml(msg.front, out);
    out << ", ";
  }

  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorRanges & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: front
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front: ";
    rosidl_generator_traits::value_to_yaml(msg.front, out);
    out << "\n";
  }

  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorRanges & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace unitree_ros

namespace rosidl_generator_traits
{

[[deprecated("use unitree_ros::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const unitree_ros::msg::SensorRanges & msg,
  std::ostream & out, size_t indentation = 0)
{
  unitree_ros::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use unitree_ros::msg::to_yaml() instead")]]
inline std::string to_yaml(const unitree_ros::msg::SensorRanges & msg)
{
  return unitree_ros::msg::to_yaml(msg);
}

template<>
inline const char * data_type<unitree_ros::msg::SensorRanges>()
{
  return "unitree_ros::msg::SensorRanges";
}

template<>
inline const char * name<unitree_ros::msg::SensorRanges>()
{
  return "unitree_ros/msg/SensorRanges";
}

template<>
struct has_fixed_size<unitree_ros::msg::SensorRanges>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unitree_ros::msg::SensorRanges>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unitree_ros::msg::SensorRanges>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__TRAITS_HPP_
