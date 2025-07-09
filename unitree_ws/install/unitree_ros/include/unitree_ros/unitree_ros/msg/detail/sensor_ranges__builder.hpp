// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__BUILDER_HPP_
#define UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unitree_ros/msg/detail/sensor_ranges__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unitree_ros
{

namespace msg
{

namespace builder
{

class Init_SensorRanges_right
{
public:
  explicit Init_SensorRanges_right(::unitree_ros::msg::SensorRanges & msg)
  : msg_(msg)
  {}
  ::unitree_ros::msg::SensorRanges right(::unitree_ros::msg::SensorRanges::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_ros::msg::SensorRanges msg_;
};

class Init_SensorRanges_left
{
public:
  explicit Init_SensorRanges_left(::unitree_ros::msg::SensorRanges & msg)
  : msg_(msg)
  {}
  Init_SensorRanges_right left(::unitree_ros::msg::SensorRanges::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_SensorRanges_right(msg_);
  }

private:
  ::unitree_ros::msg::SensorRanges msg_;
};

class Init_SensorRanges_front
{
public:
  Init_SensorRanges_front()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorRanges_left front(::unitree_ros::msg::SensorRanges::_front_type arg)
  {
    msg_.front = std::move(arg);
    return Init_SensorRanges_left(msg_);
  }

private:
  ::unitree_ros::msg::SensorRanges msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_ros::msg::SensorRanges>()
{
  return unitree_ros::msg::builder::Init_SensorRanges_front();
}

}  // namespace unitree_ros

#endif  // UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__BUILDER_HPP_
