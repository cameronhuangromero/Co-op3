// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_ros:msg/SensorRanges.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_HPP_
#define UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__unitree_ros__msg__SensorRanges __attribute__((deprecated))
#else
# define DEPRECATED__unitree_ros__msg__SensorRanges __declspec(deprecated)
#endif

namespace unitree_ros
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorRanges_
{
  using Type = SensorRanges_<ContainerAllocator>;

  explicit SensorRanges_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->front = 0.0f;
      this->left = 0.0f;
      this->right = 0.0f;
    }
  }

  explicit SensorRanges_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->front = 0.0f;
      this->left = 0.0f;
      this->right = 0.0f;
    }
  }

  // field types and members
  using _front_type =
    float;
  _front_type front;
  using _left_type =
    float;
  _left_type left;
  using _right_type =
    float;
  _right_type right;

  // setters for named parameter idiom
  Type & set__front(
    const float & _arg)
  {
    this->front = _arg;
    return *this;
  }
  Type & set__left(
    const float & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const float & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_ros::msg::SensorRanges_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_ros::msg::SensorRanges_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_ros::msg::SensorRanges_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_ros::msg::SensorRanges_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_ros__msg__SensorRanges
    std::shared_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_ros__msg__SensorRanges
    std::shared_ptr<unitree_ros::msg::SensorRanges_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorRanges_ & other) const
  {
    if (this->front != other.front) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorRanges_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorRanges_

// alias to use template instance with default allocator
using SensorRanges =
  unitree_ros::msg::SensorRanges_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_ros

#endif  // UNITREE_ROS__MSG__DETAIL__SENSOR_RANGES__STRUCT_HPP_
