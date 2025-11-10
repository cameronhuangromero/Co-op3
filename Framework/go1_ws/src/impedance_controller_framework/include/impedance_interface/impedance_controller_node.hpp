#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "impedance_interface/impedance_controller_base.hpp"

/*
 * Author: Cameron Romero
 *
 * ROS2 Node that subscribes to the filtered joint states and publishes
 * desired torques computed by an impedance controller.
 */

class ImpedanceControllerNode : public rclcpp::Node {
public:
  explicit ImpedanceControllerNode(
      std::shared_ptr<ImpedanceControllerBase> controller);

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<ImpedanceControllerBase> controller_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub_;
};
