#pragma once

#include "rclcpp/rclcpp.hpp"
#include "torque_interface/msg/joint_command.hpp"
#include "torque_interface/msg/joint_feedback.hpp"

#include <memory>

class TorqueBridge : public rclcpp::Node
{
public:
    TorqueBridge();

private:
    void single_command_callback(const torque_interface::msg::JointCommand::SharedPtr msg);
    rclcpp::Subscription<torque_interface::msg::JointCommand>::SharedPtr single_sub_cmd_;
    rclcpp::Subscription<torque_interface::msg::JointCommand>::SharedPtr leg_sub_cmd_;
    rclcpp::Subscription<torque_interface::msg::JointCommand>::SharedPtr all_sub_cmd_;
    rclcpp::Publisher<torque_interface::msg::JointFeedback>::SharedPtr pub_feedback_;

    void publish_feedback(int joint_index, float actual_tau);

    // Hook to lower-level controller
    void send_to_low_level(int joint_index, float tau);
    float get_feedback_from_low_level(int joint_index);
};
