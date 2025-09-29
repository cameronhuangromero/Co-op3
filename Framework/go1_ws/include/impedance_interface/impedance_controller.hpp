#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <vector>

/**
Interface for the Impedance Controller

impedance_controller.cpp has more notes on implementation/specifics
*/

class ImpedanceController : public rclcpp::Node{
    public:
        static constexpr int k_number_joints = 12;
        ImpedanceController();

    private:
        void desiredPositionsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void controlLoop();
        void ensureVectorSize(std::vector<double> &value, double default_value);
        double gravityCompensation(int joint_index, const std::vector<double> &measured_position, const std::vector<double> &measured_velocity);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_desired_;
        rclcpp::Subscription<std_msgs::msg::JointState>::SharedPtr sub_join_state_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torques_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::mutex data_mutex_;
        std::vector<double> measured_position_, measured_velocity_, desired_position_, desired_velocity_;
        std::vector<double> stiffness_, damping_, torque_limits_;
};