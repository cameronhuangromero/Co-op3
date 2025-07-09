#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <unitree_ros/unitree_driver.hpp>

class UnitreeDriverNode : public rclcpp::Node {
public:
    UnitreeDriverNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Unitree driver
    UnitreeDriver driver_;
};

