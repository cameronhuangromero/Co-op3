#include "unitree_driver_wrapper/unitree_driver_node.hpp"

UnitreeDriverNode::UnitreeDriverNode()
    : Node("unitree_driver_node"),
      driver_("192.168.123.161", 8082)  // or read from parameters
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&UnitreeDriverNode::cmdVelCallback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    stand_up_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/stand_up",
        [this](const auto req, auto res) {
            driver_.stand_up();
            res->success = true;
            res->message = "Robot is standing up.";
        });

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/stop",
        [this](const auto req, auto res) {
            driver_.stop();
            res->success = true;
            res->message = "Robot has stopped.";
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&UnitreeDriverNode::timerCallback, this));
}

void UnitreeDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    driver_.walk_w_vel(msg->linear.x, msg->linear.y, msg->angular.z);
}

void UnitreeDriverNode::timerCallback() {
    UNITREE_LEGGED_SDK::IMU imu = driver_.get_imu();
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.linear_acceleration.x = imu.accelerometer[0];
    msg.linear_acceleration.y = imu.accelerometer[1];
    msg.linear_acceleration.z = imu.accelerometer[2];
    msg.angular_velocity.x = imu.gyroscope[0];
    msg.angular_velocity.y = imu.gyroscope[1];
    msg.angular_velocity.z = imu.gyroscope[2];
    msg.orientation.x = imu.quaternion[1];
    msg.orientation.y = imu.quaternion[2];
    msg.orientation.z = imu.quaternion[3];
    msg.orientation.w = imu.quaternion[0];

    imu_pub_->publish(msg);
}

