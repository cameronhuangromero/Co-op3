#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <vector>
#include <algorithm>

/**
ROS2 node implementing a prototype joint space impedance controller

This node above the torque bridge in the control hierarchy.

This node subscribes to:
    - "filtered_joint_states" for current joint positions, velocities, and estimated torques coming from the low level controller via the buffer
    - "desired_joint_states" for target joint positions and velocities, provided by a higher controller

This controller computes the desired joint torques using the joint-space impedance control law (pulled from unitree_legged_sdk/example_torque.cpp)

T = stiffness * (desired_position - measured_position) + damping * (desired_velocity - measured_velocity)

The resulting torques are published to:
    - "desired_torques", which is then seen by the TorqueCommandNode to update the buffer for the low level loop
*/

ImpedanceController::ImpedanceController()
: Node("impedance_controller")
{
    // declare parameters for gain and torque limit
    this->declare_parameter<std::vector<double>>("stiffness", std::vector<double>(k_number_joints, 40.0));
    this->declare_parameter<std::vector<double>>("damping", std::vector<double>(k_number_joints, 2.0));
    this->declare_parameter<std::vector<double>>("torque_limits", std::vector<double>(k_number_joints, 10.0));
    this->declare_parameter<double>("control_rate_hz", 100.0);

    // load parameters
    stiffness_ = this->get_parameter("stiffness").as_double_array();
    damping_ = this->get_parameter("damping").as_double_array();
    torque_limits_ = this->get_parameter("torque_limits").as_double_array();
    double rate = this->get_parameter("control_rate_hz").as_double();

    // verify correct size
    ensure_vector_size(stiffness_, 40.0);
    ensure_vector_size(damping_, 2.0);
    ensure_vector_size(torque_limits_, 10.0);

    // joint vectors
    measured_position_.assign(k_number_joints, 0.0);
    measured_velocity_.assign(k_number_joints, 0.0);
    desired_position_.assign(k_number_joints, 0.0);
    desired_velocity_.assign(k_number_joints, 0.0);

    // subscriptions
    sub_desired_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("desired_positions", 10, std::bind(&ImpedanceController::desiredPositionsCallback, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("filtered_positions", 10, std::bind(&ImpedanceController::jointStateCallback, this, std::placeholders::_1));

    // publish desired torque
    pub_torques_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("desired_torques", 10);


    // timer for control loop
    auto period = std::chrono::duration<double>(1.0/rate);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&ImpedanceController:controlLoop, this));


    // logs initialization
    RCLCPP_INFO(this->get_logger(), "ImpedanceController initialized at %.1f Hz", rate);
}

// Callbacks

// called every time a new messaged is published to DesiredPositions topic
void ImpedanceController::desiredPositionsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{  
    // locks mutex for the duration of the function, which prevents race conditions of the control loop is reading desired_position_ at the same time
    std::lock_guard<std::mutex> lock(data_mutex_);

    // checks if the message has the correct number of joint positions, and prints warning/error if not
    if ((int)msg->data.size() != k_number_joints){
        RCLCPP_WARN(this->get_logger(), "Expected %d joint commands but got %zu", k_number_joints, msg->data.size());
    }

    // loops over each joint and stores the desired position listed int he message into the node's internal desired_position_ vector
    for (int i = 0; i < k_number_joints; ++i)
        desired_position_[i] = static_cast<double>(msg->data[i]);
}

void ImpedanceController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if ((int)msg->position.size() == k_number_joints){
        for (int i = 0; i < k_number_joints; ++i)
            measured_position_[i] = msg->position[i];
    }

    if ((int) msg->velocity.size() == k_number_joints) {
        for (int i = 0; i < k_number_joints; ++i)
            measured_velocity_[i] = msg->velocity[i];
    }
}


// control loop
void ImpedanceController::controlLoop()
{
    // create local copies to avoid locking mutex for the whole control calculation
    std::vector<double> local_measured_position, local_measured_velocity;
    std::vector<double> local_desired_position, local_desired_velocity;
    std::vector<double> local_stiffness, local_damping, local_torque_limits;


    // copy under the mutex lock, which ensures thread-safe access to shared variables from callbacks
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_measured_position = measured_position_;
        local_measured_velocity = measured_velocity_;
        local_desired_position = desired_position_;
        local_desired_velocity = desired_velocity_;
        local_stiffness = stiffness_;
        local_damping = damping_;
        local_torque_limits = torque_limits_;
    }

    // prepare torque message
    std_msgs::msg::Float32MultiArray torque_msg;
    torque_msg.data.resize(k_number_joints);

    // loop over joints
    for (int = 0; i < k_number_joints; ++i)
    {
        double position_error = local_desired_position[i] - local_measured_position[i];
        double velocity_error = local_desired_velocity[i] - local_measured_velocity[i];

        // apply impedance control law

        double torque = local_stiffness[i] * position_error + local_damping[i] * velocity_error;

        // apply torque limits, which changes the torque to respect hardware limits and ensures that the robot doesn't try an unsafe torque
        if (torque > local_torque_limits[i])
            torque = local_torque_limits[i];
        else if (torque < -local_torque_limits[i])
            torque = -local_torque_limits[i];

        torque_msg.data[i] = static_cast<float>(torque);
    }

    pub_torques_->publish(torque_msg);
}