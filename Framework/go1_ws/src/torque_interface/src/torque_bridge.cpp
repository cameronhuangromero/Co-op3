#include "torque_interface/torque_bridge.hpp"
# include "unitree_legged_sdk/example/control_joint_torque.cpp"

TorqueBridge::TorqueBridge() : Node("torque_bridge")
{
    single_sub_cmd_ = this->create_subscription<torque_interface::msg::JointCommand>(
        "/joint_command", 10,
        std::bind(&TorqueBridge::single_command_callback, this, std::placeholders::_1)
    );

    pub_feedback_ = this->create_publisher<torque_interface::msg::JointFeedback>(
        "/joint_feedback", 10
    );
}

void TorqueBridge::single_command_callback(const torque_interface::msg::JointCommand::SharedPtr msg)
{
    send_to_low_level(msg->joint_index, msg->torque);

    float tau_feedback = get_feedback_from_low_level(msg->joint_index);
    publish_feedback(msg->joint_index, tau_feedback);
}

void TorqueBridge::send_to_low_level(int joint_index, float tau)
{
    Custom torque(LOWLEVEL)
    torque.singlejointtorque(joint_index, tau);

}

float TorqueBridge::get_feedback_from_low_level(int joint_index)
{
    // TODO: return actual measured torque from Custom
    // PURPOSE?
    return 0.0f;
}

void TorqueBridge::publish_feedback(int joint_index, float actual_tau)
{
    auto msg = torque_interface::msg::JointFeedback();
    msg.joint_index = joint_index;
    msg.actual_torque = actual_tau;
    pub_feedback_->publish(msg);
}
