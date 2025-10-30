#include "impedance_controller_base.hpp"
#include "prototype_impedance_controller.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

/*
 * Author: Cameron Romero
 *
 * This node subscribes to filtered joint states and published desired joint
 * torques.
 *
 * This node acts as an interface between ROS2 and any impedance
 * controllers derived from the ImpedanceControllerBase class.
 *
 * The node converts incoming joint position and velocity data into arrays, and
 * then passes them it to the active impedance controller, which publishes the
 * resulting torque commands.
 */

class ImpedanceControllerNode : public rclcpp::Node {
public:
  ImpedanceControllerNode(std::shared_ptr<ImpedanceControllerBase> controller)
      : Node("impedance_controller_node"), controller_(controller) {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "filtered_joint_states", 10,
        std::bind(&ImpedanceControllerNode::jointStateCallback, this,
                  std::placeholders::_1));

    torque_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "desired_torques", 10);
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < ImpedanceControllerBase::kNumJoints ||
        msg->velocity.size() < ImpedanceControllerBase::kNumJoints) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint state message size!");
      return;
    }

    std::array<float, ImpedanceControllerBase::kNumJoints> q{};
    std::array<float, ImpedanceControllerBase::kNumJoints> dq{};

    for (int i = 0; i < ImpedanceControllerBase::kNumJoints; ++i) {
      q[i] = msg->position[i];
      dq[i] = msg->velocity[i];
    }

    auto tau = controller_->computeTorques(q, dq);

    std_msgs::msg::Float32MultiArray torque_msg;
    torque_msg.data.assign(tau.begin(), tau.end());
    torque_pub_->publish(torque_msg);
  }

  std::shared_ptr<ImpedanceControllerBase> controller_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<PrototypeImpedanceController>();
  auto node = std::make_shared<ImpedanceControllerNode>(controller);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
