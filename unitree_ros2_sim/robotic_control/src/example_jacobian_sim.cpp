#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "common/unitreeLeg.h"  // Assuming you still use your leg kinematics

#include "unitree_legged_sdk/comm.h"


using namespace UNITREE_LEGGED_SDK;

class Go1Ros2ControllerNode : public rclcpp::Node
{
public:
  Go1Ros2ControllerNode()
  : Node("go1_jacobian_node"),
    legFR(0, 0.08, 0.213, 0.213, Vec3(0.188, -0.047, 0))
  {
    RCLCPP_INFO(this->get_logger(), "Go1 Jacobian Node Started");

    // Publishers for joint commands
    fr_hip_pub = this->create_publisher<std_msgs::msg::Float64>("/FR_hip_controller/command", 10);
    fr_thigh_pub = this->create_publisher<std_msgs::msg::Float64>("/FR_thigh_controller/command", 10);
    fr_calf_pub = this->create_publisher<std_msgs::msg::Float64>("/FR_calf_controller/command", 10);

    // (Optional) subscriber to joint states
    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Go1Ros2ControllerNode::jointStateCallback, this, std::placeholders::_1));

    // Timer for periodic control
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&Go1Ros2ControllerNode::controlLoop, this));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Optional: log or track joint states
    // Example: RCLCPP_INFO(this->get_logger(), "Hip pos: %f", msg->position[FR_0]);
  }

  void controlLoop()
  {
    motiontime++;

    Vec3 desFootFR(0.2, -0.1, -0.25); // Desired foot position
    Vec3 qFR = legFR.calcQ(desFootFR, FrameType::BODY); // Inverse kinematics

    // Send joint commands
    std_msgs::msg::Float64 cmd_msg;

    cmd_msg.data = qFR(0);
    fr_hip_pub->publish(cmd_msg);

    cmd_msg.data = qFR(1);
    fr_thigh_pub->publish(cmd_msg);

    cmd_msg.data = qFR(2);
    fr_calf_pub->publish(cmd_msg);
  }

  // ROS 2 publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_hip_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_thigh_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_calf_pub;

  // ROS 2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

  // Control loop
  rclcpp::TimerBase::SharedPtr control_timer;
  int motiontime = 0;

  // Kinematics model for FR leg
  QuadrupedLeg legFR;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1Ros2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
