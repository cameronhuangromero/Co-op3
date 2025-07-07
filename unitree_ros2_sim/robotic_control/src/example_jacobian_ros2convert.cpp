#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "common/unitreeLeg.h"

#include "rclcpp/rclcpp.hpp"
#include <unistd.h> // for sleep
#include <iostream>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Go1JacobianNode : public rclcpp::Node
{
public:
  Go1JacobianNode()
      : Node("go1_jacobian_node"),
        safe(LeggedType::Go1),
        udp(LOWLEVEL, 8090, "192.168.123.10", 8007),
        legFR(0, 0.08, 0.213, 0.213, Vec3(0.188, -0.047, 0))
  {
    RCLCPP_INFO(this->get_logger(), "Go1 Controller Node Started");

    udp.InitCmdData(cmd);

    // Timers for periodic tasks
    timer_control = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&Go1JacobianNode::RobotControl, this));

    timer_udp_send = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&Go1JacobianNode::UDPSend, this));

    timer_udp_recv = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&Go1JacobianNode::UDPRecv, this));
  }

private:
  void UDPRecv()
  {
    udp.Recv();
  }

  void UDPSend()
  {
    udp.Send();
  }

  void RobotControl()
  {
    motiontime++;
    udp.GetRecv(state);

    Vec3 desFootFR(0.2, -0.1, -0.25);
    Vec3 qFR = legFR.calcQ(desFootFR, FrameType::BODY);

    float Kp = 20.0;
    float Kd = 2.0;

    for (int i = 0; i < 3; i++)
    {
      cmd.motorCmd[FR_0 + i].q = qFR(i);
      cmd.motorCmd[FR_0 + i].dq = 0;
      cmd.motorCmd[FR_0 + i].Kp = Kp;
      cmd.motorCmd[FR_0 + i].Kd = Kd;
      cmd.motorCmd[FR_0 + i].tau = 0;
    }

    safe.PositionLimit(cmd);
    int res = safe.PowerProtect(cmd, state, 1);
    if (res < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Power protect triggered. Exiting...");
      rclcpp::shutdown();
    }

    udp.SetSend(cmd);
  }

  // Unitree SDK members
  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  QuadrupedLeg legFR;

  // ROS 2 timers
  rclcpp::TimerBase::SharedPtr timer_control;
  rclcpp::TimerBase::SharedPtr timer_udp_send;
  rclcpp::TimerBase::SharedPtr timer_udp_recv;

  // Control loop variables
  double dt = 0.002; // 2 ms
  int motiontime = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1JacobianNode>());
  rclcpp::shutdown();
  return 0;
}

