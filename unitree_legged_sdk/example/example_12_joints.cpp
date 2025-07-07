/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.005; // 0.001~0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);
  for (int i = 0; i < 12; ++i) {
    std::cout << "Joint" << i << " pos: " << state.motorState[i].q << std::endl;
  }
  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if (motiontime >= 500)
  {
    // float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
    // if (torque > 5.0f)
    //   torque = 5.0f;
    // if (torque < -5.0f)
    //   torque = -5.0f;

    float Kp = 10.0f;
    float Kd = 1.0f;
    // float maxTorque = 5.0f;

    // for (int i = 0; i < 12; ++i) {
    int i = FR_1;
        float q_des = 0.0f;
        float dq_des = 0.0f;

        float q_err = q_des - state.motorState[i].q;
        float dq_err = dq_des - state.motorState[i].dq;
        float torque = Kp * q_err + Kd * dq_err;

        if (torque > 0.1f)
            torque = 0.1f;
        if (torque < -0.1f)
            torque = -0.1f;

        cmd.motorCmd[i].q = PosStopF;
        cmd.motorCmd[i].dq = VelStopF;
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = torque;
    
  }
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    exit(-1);

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}