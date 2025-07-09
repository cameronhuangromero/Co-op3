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
  std::string jointNames[12] = {
  "FR_0", "FR_1", "FR_2", "FL_0", "FL_1", "FL_2",
  "RR_0", "RR_1", "RR_2", "RL_0", "RL_1", "RL_2"
};

for (int i = 0; i < 12; ++i) {
  std::cout << jointNames[i]
            << " | pos: " << state.motorState[i].q
            << " | tau: " << state.motorState[i].tauEst << std::endl;
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
    // float Kd = 4.0f;
    // float maxTorque = 5.0f;
     struct JointConfig {
      int index;
      float maxTorque;
    };

     JointConfig joints[] = {
      {FR_0, 0.5f},
      {FR_1, 3.0f},
      {FR_2, 0.5f},
      {FL_0, 0.5f},
      {FL_1, 3.0f},
      {FL_2, 0.5f},
      {RR_0, 0.5f},
      {RR_1, 3.0f},
      {RR_2, 0.5f},
      {RL_0, 0.5f},
      {RL_1, 3.0f},
      {RL_2, 0.5f},
    };

    // for (int i = 0; i < 12; ++i) {
       for (const auto& joint : joints) {
      int i = joint.index;
        float q_des = 0.0f;
        float dq_des = 0.0f;

        float q_err = q_des - state.motorState[i].q;
        float dq_err = dq_des - state.motorState[i].dq;
        float Kd = (i % 3 == 2) ? 4.0f : 1.0f;
        float torque = Kp * q_err + Kd * dq_err;



        // FR_0=.5, FR_1=3, FR_2=1, FL_0=1, FL_1=3, FL_2=.25(just drops), RR_0=1, RR_1=3(vibrate), RR_2=.5(drops but does nothing at .25)
        // RL_0=1, RL_1=3(vibrate but at 2 doesnt move), RL_2=.25(same as RR_2)  
         if (torque > joint.maxTorque)
        torque = joint.maxTorque;
      if (torque < -joint.maxTorque)
        torque = -joint.maxTorque;

        cmd.motorCmd[i].q = PosStopF;
        cmd.motorCmd[i].dq = VelStopF;
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].Kd = 0;
        std::cout << "q_err: " << q_err << " dq_err: " << dq_err << " torque cmd: " << torque << std::endl;

        cmd.motorCmd[i].tau = torque;
    
  }
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    exit(-1);

  udp.SetSend(cmd);
}
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