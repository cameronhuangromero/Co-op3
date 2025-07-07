/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

#include <chrono>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:

  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::time_point end_time;

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
  float dt = 0.001; // 0.001~0.01
};

void Custom::UDPRecv()
{
  start_time = std::chrono::high_resolution_clock::now();
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
  end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> time_span = end_time - start_time; // in microseconds
  std::cout << "Time between UDPRecv and UDPSend: " << time_span.count() << " microseconds" << std::endl;
}

void Custom::RobotControl()
{
  try {
  std::cout << "[DEBUG] RobotControl entered. motiontime: " << motiontime << std::endl;
  motiontime++;
  udp.GetRecv(state);

  // Print to confirm loop is running
  std::cout << "[Control Loop] motiontime: " << motiontime << std::endl;

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;



  if (motiontime >= 500)
  {
    float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
    if (torque > 5.0f)
      torque = 5.0f;
    if (torque < -5.0f)
      torque = -5.0f;


      // Debug output to confirm control is active
      std::cout << "[FR_1] Pos: " << state.motorState[FR_1].q
      << " Vel: " << state.motorState[FR_1].dq
      << " -> Torque: " << torque << std::endl;

    cmd.motorCmd[FR_1].q = PosStopF;
    cmd.motorCmd[FR_1].dq = VelStopF;
    cmd.motorCmd[FR_1].Kp = 0;
    cmd.motorCmd[FR_1].Kd = 0;
    cmd.motorCmd[FR_1].tau = torque;
  }

  std::cout << "[DEBUG] Before PowerProtect:" << std::endl;
  std::cout << "  FR_0.tau = " << cmd.motorCmd[FR_0].tau << std::endl;
  std::cout << "  FL_0.tau = " << cmd.motorCmd[FL_0].tau << std::endl;
  std::cout << "  RR_0.tau = " << cmd.motorCmd[RR_0].tau << std::endl;
  std::cout << "  RL_0.tau = " << cmd.motorCmd[RL_0].tau << std::endl;
  std::cout << "  FR_1.tau = " << cmd.motorCmd[FR_1].tau << std::endl;


  std::cout << "[DEBUG] MotorState FR_1:"
          << " Pos=" << state.motorState[FR_1].q
          << " Vel=" << state.motorState[FR_1].dq
          << " Tau=" << cmd.motorCmd[FR_1].tau
          << std::endl;


  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0) {
    std::cerr << "[ERROR] PowerProtect triggered. Exiting." << res << std::endl;
    exit(-1);
  }

  udp.SetSend(cmd);
} catch (const std::exception& e) {
  std::cerr << "[EXCEPTION] in RobotControl: " << e.what() << std::endl;
} catch (...) {
  std::cerr << "[EXCEPTION] Unknown error in RobotControl" << std::endl;
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
  std::cout << "[DEBUG] loop_udpSend started" << std::endl;

  loop_udpRecv.start();
  std::cout << "[DEBUG] loop_udpRecv started" << std::endl;

  loop_control.start();
  std::cout << "[DEBUG] loop_control started" << std::endl;

  while (1)
  {
    sleep(10);
  };

  return 0;
}
