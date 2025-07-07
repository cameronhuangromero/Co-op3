/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

#include <vector>
#include <fstream>
#include <csignal>    // for signal handling
// #include <fstream>    // for file output



using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:

    std::chrono::high_resolution_clock::time_point command_issue_time;
    std::chrono::high_resolution_clock::time_point response_time;
    std::chrono::high_resolution_clock::time_point udp__send;
    std::chrono::high_resolution_clock::time_point udp__recv;
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  //void UDPSend();
  void UDPLoop();
  void RobotControl();






  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.001; // 0.001~0.01

  bool waiting_for_response = false;
  
  int count = 0;
  //static bool torque_computed = false;
};

void Custom::UDPLoop()
{
    static float target = 0.0f;
    udp.Send();
    udp__send = std::chrono::high_resolution_clock::now();
    udp.Recv();

    target = cmd.motorCmd[FR_1].tau;
    double current = state.motorState[FR_1].tauEst;
    double error = fabs(current - target);

    std::cout << "[INFO] Command issued: q = " << target << std::endl;
    std::cout << "[INFO] Current State: q = "<< current << std::endl;
    printf("%f %f", cmd.motorCmd[FR_1].tau, state.motorState[FR_1].tauEst);

    if (error > 0.03) {
        udp__recv = std::chrono::high_resolution_clock::now();

        std::cout << "Time to send and recieve: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(udp__recv - udp__send).count()
                  << " microseconds" << std::endl;

    }

}

// void Custom::UDPSend()
// {
// //   udp.Send();
// }

void Custom::RobotControl()
{
  static float target_tau = 0.0f;
  motiontime++;
  udp.GetRecv(state);
  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if (motiontime >= 500)
  {
    
    

   
        

    
  
    if (!waiting_for_response)
              {
                float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
        
                if (torque > 2.0f)
                  torque = 2.0f;
                if (torque < -2.0f)
                  torque = -2.0f;

                  command_issue_time = std::chrono::high_resolution_clock::now();
                  waiting_for_response = true;
                  target_tau = torque;
                  //torque_computed = true;
                  
                  // std::cout << "[INFO] Command issued: tau = " << target_tau << std::endl;
              }

               cmd.motorCmd[FR_1].q = PosStopF;
                cmd.motorCmd[FR_1].dq = VelStopF;
                cmd.motorCmd[FR_1].Kp = 0;
                cmd.motorCmd[FR_1].Kd = 0;
                cmd.motorCmd[FR_1].tau = target_tau;
                

    // Check if state reached the target within a small threshold
      double current_tau = state.motorState[FR_1].tauEst;

      double error = fabs(current_tau - target_tau);

      // printf("[DEBUG] tauEst = %f, target_tau = %f, error = %f\n", current_tau, target_tau, error);



      if (error < 0.03)
      {
          response_time = std::chrono::high_resolution_clock::now();
          auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(response_time - command_issue_time).count();

          // std::cout << "[REACHED] Target position reached!" << std::endl;
          // std::cout << "Time to reach target: " << duration_us << " microseconds" << std::endl;

          waiting_for_response = false; // Reset for next command
          //torque_computed = false;
          
          
          
      }
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
  // LoopFunc loop_udpSend("udp_send", custom.dt, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udploop("udp_loop", custom.dt, 3, boost::bind(&Custom::UDPLoop, &custom));

  // loop_udpSend.start();
  loop_udploop.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}