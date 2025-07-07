/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include <chrono>

using namespace std;
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
  void UDPLoop();
  // void PINGLoop();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float qDes[3] = {0};
  float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01

  bool response_detected = false;
  float baseline_q = 0.0;
  bool command_sent = false;
  bool waiting_for_response = false;
  double target_q = 0.0;

};

void Custom::UDPLoop()
{
  udp.Send();
  udp__send = std::chrono::high_resolution_clock::now();
  udp.Recv();

  target_q = cmd.motorCmd[FR_2].q;
  double current_q = state.motorState[FR_2].q;
  double error = fabs(current_q - target_q);

  // std::cout << "[INFO] Command issued: q = " << target_q << std::endl;
  // std::cout << "[INFO] Current State: q = "<< current_q << std::endl;
  printf("%f %f", cmd.motorCmd[FR_2].q, state.motorState[FR_2].q);
  
  if (error > .04)
      {
        udp__recv = std::chrono::high_resolution_clock::now();

        //printf("%d", motiontime)
        
        //HERE//
        
        // std::cout << "Time to send and recieve: "
        //           << std::chrono::duration_cast<std::chrono::microseconds>(udp__recv - udp__send).count()
        //           << " microseconds" << std::endl;
      }



}

// void Custom::PINGLoop()
// // continous check if at position
// {
  
// }

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::RobotControl()
// readout time difference between issuing command and getting data back
{
  motiontime++;
  udp.GetRecv(state);
  //printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  // if( motiontime >= 100){
  if (motiontime >= 0)
  {
    // first, get record initial position
    // if( motiontime >= 100 && motiontime < 500){
    if (motiontime >= 0 && motiontime < 10)
    {
      qInit[0] = state.motorState[FR_0].q;
      qInit[1] = state.motorState[FR_1].q;
      qInit[2] = state.motorState[FR_2].q;
    }
    // second, move to the origin point of a sine movement with Kp Kd
    // if( motiontime >= 500 && motiontime < 1500){
    if (motiontime >= 10 && motiontime < 400)
    {
      rate_count++;
      double rate = rate_count / 200.0; // needs count to 200
      Kp[0] = 5.0;
      Kp[1] = 5.0;
      Kp[2] = 5.0;
      Kd[0] = 1.0;
      Kd[1] = 1.0;
      Kd[2] = 1.0;
      // Kp[0] = 20.0; Kp[1] = 20.0; Kp[2] = 20.0;
      // Kd[0] = 2.0; Kd[1] = 2.0; Kd[2] = 2.0;

      qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
      qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
      qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

      cmd.motorCmd[FR_0].q = qDes[0];
      cmd.motorCmd[FR_0].dq = 0;
      cmd.motorCmd[FR_0].Kp = Kp[0];
      cmd.motorCmd[FR_0].Kd = Kd[0];
      cmd.motorCmd[FR_0].tau = -0.65f;

      cmd.motorCmd[FR_1].q = qDes[1];
      cmd.motorCmd[FR_1].dq = 0;
      cmd.motorCmd[FR_1].Kp = Kp[1];
      cmd.motorCmd[FR_1].Kd = Kd[1];
      cmd.motorCmd[FR_1].tau = 0.0f;

      cmd.motorCmd[FR_2].q = qDes[2];
      cmd.motorCmd[FR_2].dq = 0;
      cmd.motorCmd[FR_2].Kp = Kp[2];
      cmd.motorCmd[FR_2].Kd = Kd[2];
      cmd.motorCmd[FR_2].tau = 0.0f;


    }
    double sin_joint1, sin_joint2;
    if (motiontime >= 400)
    {
      // sin_count++;
      // sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
      // sin_joint2 = -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
      // qDes[0] = sin_mid_q[0];
      // qDes[1] = sin_mid_q[1];
      // // sending to FR_2
      // qDes[2] = sin_mid_q[2] + sin_joint2;

      //sin_count++;
      sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
      sin_joint2 = -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
      qDes[0] = sin_mid_q[0];
      qDes[1] = sin_mid_q[1];
                // sending to FR_2
      qDes[2] = sin_mid_q[2] + sin_joint2;

      cmd.motorCmd[FR_2].q = qDes[2];
      cmd.motorCmd[FR_2].dq = 0;
      cmd.motorCmd[FR_2].Kp = Kp[2];
      cmd.motorCmd[FR_2].Kd = Kd[2];
      cmd.motorCmd[FR_2].tau = 0.0f;
      target_q = cmd.motorCmd[FR_2].q;
                  //waiting_for_response = true;

      std::cout << "[INFO] Command issued: q = " << target_q << std::endl;

      double last_position = state.motorState[FR_2].q; // Track last known position
      auto last_progress_time = std::chrono::high_resolution_clock::now(); // Track last pr

      if (!waiting_for_response)
              {
                
                  command_issue_time = std::chrono::high_resolution_clock::now();
                  waiting_for_response = true;

                      last_progress_time = command_issue_time; // Initialize last progress time
                      last_position = state.motorState[FR_2].q; // Initialize last position
                  
              }

    // Check if state reached the target within a small threshold
      double current_q = state.motorState[FR_2].q;
      // printf("%f", state.motorState[FR_2].q);
      std::cout << "[INFO] Current State: q = " << current_q << std::endl;
      double error = fabs(current_q - target_q);
      
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(now - command_issue_time).count();


// Start a sub-loop to repeatedly check progress within a single iteration
while (elapsed_time_us <= 5000) { // Check for up to 2 seconds
    now = std::chrono::high_resolution_clock::now();
    elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(now - command_issue_time).count();

    // Update progress tracking if robot moves
    current_q = state.motorState[FR_2].q;
    if (fabs(current_q - last_position) > 0.01) {
        last_progress_time = now; // Update progress time
        last_position = current_q; // Update last known position
        break; // Exit the sub-loop if progress is detected
    }

    // Optionally add a small delay in between while loop checks to prevent busy-waiting 
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// If no progress detected in the full 2 seconds, send a new command

// std::cout << "elapsed time " << elapsed_time_us << " microseconds" << std::endl;
      if (error < 0.03) {
    // This block executes only if error is below the threshold
    response_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(response_time - command_issue_time).count();

    std::cout << "[REACHED] Target position reached!" << std::endl;
    std::cout << "Time to reach target: " << duration_us << " microseconds" << std::endl;
}

    if (elapsed_time_us > 5000) {
      std::cout << "Elapsed Time Maxed Out. Sending New Command" << std::endl;

    }

// This block executes if either condition is true
if (error < 0.03 || elapsed_time_us > 5000) {
    waiting_for_response = false; // Reset for next command
    sin_count++;
}
    }

    // cmd.motorCmd[FR_0].q = qDes[0];
    // cmd.motorCmd[FR_0].dq = 0;
    // cmd.motorCmd[FR_0].Kp = Kp[0];
    // cmd.motorCmd[FR_0].Kd = Kd[0];
    // cmd.motorCmd[FR_0].tau = -0.65f;

    // cmd.motorCmd[FR_1].q = qDes[1];
    // cmd.motorCmd[FR_1].dq = 0;
    // cmd.motorCmd[FR_1].Kp = Kp[1];
    // cmd.motorCmd[FR_1].Kd = Kd[1];
    // cmd.motorCmd[FR_1].tau = 0.0f;

    // cmd.motorCmd[FR_2].q = qDes[2];
    // cmd.motorCmd[FR_2].dq = 0;
    // cmd.motorCmd[FR_2].Kp = Kp[2];
    // cmd.motorCmd[FR_2].Kd = Kd[2];
    // cmd.motorCmd[FR_2].tau = 0.0f;
    
     
}
    
  

  if (motiontime > 10)
  {
    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
  }

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter! to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  // InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  //LoopFunc loop_pingloop("ping_loop", custom.dt, boost::bind(&Custom::PINGLoop, &custom));
  LoopFunc loop_udploop("udp_loop", custom.dt, 3, boost::bind(&Custom::UDPLoop, &custom));

  //loop_pingloop.start();
  loop_udploop.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}