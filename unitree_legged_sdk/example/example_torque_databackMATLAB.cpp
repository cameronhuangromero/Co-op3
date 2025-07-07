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
#include <iomanip>

std::chrono::high_resolution_clock::time_point program_start;


using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:

    std::chrono::high_resolution_clock::time_point command_issue_time;
    std::chrono::high_resolution_clock::time_point response_time;
    std::chrono::high_resolution_clock::time_point udp__send;
    std::chrono::high_resolution_clock::time_point udp__recv;
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point endtime;
    std::chrono::high_resolution_clock::time_point last_log_entry_time;

  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  //void UDPSend();
  void UDPLoop();
  void RobotControl();

  struct LogEntry {
    float tauEst;
    float targetTau;
    float error;
    int64_t reachTime_us;
    int64_t pgmtime_us;
    int64_t time_since_last_entry_us = -1; // Add this
    int64_t time_from_start_us = 0; // Add this to track time from program start
};

std::vector<LogEntry> logData;


  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.001; // 0.001~0.01

  bool waiting_for_response = false;
  
  int count = 0;
  bool has_logged_once = false;
  //static bool torque_computed = false;
};

void Custom::UDPLoop()
{
    static float target_tau = 0.0f;
    udp.Send();
    udp__send = std::chrono::high_resolution_clock::now();
    udp.Recv();

    target_tau = cmd.motorCmd[FR_1].tau;
    double current_tau = state.motorState[FR_1].tauEst;
    double error = fabs(current_tau - target_tau);

    // std::cout << "[INFO] Command issued: q = " << target_tau << std::endl;
    // std::cout << "[INFO] Current State: q = "<< current_tau << std::endl;
    // printf("%f %f", cmd.motorCmd[FR_1].tau, state.motorState[FR_1].tauEst);

    if (error > 0.03) {
        udp__recv = std::chrono::high_resolution_clock::now();

        // std::cout << "Time to send and recieve: "
        //           << std::chrono::duration_cast<std::chrono::microseconds>(udp__recv - udp__send).count()
        //           << " microseconds" << std::endl;

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

  start_time = std::chrono::high_resolution_clock::now();
  

  if (motiontime >= 500)
  {
    
    
    

   
        

    
  
    if (!waiting_for_response)
              {
                float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
        
                if (torque > 2.0f)
                  torque = 2.0f;
                if (torque < -2.0f)
                  torque = -2.0f;

                  // log this time
                  
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

      // auto now = std::chrono::high_resolution_clock::now();
      // int64_t time_us = std::chrono::duration_cast<std::chrono::microseconds>(now - program_start).count();     

      Custom::LogEntry entry;
      // entry.time_us = time_us; // ← Your time function here
      entry.tauEst = current_tau;                     // ← Your actual estimated torque
      entry.targetTau = target_tau;            // ← Your desired torque (e.g. 0)
      entry.error = entry.targetTau - entry.tauEst;
      entry.reachTime_us = -1; // default to -1, means "not reached"
      
      
      
      


      if (error < 0.03)
      {
          response_time = std::chrono::high_resolution_clock::now();
          auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(response_time - command_issue_time).count();

          // std::cout << "[REACHED] Target position reached!" << std::endl;
          // std::cout << "Time to reach target: " << duration_us << " microseconds" << std::endl;

          // also log absolute time
          // progromatically check in code that are running at 1kHz/1ms still
          entry.reachTime_us = duration_us;
          waiting_for_response = false; // Reset for next command
          //torque_computed = false;
          
          
          
      }

      auto now = std::chrono::high_resolution_clock::now();
      entry.time_from_start_us = std::chrono::duration_cast<std::chrono::microseconds>(now - program_start).count();
      // Time between entries
      if (has_logged_once) {
          entry.time_since_last_entry_us = std::chrono::duration_cast<std::chrono::microseconds>(now - last_log_entry_time).count();
      } else {
          entry.time_since_last_entry_us = -1; // First entry
          has_logged_once = true;
      }
      last_log_entry_time = now;

     endtime = std::chrono::high_resolution_clock::now();
      auto pgmtime = std::chrono::duration_cast<std::chrono::microseconds>(endtime - start_time).count();


      entry.pgmtime_us = pgmtime;

      logData.push_back(entry);
      std::cout << "[LOG] Entry logged: tauEst=" << entry.tauEst 
          << ", targetTau=" << entry.targetTau 
          << ", error=" << entry.error 
          << ", reachtime=" << entry.reachTime_us
          
          << ", time since last entry=" << entry.time_since_last_entry_us
          << std::endl;
    }



  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    exit(-1);

  udp.SetSend(cmd);
}
Custom* custom_ptr = nullptr;


void signalHandler(int signum)
{
    std::cout << "\n[INFO] Interrupt signal (" << signum << ") received.\n";
    
    // std::ofstream file("robot_data.csv");
    if (custom_ptr) {
      std::ofstream file("robot_data_" + std::to_string(time(nullptr)) + ".csv");
      
      if (!file) {
            std::cerr << "[ERROR] Could not open file for writing\n";
            return;
      }
      file << std::fixed << std::setprecision(6); // Set float precision
      file << "tauEst,targetTau,error,reachtime,timesincelastentry,timefromstart,pgmtime\n";

      for (const auto& entry : custom_ptr->logData) {
        // std::cout << entry.tauEst << "," << entry.targetTau << "," << entry.error << std::endl;
        file << entry.tauEst << "," << entry.targetTau << "," << entry.error << "," 
        << entry.reachTime_us << "," << entry.time_since_last_entry_us << "," << entry.time_from_start_us << "," << entry.pgmtime_us << "\n";
        }

        auto program_end = std::chrono::high_resolution_clock::now();
        int64_t runtime_us = std::chrono::duration_cast<std::chrono::microseconds>(program_end - program_start).count();

        std::cout << "[INFO] Total program runtime: " << runtime_us / 1e6 << " seconds (" << runtime_us << " microseconds)\n";

        // Optionally, append runtime to the file
        file << "\n# Total Runtime (us): " << runtime_us << "\n";
        
      file.close();

      std::cout << "[INFO] Data saved to robot_data.csv\n";
      std::cout << "[DEBUG] Entries in logData: " << custom_ptr->logData.size() << std::endl;

    }
    exit(signum);
}


int main(void)
{
  
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();


  signal(SIGINT, signalHandler);  // Register Ctrl+C handler
  Custom custom(LOWLEVEL);
  
  custom_ptr = &custom;  // Set global pointer
  program_start = std::chrono::high_resolution_clock::now();
    
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