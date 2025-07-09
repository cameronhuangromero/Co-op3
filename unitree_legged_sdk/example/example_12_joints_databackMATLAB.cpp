/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <csignal>
#include <iomanip>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

std::chrono::high_resolution_clock::time_point program_start;

class Custom {
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007) {
    udp.InitCmdData(cmd);
  }

  void UDPSend() { udp.Send(); }
  void UDPRecv() { udp.Recv(); }
  void RobotControl();

  struct LogEntry {
    std::string jointName;   // ← NEW
    float tauEst;
    float targetTau;
    float error;
    int64_t reachTime_us;
    int64_t pgmtime_us;
    int64_t time_since_last_entry_us = -1;
    int64_t time_from_start_us = 0;
};


  std::vector<LogEntry> logData;
  std::vector<int64_t> joint_reach_times = std::vector<int64_t>(12, -1);
  std::vector<std::chrono::high_resolution_clock::time_point> joint_command_times = std::vector<std::chrono::high_resolution_clock::time_point>(12);
  std::vector<bool> joint_waiting = std::vector<bool>(12, false);


  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.001;
};

void Custom::RobotControl() {
  auto loop_start = std::chrono::high_resolution_clock::now();
motiontime++;
udp.GetRecv(state);

std::string jointNames[12] = {
  "FR_0", "FR_1", "FR_2", "FL_0", "FL_1", "FL_2",
  "RR_0", "RR_1", "RR_2", "RL_0", "RL_1", "RL_2"
};

// Gravity compensation
cmd.motorCmd[FR_0].tau = -0.65f;
cmd.motorCmd[FL_0].tau = +0.65f;
cmd.motorCmd[RR_0].tau = -0.65f;
cmd.motorCmd[RL_0].tau = +0.65f;

if (motiontime >= 500) {
  float Kp = 10.0f;

  struct JointConfig {
    int index;
    float maxTorque;
  };

  JointConfig joints[] = {
    {FR_0, 0.5f}, {FR_1, 3.0f}, {FR_2, 0.5f},
    {FL_0, 0.5f}, {FL_1, 3.0f}, {FL_2, 0.5f},
    {RR_0, 0.5f}, {RR_1, 3.0f}, {RR_2, 0.5f},
    {RL_0, 0.5f}, {RL_1, 3.0f}, {RL_2, 0.5f},
  };



auto now = std::chrono::high_resolution_clock::now();
  static std::chrono::high_resolution_clock::time_point last_log_time = now;

  for (const auto& joint : joints) {
int i = joint.index;
    

// Store time only once when first sending command
      if (!joint_waiting[i]) {
          joint_command_times[i] = now;
          joint_waiting[i] = true;
      }

    
    float q_des = 0.0f;
    float dq_des = 0.0f;

    float q_err = q_des - state.motorState[i].q;
    float dq_err = dq_des - state.motorState[i].dq;
    float Kd = (i % 3 == 2) ? 4.0f : 1.0f;
    float torque = Kp * q_err + Kd * dq_err;

    if (torque > joint.maxTorque) torque = joint.maxTorque;
    if (torque < -joint.maxTorque) torque = -joint.maxTorque;

    cmd.motorCmd[i].q = PosStopF;
    cmd.motorCmd[i].dq = VelStopF;
    cmd.motorCmd[i].Kp = 0;
    cmd.motorCmd[i].Kd = 0;
    cmd.motorCmd[i].tau = torque;

    float error = fabs(torque - state.motorState[i].tauEst);
if (joint_reach_times[i] == -1 && error < 0.03f) {
    joint_reach_times[i] = std::chrono::duration_cast<std::chrono::microseconds>(now - joint_command_times[i]).count();
    joint_waiting[i] = false;
  }

    LogEntry entry;
    entry.jointName = jointNames[i];
    entry.tauEst = state.motorState[i].tauEst;
    entry.targetTau = torque;
    entry.error = torque - state.motorState[i].tauEst;
    entry.reachTime_us = joint_reach_times[i];  // either -1 or previously set time

  auto loop_end = std::chrono::high_resolution_clock::now();

    entry.time_from_start_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - program_start).count();
    entry.time_since_last_entry_us = std::chrono::duration_cast<std::chrono::microseconds>(now - last_log_time).count();
    entry.pgmtime_us = std::chrono::duration_cast<std::chrono::microseconds>(now - loop_start).count();

    logData.push_back(entry);
  }

  last_log_time = now;
}

int res = safe.PowerProtect(cmd, state, 1);
if (res < 0) exit(-1);

udp.SetSend(cmd);

}

Custom* custom_ptr = nullptr;

void signalHandler(int signum) {
  std::cout << "\n[INFO] Interrupt signal (" << signum << ") received. Saving data...\n";

  if (custom_ptr) {
    std::ofstream file("robot_log_" + std::to_string(time(nullptr)) + ".csv");
    file << std::fixed << std::setprecision(6);
file << "jointName,tauEst,targetTau,error,reachtime,timesincelastentry,timefromstart,pgmtime\n";

    for (const auto& entry : custom_ptr->logData) {
      file << entry.jointName << "," << entry.tauEst << "," << entry.targetTau << "," 
     << entry.error << "," << entry.reachTime_us << "," 
     << entry.time_since_last_entry_us << "," << entry.time_from_start_us 
     << "," << entry.pgmtime_us << "\n";

    }

    file.close();
    std::cout << "[INFO] Data written to CSV.\n";
  }
  exit(signum);
}

int main(void) {
  std::cout << "Communication level is set to LOW-level.\n"
            << "WARNING: Make sure the robot is hung up.\n"
            << "NOTE: The robot also needs to be set to LOW-level mode.\n"
            << "Press Enter to continue...\n";
  std::cin.ignore();

  signal(SIGINT, signalHandler);
  Custom custom(LOWLEVEL);
  custom_ptr = &custom;
  program_start = std::chrono::high_resolution_clock::now();

  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1) {
    sleep(10);
  }

  return 0;
}
