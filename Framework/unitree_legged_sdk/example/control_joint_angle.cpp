/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <atomic>
#include <algorithm>
#include "common/joint_state_buffer.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();
    void receiveJointCommand(int joint_index, float target);


  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  // float qDes[3] = {0};
  float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01

  float qDes[12] = {0};     // Desired joint positions
  float KpDes[12] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  float KdDes[12] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  float qTarget[12] = {0};  // Final target set externally
  float stepSize = 0.005f;  // For very smooth transitions
  bool joints_initialized = false;

private:
  // static constexpr int kRingSize = 1000;  // Or however deep you want
  // struct JointState {
  //   float q;
  //   float dq;
  //   float tauEst;
  // };
  // std::array<std::array<JointState, 12>, kRingSize> stateBuffer;
  // std::atomic<int> bufferIndex{0};                // Shared index, if multithreaded
  JointStateBuffer buffer;


};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::receiveJointCommand(int joint_index, float target)
{
  if (!joints_initialized)
{
  for (int i = 0; i < 12; ++i)
  {
    qDes[i] = state.motorState[i].q;
    // qTarget[i] = qDes[i];
  }
  joints_initialized = true;
}


  if (joint_index >= 0 && joint_index < 12)
  {
    qTarget[joint_index] = target;  // just set the new goal
  }
}


void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);
  printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  // if( motiontime >= 100){
  if (motiontime > 10)
  {

    for (int i = 0; i < 12; ++i)
{
  float diff = qTarget[i] - qDes[i];

  if (fabs(diff) < stepSize)
    qDes[i] = qTarget[i];  // close enough, snap to target
  else
    qDes[i] += (diff > 0 ? stepSize : -stepSize);  // move a bit toward target

  cmd.motorCmd[i].q = qDes[i];
  cmd.motorCmd[i].dq = 0;
  cmd.motorCmd[i].Kp = KpDes[i];
  cmd.motorCmd[i].Kd = KdDes[i];
  cmd.motorCmd[i].tau = 0.0f;
}
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

//   int index = bufferIndex.fetch_add(1) % kRingSize;
//     for (int i = 0; i < 12; ++i) {
//         stateBuffer[index][i] = {
//             state.motorState[i].q,
//             state.motorState[i].dq,
//             state.motorState[i].tauEst
//     };
// }

//     if (motiontime % 1000 == 0) {
//     std::cout << "Joint states at buffer[" << index << "]:" << std::endl;
//     for (int i = 0; i < 12; ++i) {
//         std::cout << "Joint " << i << " q: " << stateBuffer[index][i].q
//                   << ", dq: " << stateBuffer[index][i].dq
//                   << ", tau: " << stateBuffer[index][i].tauEst << std::endl;
//     }
// }
  std::array<JointState, 12> currentStates;
    for (int i = 0; i < 12; ++i) {
      currentStates[i] = {
        state.motorState[i].q,
        state.motorState[i].dq,
        state.motorState[i].tauEst
      };
    }
  buffer.push(currentStates);

  if (motiontime % 1000 == 0) {
    auto latest = buffer.latest();
    std::cout << "Latest joint q[0]: " << latest[0].q << ", dq[0]: " << latest[0].dq << std::endl;
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
  // InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  custom.receiveJointCommand(1, 1.4f);


  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}