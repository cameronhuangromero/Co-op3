/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <algorithm> // for std::clamp



using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
                          

  {
    udp.InitCmdData(cmd);
    for (int i = 0; i < 12; ++i) {
      bufferedTorques[i].store(0.0f);
      torqueEnabled[i].store(false);
    }
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl();  // for LoopFunc
  void singlejointtorque(int jointIndex, float userTorque);
  int getJointIndexFromName(const std::string& jointName);
  void alljointtorque(const std::array<float, 12>& torques);
  void legjointtorque(int legPrefix, float tau0, float tau1, float tau2);

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
  enum LegID { FR = 0, FL = 1, RR = 2, RL = 3 };

  

private:
  std::array<std::atomic<float>, 12> bufferedTorques{};
  std::array<std::atomic<bool>, 12> torqueEnabled{};  // Which joints are active
  // std::array<float, 12> GetJointTorques() const;

  static constexpr int kRingSize = 1000;  // Keep last 1000 readings
  std::array<std::array<float, 12>, kRingSize> torqueBuffer;
  std::atomic<int> bufferIndex{0};
  

};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}





// from ROS2?
void Custom::singlejointtorque(int jointIndex, float userTorque) {
  // ?
    if (jointIndex < 0 || jointIndex >= 12) {
        std::cerr << "Invalid joint index: " << jointIndex << std::endl;
        return;
    }
    bufferedTorques[jointIndex].store(std::clamp(userTorque, -5.0f, 5.0f));
    torqueEnabled[jointIndex].store(true);
}


void Custom::legjointtorque(int legIndex, float tau0, float tau1, float tau2) {
    if (legIndex < Custom::FR || legIndex > Custom::RL){
        std::cerr << "Invalid leg index" << std::endl;
        return;
    }
    
    int baseIndex = legIndex * 3;
    float torques[3] = {tau0, tau1, tau2};

    for (int i = 0; i < 3; ++i) {
        int jointIndex = baseIndex + i;
        bufferedTorques[jointIndex].store(std::clamp(torques[i], -5.0f, 5.0f));
        torqueEnabled[jointIndex].store(true);
    }
}




void Custom::alljointtorque(const std::array<float, 12>& torques) {
    for (int i = 0; i < 12; ++i) {
        bufferedTorques[i].store(std::clamp(torques[i], -5.0f, 5.0f));
        torqueEnabled[i].store(true);
    }
}




void Custom::RobotControl() {
    motiontime++;
    udp.GetRecv(state);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    if (motiontime >= 500)
    {
        for (int i = 0; i < 12; ++i) {
            if (torqueEnabled[i].load()) {
                cmd.motorCmd[i].q = PosStopF;
                cmd.motorCmd[i].dq = VelStopF;
                cmd.motorCmd[i].Kp = 0;
                cmd.motorCmd[i].Kd = 0;
                cmd.motorCmd[i].tau = bufferedTorques[i].load();
                std::cout << "Joint " << i << " torque: " << bufferedTorques[i].load() << std::endl;
            }
        }
    }

    int res = safe.PowerProtect(cmd, state, 1);
    if (res < 0)
        exit(-1);

    udp.SetSend(cmd);

    // Save current torques to ring buffer
    int index = bufferIndex.fetch_add(1) % kRingSize;
    for (int i = 0; i < 12; ++i) {
        torqueBuffer[index][i] = state.motorState[i].tauEst;
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
custom.singlejointtorque(FR_1, 0.2f);

// custom.legjointtorque(Custom::FL, 0.1f, 0.2f, 0.3f);

// std::array<float, 12> allTaus = {0.1f, 0.1f, 0.1f, 0.2f, 0.2f, 0.2f,
//                                  0.3f, 0.3f, 0.3f, 0.4f, 0.4f, 0.4f};
// custom.alljointtorque(allTaus);
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