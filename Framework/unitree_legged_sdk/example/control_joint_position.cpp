
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include "common/unitreeLeg.h"  // Make sure this is included
#include <atomic>   // for atomic counter
#include "common/joint_state_buffer.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007),
                          // legFR is from unitreeleg, Vec3 is position vector from hip joint to robots body frame(offset of FR)
                          legFR(0, 0.08, 0.213, 0.213, Vec3(0.188, -0.047, 0)),
                          legFL(1, 0.0838, 0.2, 0.2, Vec3(0.188, 0.047, 0)),
                          legRR(2, 0.0838, 0.2, 0.2, Vec3(-0.188, -0.047, 0)),
                          legRL(3, 0.0838, 0.2, 0.2, Vec3(-0.188, 0.047, 0))
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
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

  // IK legs
    QuadrupedLeg legFR, legFL, legRR, legRL;
    Vec3 desFootFR = Vec3(0.2, -0.1, -0.25);  // X (forward), Y (side), Z (down)
    Vec3 desFootFL = Vec3(0.2, 0.1, -0.25);
    Vec3 desFootRR = Vec3(-0.2, -0.1, -0.25);
    Vec3 desFootRL = Vec3(-0.2, 0.1, -0.25);

private:
//     static const int kRingSize = 2000;         // how many snapshots to store
//   std::atomic<int> bufferIndex{0};           // thread-safe index counter
//   struct JointState {
//     float q;
//     float dq;
//     float tauEst;
//   };
//   std::array<std::array<JointState, 12>, kRingSize> stateBuffer;
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



void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);

  float Kp = 20.0;
  float Kd = 2.0;

  // Front Right (FR)
  Vec3 qFR = legFR.calcQ(desFootFR, FrameType::BODY);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[FR_0 + i].q = qFR(i);
    cmd.motorCmd[FR_0 + i].dq = 0;
    cmd.motorCmd[FR_0 + i].Kp = Kp;
    cmd.motorCmd[FR_0 + i].Kd = Kd;
    cmd.motorCmd[FR_0 + i].tau = 0;
  }

  // Front Left (FL)
  Vec3 qFL = legFL.calcQ(desFootFL, FrameType::BODY);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[FL_0 + i].q = qFL(i);
    cmd.motorCmd[FL_0 + i].dq = 0;
    cmd.motorCmd[FL_0 + i].Kp = Kp;
    cmd.motorCmd[FL_0 + i].Kd = Kd;
    cmd.motorCmd[FL_0 + i].tau = 0;
  }

  // Rear Right (RR)
  Vec3 qRR = legRR.calcQ(desFootRR, FrameType::BODY);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[RR_0 + i].q = qRR(i);
    cmd.motorCmd[RR_0 + i].dq = 0;
    cmd.motorCmd[RR_0 + i].Kp = Kp;
    cmd.motorCmd[RR_0 + i].Kd = Kd;
    cmd.motorCmd[RR_0 + i].tau = 0;
  }

  // Rear Left (RL)
  Vec3 qRL = legRL.calcQ(desFootRL, FrameType::BODY);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[RL_0 + i].q = qRL(i);
    cmd.motorCmd[RL_0 + i].dq = 0;
    cmd.motorCmd[RL_0 + i].Kp = Kp;
    cmd.motorCmd[RL_0 + i].Kd = Kd;
    cmd.motorCmd[RL_0 + i].tau = 0;
  }

  // Safety and send
  safe.PositionLimit(cmd);
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    exit(-1);

  udp.SetSend(cmd);

//     int index = bufferIndex.fetch_add(1) % kRingSize;
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
            << "Press Enter! to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
// commands
    custom.desFootFR = Vec3(0.2, -0.1, -0.23);
  custom.desFootFL = Vec3(0.2, 0.1, -0.23);
  custom.desFootRR = Vec3(-0.2, -0.1, -0.23);
  custom.desFootRL = Vec3(-0.2, 0.1, -0.23);
  // InitEnvironment();
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