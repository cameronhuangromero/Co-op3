
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <atomic>
#include "common/unitreeLeg.h"
#include "common/joint_state_buffer.h"


using namespace std;
using namespace UNITREE_LEGGED_SDK;

Mat3 Mat3Inverse(const Mat3& m) {
    float det =
        m(0,0)*(m(1,1)*m(2,2)-m(1,2)*m(2,1)) -
        m(0,1)*(m(1,0)*m(2,2)-m(1,2)*m(2,0)) +
        m(0,2)*(m(1,0)*m(2,1)-m(1,1)*m(2,0));

    if (fabs(det) < 1e-8) {
        cerr << "Matrix is singular or nearly singular, cannot invert!" << endl;
        return Mat3::Zero();  // Or some safe fallback
    }

    float invDet = 1.0f / det;

    Mat3 inv;
    inv(0,0) =  (m(1,1)*m(2,2) - m(1,2)*m(2,1)) * invDet;
    inv(0,1) = -(m(0,1)*m(2,2) - m(0,2)*m(2,1)) * invDet;
    inv(0,2) =  (m(0,1)*m(1,2) - m(0,2)*m(1,1)) * invDet;

    inv(1,0) = -(m(1,0)*m(2,2) - m(1,2)*m(2,0)) * invDet;
    inv(1,1) =  (m(0,0)*m(2,2) - m(0,2)*m(2,0)) * invDet;
    inv(1,2) = -(m(0,0)*m(1,2) - m(0,2)*m(1,0)) * invDet;

    inv(2,0) =  (m(1,0)*m(2,1) - m(1,1)*m(2,0)) * invDet;
    inv(2,1) = -(m(0,0)*m(2,1) - m(0,1)*m(2,0)) * invDet;
    inv(2,2) =  (m(0,0)*m(1,1) - m(0,1)*m(1,0)) * invDet;

    return inv;
}

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
  float dt = 0.002;
  int motiontime = 0;

  // Only one leg for simplicity — extend for more
  QuadrupedLeg legFR, legFL, legRR, legRL;
  Vec3 desiredFootForceFR = Vec3(0, 0, -20);  // Downward force in N
  Vec3 desiredFootForceFL = Vec3(0, 0, -20); 
  Vec3 desiredFootForceRR = Vec3(0, 0, -20); 
  Vec3 desiredFootForceRL = Vec3(0, 0, -20); 

private:
//   static const int kRingSize = 2000;
//   std::atomic<int> bufferIndex{0};
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


Vec3 qFR;
  for (int i = 0; i < 3; ++i)
    qFR(i) = state.motorState[FR_0 + i].q;

  // Compute joint torques using Jacobian transpose: tau = J^T * F
  Vec3 tauFR = legFR.calcTau(qFR, desiredFootForceFR);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[FR_0 + i].q = 0;
    cmd.motorCmd[FR_0 + i].dq = 0;
    cmd.motorCmd[FR_0 + i].Kp = 0;
    cmd.motorCmd[FR_0 + i].Kd = 0;
    cmd.motorCmd[FR_0 + i].tau = tauFR(i);
  }

  Vec3 qFL;
  for (int i = 0; i < 3; ++i)
    qFL(i) = state.motorState[FL_0 + i].q;

  Vec3 tauFL = legFL.calcTau(qFL, desiredFootForceFL);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[FL_0 + i].q = 0;
    cmd.motorCmd[FL_0 + i].dq = 0;
    cmd.motorCmd[FL_0 + i].Kp = 0;
    cmd.motorCmd[FL_0 + i].Kd = 0;
    cmd.motorCmd[FL_0 + i].tau = tauFL(i);
  }

  Vec3 qRR;
  for (int i = 0; i < 3; ++i)
    qRR(i) = state.motorState[RR_0 + i].q;

  Vec3 tauRR = legRR.calcTau(qRR, desiredFootForceRR);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[RR_0 + i].q = 0;
    cmd.motorCmd[RR_0 + i].dq = 0;
    cmd.motorCmd[RR_0 + i].Kp = 0;
    cmd.motorCmd[RR_0 + i].Kd = 0;
    cmd.motorCmd[RR_0 + i].tau = tauRR(i);
  }

  Vec3 qRL;
  for (int i = 0; i < 3; ++i)
    qRL(i) = state.motorState[RL_0 + i].q;

  Vec3 tauRL = legRL.calcTau(qRL, desiredFootForceRL);
  for (int i = 0; i < 3; ++i)
  {
    cmd.motorCmd[RL_0 + i].q = 0;
    cmd.motorCmd[RL_0 + i].dq = 0;
    cmd.motorCmd[RL_0 + i].Kp = 0;
    cmd.motorCmd[RL_0 + i].Kd = 0;
    cmd.motorCmd[RL_0 + i].tau = tauRL(i);
  }

 Mat3 J = legFR.calcJaco(qFR);
  Mat3 JT = J.transpose();

  Vec3 tauEst;
  for (int i = 0; i < 3; ++i)
      tauEst(i) = state.motorState[FR_0 + i].tauEst;

  Mat3 JT_inv = Mat3Inverse(JT);
  Vec3 footForceEstimate = JT_inv * tauEst;

  std::cout << "Estimated foot force (N): " << footForceEstimate.transpose() << std::endl;

  safe.PositionLimit(cmd);
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    exit(-1);

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
  auto smoothed = buffer.boxcarAverage(10);
  std::cout << "[Filtered] Joint States (boxcar avg over 10):" << std::endl;
  for (int i = 0; i < 12; ++i) {
    std::cout << "Joint " << i
              << " q: " << smoothed[i].q
              << " dq: " << smoothed[i].dq
              << " tau: " << smoothed[i].tauEst
              << std::endl;
  }
}
}


int main(void)
{
  std::cout << "Robot must be hung up and in LOW-level mode." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  custom.desiredFootForceFR = Vec3(0, 0, -5);  // Apply downward force
  custom.desiredFootForceFL = Vec3(0, 0, -5);
  custom.desiredFootForceRR = Vec3(0, 0, -5);
  custom.desiredFootForceRL = Vec3(0, 0, -5);

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
