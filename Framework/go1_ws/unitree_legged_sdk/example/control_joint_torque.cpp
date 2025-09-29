/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <algorithm> // for std::clamp
#include "common/joint_state_buffer.h"
#include <fcntl.h>    // shm_open
#include <sys/mman.h> // mmap
#include <unistd.h>   // ftruncate, close



using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level, JointStateBuffer* buf, TorqueCommandBuffer* cmd_buf) 
    : safe(LeggedType::Go1),
      udp(level, 8090, "192.168.123.10", 8007),
      buffer(buf),    // pass in shared memory buffer
      command_buffer(cmd_buf)
                          

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

  // static constexpr int kRingSize = 1000;  // Keep last 1000 readings
  // struct JointState {
  //   float q;
  //   float dq;
  //   float tauEst;
  // };
  // std::array<std::array<JointState, 12>, kRingSize> stateBuffer;
  // std::atomic<int> bufferIndex{0};
  JointStateBuffer* buffer; 
  TorqueCommandBuffer* command_buffer;

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

    // if (motiontime >= 500)
    if (motiontime >= 500 && motiontime < 506)
    {
        for (int i = 0; i < 12; ++i) {
            // if (torqueEnabled[i].load()) 
            if (command_buffer->enabled[i].load())
            {
                cmd.motorCmd[i].q = PosStopF;
                cmd.motorCmd[i].dq = VelStopF;
                cmd.motorCmd[i].Kp = 0;
                cmd.motorCmd[i].Kd = 0;
                // cmd.motorCmd[i].tau = bufferedTorques[i].load();
                cmd.motorCmd[i].tau = std::clamp(command_buffer->torques[i].load(), -5.0f, 5.0f);
                std::cout << "Joint " << i << " torque: " << bufferedTorques[i].load() << std::endl;
            }
        }
    }

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
buffer->push(currentStates);

  if (motiontime % 1000 == 0) {
  auto smoothed = buffer->boxcarAverage(10);
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
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();


  // --- Create shared memory for joint state buffer ---
  const char* shm_name = "/joint_state_buffer";
  int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
  if (fd < 0) {
    perror("shm_open failed");
    return -1;
  }
  ftruncate(fd, sizeof(JointStateBuffer));
  JointStateBuffer* shm_buffer = static_cast<JointStateBuffer*>(
      mmap(nullptr, sizeof(JointStateBuffer), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  close(fd);

  if (shm_buffer == MAP_FAILED) {
    perror("mmap failed");
    return -1;
  }

  // Initialize buffer (only once, here in writer process)
  shm_buffer->init();

  // --- Create shared memory for torque command buffer ---
const char* shm_cmd_name = "/torque_command_buffer";
int fd_cmd = shm_open(shm_cmd_name, O_CREAT | O_RDWR, 0666);
if (fd_cmd < 0) {
    perror("shm_open torque_command_buffer failed");
    return -1;
}
ftruncate(fd_cmd, sizeof(TorqueCommandBuffer));
TorqueCommandBuffer* shm_cmd_buffer = static_cast<TorqueCommandBuffer*>(
    mmap(nullptr, sizeof(TorqueCommandBuffer),
         PROT_READ | PROT_WRITE, MAP_SHARED, fd_cmd, 0));
close(fd_cmd);

if (shm_cmd_buffer == MAP_FAILED) {
    perror("mmap torque_command_buffer failed");
    return -1;
}

// Initialize command buffer (only once, here in writer process)
shm_cmd_buffer->init();


  Custom custom(LOWLEVEL, shm_buffer, shm_cmd_buffer);
  // custom.singlejointtorque(FR_1, 0.2f);

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