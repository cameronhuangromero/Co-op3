/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#include "common/unitreeLeg.h"  // Make sure this is included

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007),
                          // legFR is from unitreeleg, Vec3 is position vector from hip joint to robots body frame(offset of FR)
                          legFR(0, 0.08, 0.213, 0.213, Vec3(0.188, -0.047, 0))
                          // legFL(1, 0.0838, 0.2, 0.2, Vec3(0.188, 0.047, 0)),
                          // legRR(2, 0.0838, 0.2, 0.2, Vec3(-0.188, -0.047, 0)),
                          // legRL(3, 0.0838, 0.2, 0.2, Vec3(-0.188, 0.047, 0))
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
  QuadrupedLeg legFR;
 

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
  
   // Example desired foot position in BODY frame (m)
  Vec3 desFootFR(0.2, -0.1, -0.25);  // X (forward), Y (side), Z (down)


   // Use IK to compute joint angles
   // Since in BODY frame will transform BODY to HIP using pInHip = pEe - _pHip2B
   // If was HIP frame, would just skip transform
  Vec3 qFR = legFR.calcQ(desFootFR, FrameType::BODY);
  // calcQ from unitreeLeg, pEe is desired pos, uses IK based on frametype
  
  float Kp = 20.0;
  float Kd = 2.0;

   // Set joint commands (example for FR leg)
  cmd.motorCmd[FR_0].q = qFR(0);
  cmd.motorCmd[FR_0].dq = 0;
  cmd.motorCmd[FR_0].Kp = Kp;
  cmd.motorCmd[FR_0].Kd = Kd;
  cmd.motorCmd[FR_0].tau = 0;
  // cmds are part of LowCmd object 

  cmd.motorCmd[FR_1].q = qFR(1);
  cmd.motorCmd[FR_1].dq = 0;
  cmd.motorCmd[FR_1].Kp = Kp;
  cmd.motorCmd[FR_1].Kd = Kd;
  cmd.motorCmd[FR_1].tau = 0;

  cmd.motorCmd[FR_2].q = qFR(2);
  cmd.motorCmd[FR_2].dq = 0;
  cmd.motorCmd[FR_2].Kp = Kp;
  cmd.motorCmd[FR_2].Kd = Kd;
  cmd.motorCmd[FR_2].tau = 0;

// check pos limits for each joint 
    safe.PositionLimit(cmd);
    // monitors power consumption based on cmd and state
    int res1 = safe.PowerProtect(cmd, state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
      
  

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
