#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <algorithm>

#include "common/joint_state_buffer.h"
#include "common/shm_utils.h" // for create_or_attach_shm()

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level, JointStateBuffer* shared_buffer)
        : safe(LeggedType::A1), udp(level), state(), cmd(), motiontime(0), shmBuffer(shared_buffer)
    {
        udp.InitCmdData(cmd);
    }

    void RobotControl()
    {
        motiontime++;
        udp.GetRecv(state);

        // Example: gravity compensation
        cmd.motorCmd[FR_0].tau = -0.65f;
        cmd.motorCmd[FL_0].tau = +0.65f;
        cmd.motorCmd[RR_0].tau = -0.65f;
        cmd.motorCmd[RL_0].tau = +0.65f;

        // Fill joint states
        std::array<JointState, JointStateBuffer::kNumJoints> jointStates;
        for (int i = 0; i < JointStateBuffer::kNumJoints; i++)
        {
            jointStates[i].q   = state.motorState[i].q;
            jointStates[i].dq  = state.motorState[i].dq;
            jointStates[i].tau = state.motorState[i].tauEst;
        }

        // Push to shared memory buffer
        shmBuffer->push(jointStates);

        udp.SetSend(cmd);
    }

    Safety safe;
    UDP udp;
    LowState state;
    LowCmd cmd;
    int motiontime;

private:
    JointStateBuffer* shmBuffer;
};

int main(int argc, char** argv)
{
    // Create or attach shared memory buffer
    auto* shmBuffer = create_or_attach_shm<JointStateBuffer>("/joint_state_shm", true);
    shmBuffer->clear(); // only creator clears it

    Custom custom(LOWLEVEL, shmBuffer);

    LoopFunc loop_control("control_loop", 0.002, boost::bind(&Custom::RobotControl, &custom));
    loop_control.start();

    while (true)
    {
        sleep(1);
    }

    return 0;
}
