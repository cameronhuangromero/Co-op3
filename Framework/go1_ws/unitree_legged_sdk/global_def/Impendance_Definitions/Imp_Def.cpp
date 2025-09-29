#include <array>
#include <iostream>
#include <stdexcept>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "common/unitreeLeg.h"

using namespace UNITREE_LEGGED_SDK;

class TaskSpaceImpedance {
public:
    TaskSpaceImpedance()
    {
        // default: moderate stiffness/damping per axis (x, y, z)
        stiffness.fill(50.0f);
        damping.fill(5.0f);
    }

    void setStiffness(const std::array<float, 3>& k)
    {
        stiffness = k;
    }

    void setDamping(const std::array<float, 3>& b)
    {
        damping = b;
    }

    // Check if gains are valid before computing torques
    bool validateGains()
    {
        for (int i = 0; i < 3; ++i) {
            if (stiffness[i] < 0.0f || damping[i] < 0.0f) {
                sendUDPWarning("Negative stiffness or damping detected! Emergency stop.");
                logError("Negative gain detected on axis " + std::to_string(i));
                zeroAllTorques();
                return false;
            }
        }
        return true;
    }

    // Main control calculation
    std::array<float, 3> computeTaskSpaceForce(
        const std::array<float, 3>& pos_error,
        const std::array<float, 3>& vel_error)
    {
        if (!validateGains()) {
            return {0.0f, 0.0f, 0.0f};
        }

        std::array<float, 3> force;
        for (int i = 0; i < 3; ++i) {
            force[i] = stiffness[i] * pos_error[i] + damping[i] * vel_error[i];
        }
        return force;
    }

    // Compute joint torques for all 4 legs from task-space forces
    void computeAllLegTorques(const std::array<Vec3, 4>& pos_errors,
                              const std::array<Vec3, 4>& vel_errors,
                              const std::array<Vec3, 4>& q,
                              std::array<Vec3, 4>& tau_out)
    {
        for (int leg = 0; leg < 4; ++leg) {
            std::array<float, 3> pe = {pos_errors[leg][0], pos_errors[leg][1], pos_errors[leg][2]};
            std::array<float, 3> ve = {vel_errors[leg][0], vel_errors[leg][1], vel_errors[leg][2]};
            auto f = computeTaskSpaceForce(pe, ve);
            Vec3 force_vec(f[0], f[1], f[2]);
            tau_out[leg] = legs[leg].calcTau(q[leg], force_vec);
        }
    }

private:
    std::array<float, 3> stiffness;  // per axis
    std::array<float, 3> damping;    // per axis

    // Go1 leg kinematics for each leg
    Go1Leg legs[4] = {
        Go1Leg(0, Vec3( 0.1805, -0.047, 0.0)),
        Go1Leg(1, Vec3( 0.1805,  0.047, 0.0)),
        Go1Leg(2, Vec3(-0.1805, -0.047, 0.0)),
        Go1Leg(3, Vec3(-0.1805,  0.047, 0.0))
    };

    void sendUDPWarning(const std::string& msg)
    {
        // Example: send UDP packet with warning
        // You'd use UNITREE_LEGGED_SDK::UDP if integrated into robot code
        std::cerr << "[UDP WARNING] " << msg << std::endl;
    }

    void logError(const std::string& msg)
    {
        // Replace with RCLCPP_ERROR for ROS 2
        std::cerr << "[ERROR] " << msg << std::endl;
    }

    void zeroAllTorques()
    {
        // This assumes you’ll integrate with robot cmd struct
        // Example:
        std::cerr << "[SAFETY] Zeroing all torques" << std::endl;
        // In your actual control loop:
        // for (int m = 0; m < 12; m++) cmd.motorCmd[m].tau = 0.0f;
    }
};
