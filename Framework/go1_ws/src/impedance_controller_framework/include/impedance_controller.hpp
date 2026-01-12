#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <array>
#include <optional>
#include <string>
#include <Eigen/Dense>

/*
    @brief Defines interfaces and data structures for impedance controllers

    Declares the abstract base class "ImpedanceController", which specifies required interface for all imp. cons., including the method to computer torques from joint states

    Also defines data structures to represent joint space and task space states, which are encapsulated in the input handler passed to controllers

    Author: Cameron Romero
    Date: 1/12/26
*/

struct JointState
{
    static constexpr int kNumJoints = 12;

    std::array<float, kNumJoints> positions{};
    std::array<float, kNumJoints> velocities{};
    std::array<float, kNumJoints> efforts{};
};

struct TaskSpaceState
{
    Eigen::Vector3f position{};
    Eigen::Quaternionf orientation{};
    Eigen::Vector3f linear_velocity{};
    Eigen::Vector3f angular_velocity{};
};

struct ImpedanceControllerInput
{
    std::optional<JointState> joint_state;
    std::optional<TaskSpaceState> task_space_state;
};

class ImpedanceController
{
public:
    static constexpr int kNumJoints = 12;

    virtual ~ImpedanceController() = default;

    virtual std::array<float, kNumJoints> computeTorque(const ImpedanceControllerInput& input) = 0;
    virtual std::string name() const = 0;
};

#endif