#ifndef DUMMY_IMPEDANCE_CONTROLLER
#define DUMMY_IMPEDANCE_CONTROLLER

#include "impedance_controller.hpp"

/*
    @brief Simple testing dummy interface that returns 0 torque

    Used for testing and integration

    Author: Cameron Romero
    Date: 1/12/26
*/

class DummyImpedanceController : public ImpedanceController
{
public:
    DummyImpedanceController() = default;
    ~DummyImpedanceController() override = default;

    std::array<float, kNumJoints> computeTorque(const ImpedanceControllerInput& input)
    {
        std::array<float, kNumJoints> zero_torque{};
        zero_torque.fill(1.0f);
        return zero_torque;
    }

    std::string name() const override
    {
        return "DummyController";
    }
};

#endif