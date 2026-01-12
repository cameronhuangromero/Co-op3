#ifndef IMPEDANCE_CONTROLLER_MANAGER
#define IMPEDANCE_CONTROLLER_MANAGER

#include <memory>
#include <string>
#include <mutex>
#include "impedance_controller.hpp"

/*
    @brief Manages a single active impedance controller instance

    Allows switching between different impedance controllers
    Forwards input to controllers and fetches output torque from current controller

    Author: Cameron Romero
    Date: 1/12/26
*/

class ImpedanceControllerManager
{
public:
    ImpedanceControllerManager();

    bool setActiveController(const std::string& controller_name_);

    std::string getActiveControllerName();

    std::array<float, ImpedanceController::kNumJoints> computeTorque(const ImpedanceControllerInput& input);

private:
    std::mutex mutex_;
    std::shared_ptr<ImpedanceController> active_controller_;
};

#endif