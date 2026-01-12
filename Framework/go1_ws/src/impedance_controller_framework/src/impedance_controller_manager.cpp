#include "impedance_controller_manager.hpp"
#include "impedance_controller_factory.hpp"

/*
    @brief Implementation of ImpedanceControllerManager managing a single active controller

    Author: Cameron Romero
    Date: 1/12/26
*/

ImpedanceControllerManager::ImpedanceControllerManager() : active_controller_(nullptr) {}

bool ImpedanceControllerManager::setActiveController(const std::string& controller_name)
{
    auto controller = ImpedanceControllerFactory::create(controller_name);
    if (!controller)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    active_controller_ = controller;
    return true;
}

std::string ImpedanceControllerManager::getActiveControllerName()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return active_controller_ ? active_controller_->name() : "";
}

std::array<float, ImpedanceController::kNumJoints> ImpedanceControllerManager::computeTorque(const ImpedanceControllerInput& input)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_controller_)
    {
        std::array<float, ImpedanceController::kNumJoints> zeros{};
        zeros.fill(0.0f);
        return zeros;
    }
    return active_controller_->computeTorque(input);
}