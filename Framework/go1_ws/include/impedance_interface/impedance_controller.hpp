#pragma once
#include <vector>
#include <map>
#include <string>

class ImpedanceControllerInterace {
public:
    virtual ~ImpedanceControllerInterface() = default;

    virtual std::vector<double> computeTorque(
        const std::vector<double> &desired_position,
        const std::vector<double> &desired_velocity,
        const std::vector<double> &measured_position,
        const std::vector<double> &measured_velocity
    ) = 0;

    virtual void setParameters(const std::map<std::string, std::vector<double>> &parameters){}
};