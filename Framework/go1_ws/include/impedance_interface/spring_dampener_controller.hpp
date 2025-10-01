#pragma once
#include "ImpedanceControllerInterface.hpp"
#include <vector>
#include <map>
#include <string>

class SpringDamperController : public ImpedanceControllerInterface{
    public:
        SpringDamperController(const std::vector<double> &stiffness, const std::vector<double> &damping);

        std::vector<double> computeTorque(
            const std::vector<double> &desired_position,
            const std::vector<double> &desired_velocity,
            const std::vector<double> &measured_position,
            const std::vector<double> &measured_velocity
        ) override;

        void setParameters(const std::map<std::string, std::vector<double>> &parameters) override;

    private:
        std::vector<double> stiffness_;
        std::vector<double> damping_;
};