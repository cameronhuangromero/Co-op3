#pragma once
#include "ImpedanceControllerInterface.hpp"
#include <vector>
#include <map>
#include <string>

class PIDController : public ImpedanceControllerInterface {
    public: 
        PIDController(const std::vector<double> &Kp, const std::vector<double> &Ki, const std::vector<double> &Kd);

        std::vector<double> computeTorque(
            const std::vector<double> &desired_position,
            const std::vector<double> &desired_velocity,
            const std::vector<double> &measured_position,
            const std::vector<double> &measured_velocity
        ) override;

        void setParameters(const std::map<std::string, std::vector<double>> &parameters) override;

        private:
            std::vector<double> Kp_;
            std::vector<double> Ki_;
            std::vector<double> Kd_;
            std::vector<double> integral_error_;
};