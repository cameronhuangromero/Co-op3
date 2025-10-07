#include "impedance_interface/pid_impedance_controller.hpp"
#include <iostream>
#include <vector>
#include <map>

int main(){
    std::vector<double> desired_position = {1.0, 0.5, -0.2};
    std::vector<double> desired_velocity = {0.0, 0.0, 0.0};
    std::vector<double> measured_position = {0.8, 0.3, -0.1};
    std::vector<double> measured_velocity = {0.05, -0.02, 0.01};

    std::vector<double> Kp = {60.0, 50.0, 40.0};
    std::vector<double> Ki = {0.1, 0.1, 0.1};
    std::vector<double> Kd = {5.0, 4.0, 3.0};

    PIDController pid(Kp, Ki, Kd);

    std::vector<double> torque = pid.computeTorque(
        desired_position, desired_velocity,
        measured_position, measured_velocity
    );

    std::cout << "PID Controller Torque:" << std::endl;
    for (size_t i = 0; i < torque.size(); ++i){
        std::cout << "Joint " << i << ": " << torque[i] << std::endl;
    }

    return 0;
}