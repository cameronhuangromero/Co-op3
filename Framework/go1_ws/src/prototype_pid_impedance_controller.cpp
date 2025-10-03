PIDController::PIDController(const std::vector<double> &Kp, const std::vector<double> &Ki, const std::vector<double> &Kd): Kp_(Kp), Ki_(Ki), Kd_(Kd), integral_error_(Kp.size(), 0.0) {}

std::vector<double> PIDController::computeTorque(
    const std::vector<double> &desired_position,
    const std::vector<double> &desired_velocity,
    const std::vector<double> &measured_position,
    const std::vector<double> &measured_velocity)
{
    std::vector<double> torque(desired_position.size(), 0.0);
    for(size_t i = 0; i < desired_position.size(); ++i){
        double error = desired_position[i] - measured_position[i];
        integral_error_[i] += error;
        double derivative = desired_velocity[i] - measured_velocity[i];

        torque[i] = Kp_[i] * error + Ki_[i] * integral_error_[i] + Kd_[i] * derivative;
    }
    return torque;
}

void PIDController::setParameters(const std::map<std::string, std::vector<double>> &parameters){
    if (parameters.count("Kp")) Kp_ = parameters.at("Kp");
    if (parameters.count("Ki")) Ki_ = parameters.at("Ki");
    if (parameters.count("Kd")) Kd_ = parameters.at("Kd");
}