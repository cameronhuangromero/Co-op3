SpringDamperController::SpringDamperController(const std::vector<double> &stiffness, const std::vector<double> &damping)
    : stiffness_(stiffness), damping_(damping){}

std::vector<double> SpringDamperController::computeTorque(
    const std::vector<double> &desired_position,
    const std::vector<double> &desired_velocity,
    const std::vector<double> &measured_position,
    const std::vector<double> &measured_velocity)
{
    std::vector<double> torque(desired_position.size(), 0.0);
    for (size_t i = 0; i < desired_position.size(); ++i){
        torque[i] = stiffness_[i] * (desired_position[i] - measured_position[i]) + damping_[i] * (desired_velocity[i] - measured_velocity[i]);
    }
    return torque;
}

void SpringDamperController::setParameters(const std::map<std::string, std::vector<double>> &parameters)
{
    if (parameters.count("stiffness")) stiffness_ = parameters.at("stiffness");
    if (parameters.count("damping")) damping_ = parameters.at("damping");
}