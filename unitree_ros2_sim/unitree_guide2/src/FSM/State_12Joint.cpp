#include <iostream>
#include "FSM/State_12Joint.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>  // Needed for tanh

State_12Joint::State_12Joint(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::ALL_TORQUE_JOINT, "12Joint")
                // _active(false)
                {}


void State_12Joint::enter(){
    RCLCPP_INFO(rclcpp::get_logger("FSM"), "Entering ALL_TORQUE_JOINT state.");
    // _motionTime = 0;
     _interpolationPercent = 0.0f;
    for (int i = 0; i < 12; ++i) {
        _startPos[i] = _lowState->motorState[i].q;
        _targetPos[i] = 0.0f;  // or set this based on your actual target config
    }
}


void State_12Joint::run() {
    _interpolationPercent += 1.0f / _interpolationDuration;
    if (_interpolationPercent > 1.0f) _interpolationPercent = 1.0f;

    float Kp = 10.0f;
    float Kd = 1.0f;
    float maxTorque = 5.0f;

    for (int i = 0; i < 12; ++i) {
        // Interpolate desired position
        // float q_des = (1 - _interpolationPercent) * _startPos[i] + _interpolationPercent * _targetPos[i];
       

        // float error = fabs(torque - _lowState->motorState[i].tauEst);

            float torque = _lastTargetTorques[i];  // Default to last torque value

        if (!_jointWaiting[i]) {

             float q_des = 0.0f;
            float dq_des = 0.0f;

            float q_err = q_des - _lowState->motorState[i].q;
            float dq_err = dq_des - _lowState->motorState[i].dq;

            float raw_torque = Kp * q_err + Kd * dq_err;
            float torque = maxTorque * std::tanh(raw_torque / maxTorque);

            if (torque > maxTorque) torque = maxTorque;
            if (torque < -maxTorque) torque = -maxTorque;
            // Start waiting for this joint to reach
            _lastTargetTorques[i] = torque;
            _jointWaiting[i] = true;
        }

        

        // Always send the last commanded torque until goal is reached
        _lowCmd->motorCmd[i].q = 0.0;
        _lowCmd->motorCmd[i].dq = 0.0;
        _lowCmd->motorCmd[i].Kp = 0;
        _lowCmd->motorCmd[i].Kd = 0;
        _lowCmd->motorCmd[i].tau = _lastTargetTorques[i];

        float error = fabs(_lastTargetTorques[i] - _lowState->motorState[i].tauEst);


        // Check if the joint has reached the torque goal
        if (_jointWaiting[i] && error < 0.03f) {
            _jointWaiting[i] = false;  // Reset for next cycle if needed
        }

        RCLCPP_INFO(rclcpp::get_logger("FSM"),
            "Joint %d: cmd=%.3f, tau=%.3f, torque=%.3f, error=%.4f, holding=%d",
            i, torque, _lowState->motorState[i].tauEst, _lastTargetTorques[i], error, _jointWaiting[i]);
    }
}


void State_12Joint::exit() {
    RCLCPP_INFO(rclcpp::get_logger("FSM"), "Exiting ALL_TORQUE_JOINT state.");
}

FSMStateName State_12Joint::checkChange(){
    if (_motionTime > 50 && _lowState->userCmd == UserCommand::L2_B) {
        // std::cout << "Current userCmd: " << static_cast<int>(_lowState->userCmd)
        //   << ", stateName: " << _stateNameString << std::endl;


        return FSMStateName::ALL_TORQUE_JOINT;

}
}