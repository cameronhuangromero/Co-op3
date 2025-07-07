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
    // _motionTime++;

    // Smoothly increment percent
    _interpolationPercent += 1.0f / _interpolationDuration;
    if (_interpolationPercent > 1.0f) _interpolationPercent = 1.0f;

    float Kp = 10.0f;
    float Kd = 1.0f;
    float maxTorque = 5.0f;

    for (int i = 0; i < 12; ++i) {
        // Gradual desired trajectory:
        float q_des = (1 - _interpolationPercent) * _startPos[i] + _interpolationPercent * _targetPos[i];
        float dq_des = 0.0f;

        float q_err = q_des - _lowState->motorState[i].q;
        float dq_err = dq_des - _lowState->motorState[i].dq;

        float raw_torque = Kp * q_err + Kd * dq_err;
        float torque = maxTorque * std::tanh(raw_torque / maxTorque);

        _lowCmd->motorCmd[i].q = 0.0;
        _lowCmd->motorCmd[i].dq = 0.0;
        _lowCmd->motorCmd[i].Kp = 0;
        _lowCmd->motorCmd[i].Kd = 0;
        _lowCmd->motorCmd[i].tau = torque;

        RCLCPP_INFO(rclcpp::get_logger("FSM"),
            "Joint %d: q_des=%.3f, q=%.3f, torque=%.3f",
            i, q_des, _lowState->motorState[i].q, torque);
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