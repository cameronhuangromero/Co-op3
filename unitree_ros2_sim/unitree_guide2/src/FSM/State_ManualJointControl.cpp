#include <iostream>
#include "FSM/State_ManualJointControl.h"
#include "interface/IOROS.h"
#include "rclcpp/rclcpp.hpp"
#include "common/unitreeLeg.h"
#include "unitree_legged_sdk/comm.h"

using namespace UNITREE_LEGGED_SDK;

State_ManualJointControl::State_ManualJointControl(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::MANUAL_JOINT_CONTROL, "manual_joint_control"),
      legFR(0, Vec3(0.188, -0.047, 0)) { // Initialize legFR here
    
    // Initialize ROS node
    _node = rclcpp::Node::make_shared("manual_joint_control_node");

    // Executor for spinning ROS node
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(_node);
    std::thread([executor]() {
        executor->spin();
    }).detach();

    RCLCPP_INFO(_node->get_logger(), "Initialized MANUAL_JOINT_CONTROL state node.");
}

void State_ManualJointControl::enter() {
    _motionTime = 0;
    RCLCPP_INFO(rclcpp::get_logger("FSM"), "Entering MANUAL_JOINT_CONTROL state.");
    
    for (int i = 0; i < 12; ++i) {
        initial_position[i] = _lowState->motorState[i].q;  // Read current joint positions
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FSM"), "Initial positions:");
    for (int i = 0; i < 12; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("FSM"), "Joint[%d]: %f", i, initial_position[i]);
    }

}

void State_ManualJointControl::run() {
    
    _motionTime++;

    // Vec3 desFootFR(0.2, -0.1, -0.25);  // Desired foot position
    Vec3 desFootFR(2, -1, 0);
    Vec3 qFR = legFR.calcQ(desFootFR, FrameType::BODY);  // Inverse kinematics

    // Populate LowlevelCmd structure
    // LowlevelCmd cmd;
    _lowCmd->motorCmd[0].q = qFR(0);
    _lowCmd->motorCmd[1].q = qFR(1);
    _lowCmd->motorCmd[2].q = qFR(2);

    _lowCmd->motorCmd[0].mode = 0x0A;  // Assuming position control
    _lowCmd->motorCmd[1].mode = 0x0A;
    _lowCmd->motorCmd[2].mode = 0x0A;

    // for (int i = 3; i < 12; ++i) { // Assuming TOTAL_JOINTS is 12 for quadruped
    //     cmd.motorCmd[i].mode = 0x00;  // Passive mode
    //     cmd.motorCmd[i].q = 0.0;
    //     cmd.motorCmd[i].dq = 0.0;
    //     cmd.motorCmd[i].tau = 0.0;
    // }

    for (int i = 3; i < 12; ++i) {
    _lowCmd->motorCmd[i].mode = 0x0A;  // Position control mode
    _lowCmd->motorCmd[i].q = initial_position[i];  // Lock to current position
    _lowCmd->motorCmd[i].dq = 0.0;
    _lowCmd->motorCmd[i].tau = 0.0;

//     for (int i = 0; i < 12; ++i) {
//     RCLCPP_INFO(rclcpp::get_logger("FSM"), "Cmd Joint[%d]: mode=%d, q=%f, dq=%f, tau=%f",
//                 i, _lowCmd->motorCmd[i].mode, _lowCmd->motorCmd[i].q, _lowCmd->motorCmd[i].dq, _lowCmd->motorCmd[i].tau);
// }
}


    // auto ioRos = dynamic_cast<IOROS *>(_ctrlComp->ioInter);
    // if (ioRos) {
    //     ioRos->sendCmd(&cmd);  // Send the command via IOROS
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("FSM"), "IOROS interface not available!");
    // }
}

void State_ManualJointControl::exit() {
    RCLCPP_INFO(rclcpp::get_logger("FSM"), "Exiting MANUAL_JOINT_CONTROL state.");
}

FSMStateName State_ManualJointControl::checkChange(){
    if (_motionTime > 50 && _lowState->userCmd == UserCommand::L2_B) {
        // std::cout << "Current userCmd: " << static_cast<int>(_lowState->userCmd)
        //   << ", stateName: " << _stateNameString << std::endl;


        return FSMStateName::MANUAL_JOINT_CONTROL;

}
}