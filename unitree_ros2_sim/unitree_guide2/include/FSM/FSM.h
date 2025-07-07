/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#include "FSM/State_ManualJointControl.h"
#include "FSM/State_12Joint.h"
#include <rclcpp/rclcpp.hpp>
#include <thread>


#ifdef COMPILE_WITH_MOVE_BASE
    #include "FSM/State_move_base.h"
#endif  // COMPILE_WITH_MOVE_BASE

#ifdef COMPILE_WITH_ROS2_MB
    #include "FSM/State_move_base.h"
#endif  // COMPILE_WITH_ROS2_MB

struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_FreeStand *freeStand;
    State_Trotting *trotting;
    State_BalanceTest *balanceTest;
    State_SwingTest *swingTest;
    State_StepTest *stepTest;
    
    State_ManualJointControl *manualJoint;  // your new state
    State_12Joint *AllJoint;

#ifdef COMPILE_WITH_MOVE_BASE
    State_move_base *moveBase;
#endif  // COMPILE_WITH_MOVE_BASE

#ifdef COMPILE_WITH_ROS2_MB
    State_move_base *moveBase;
#endif  // COMPILE_WITH_ROS2_MB

    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete balanceTest;
        delete swingTest;
        delete stepTest;
        delete manualJoint;
        delete AllJoint;

#ifdef COMPILE_WITH_MOVE_BASE
        delete moveBase;
#endif  // COMPILE_WITH_MOVE_BASE
#ifdef COMPILE_WITH_ROS2_MB
        delete moveBase;
#endif  // COMPILE_WITH_ROS2_MB
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::thread _rosSpinThread;
};


#endif  // FSM_H