#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "FSM/FSMState.h"
#include "rclcpp/rclcpp.hpp"

class State_ManualJointControl : public FSMState {
public:
    State_ManualJointControl(CtrlComponents *ctrlComp);
    FSMStateName checkChange() override;
    void enter() override;
    void run() override;
    void exit() override;

    std::shared_ptr<rclcpp::Node> getNode() const { return _node; }
    // QuadrupedLeg legFR; // Add this line
    int _motionTime = 0;
    double initial_position[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:
    void motorCmdCallback(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);
    
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorCmd>::SharedPtr _motorCmdSub;
    ros2_unitree_legged_msgs::msg::MotorCmd _cmd;
    bool _hasCmd = false;
    rclcpp::Node::SharedPtr _node;

    Go1Leg legFR;
};
