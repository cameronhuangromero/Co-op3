#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

ImpedanceController::ImpedanceController()
: Node("impedance_controller")
{
    
}