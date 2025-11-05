#pragma once

#include "impedance_controller_node.cpp"
#include "impedance_interface/prototype_impedance_controller.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<PrototypeImpedanceController>();
  auto node = std::make_shared<ImpedanceControllerNode>(controller);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
