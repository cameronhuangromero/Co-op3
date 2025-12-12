#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "../../include/common/controller_manager.hpp"
#include "../../include/control_interface/control_context.hpp"
#include "../../include/impedance_interface/prototype_impedance_controller.hpp"

int main() {
  using namespace impedance;
  using namespace impedance::common;
  using namespace impedance::control;
  using namespace impedance::interface;

  ControlContext context;
  for (std::size_t i = 0; i < context.q.size(); i++) {
    context.q[i] = static_cast<float>(i) + 1.0f;
    context.qd[i] = 0.1f * static_cast<float>(i);
  }
  context.stamp = std::chrono::steady_clock::now();

  auto list = CtrlProcControllerManager::instance().listControllers();
  std::cout << "Registered Controllers:\n";
  for (const auto &id : list) {
    std::cout << " - " << id << "\n";
  }

  auto controller = CtrlProcControllerManager::instance().create("prototype");
  if (!controller) {
    std::cerr << "Failed to create prototype controller\n";
    return 2;
  }

  auto torques = controller->computeTorques(context);

  bool all_zero = true;
  for (auto t : torques) {
    if (std::fabs(t) > 1e-6f) {
      all_zero = false;
      break;
    }
  }

  if (all_zero) {
    std::cout << "Prototype controller produced zero torques - OK\n";
    return 0;
  } else {
    std::cerr << "Prototype controller produced non-zero torques - FAIL\n";
    std::cerr << "Torques:\n";
    for (auto t : torques)
      std::cerr << t << " ";
    std::cerr << "\n";
    return 3;
  }
}
