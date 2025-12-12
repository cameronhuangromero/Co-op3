#include "../../include/impedance_interface/prototype_impedance_controller.hpp"
#include "../../include/common/controller_manager.hpp"

#include <array>
#include <memory>

using namespace impedance::interface;
using impedance::common::CtrlProcControllerManager;

std::array<float, PrototypeImpedanceController::kNumJoints>
PrototypeImpedanceController::computeTorques(const control::ControlContext &) {
  std::array<float, kNumJoints> zeros{};
  zeros.fill(0.0f);
  return zeros;
}

static bool registeredController = []() -> bool {
  CtrlProcControllerManager::instance().registerController(
      "prototype", []() -> std::unique_ptr<ImpedanceControllerBase> {
        auto p = std::make_unique<PrototypeImpedanceController>();
        p->setID("prototype");
        return p;
      });
  return true;
}();
