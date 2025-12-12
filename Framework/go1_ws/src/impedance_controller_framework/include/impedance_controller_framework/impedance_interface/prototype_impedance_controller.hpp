#pragma once

#include "impedance_controller_base.hpp"

namespace impedance {
namespace interface {
class PrototypeImpedanceController : public ImpedanceControllerBase {
public:
  PrototypeImpedanceController() = default;
  ~PrototypeImpedanceController() override = default;

  std::array<float, kNumJoints>
  computeTorques(const control::ControlContext &) override;
};
} // namespace interface
} // namespace impedance
