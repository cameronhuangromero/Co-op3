#pragma once
#include "impedance_interface/impedance_controller_base.hpp"
#include <array>

/*
 * Author: Cameron Romero
 *
 * Defines a simple prototype implementation of an impedance controller.
 *
 * This class provides a basic example of an impedance controller derived from
 * ImpedanceControllerBase.
 *
 * It serves as a blueprint or testing implementation that outputs zero torques
 * by default, allowing verification of the control pipeline without applying
 * actual forces to the robot.
 */

class PrototypeImpedanceController : public ImpedanceControllerBase {
public:
  std::array<float, kNumJoints>
  computeTorques(const std::array<float, kNumJoints> &q,
                 const std::array<float, kNumJoints> &qd) override {
    std::array<float, kNumJoints> tau{};

    for (int i = 0; i < kNumJoints; ++i) {
      tau[i] = 0.0f;
    }

    return tau;
  }
};
