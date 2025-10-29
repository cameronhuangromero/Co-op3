#pragma once
#include <array>

/*
 * Author: Cameron Romero
 *
 * Abstract base class for impedance controllers used with the ROS2 framework
 * for the Unitree GO1
 *
 * This class defines a common interface for impedance control modules that
 * compute joint torques based on system state information like positions and
 * velocities.
 *
 * The design allows for multiple impedance control modules to be stacked upon
 * one another within a modular control framework
 *
 * This class does not assume a control space. It simply provides a torque
 * computation interface that can be specialized during implementation of
 * subclasses.
 */

class ImpedanceControllerBase {
public:
  // Unitree Go1 has 4 legs, each with 3 joints
  static constexpr int kNumJoints = 12;

  virtual ~ImpedanceControllerBase() = default;

  // Compute torques based on inputs (could be joint positions, Cartesian
  // coordinates, etc.)
  //
  // Takes in an array of the current joint positions, q, and the current joint
  // velocities, qd.
  //
  // Outputs an array of torques that need to be applied to each
  // respective joint.
  virtual std::array<float, kNumJoints>
  computeTorques(const std::array<float, kNumJoints> &q,
                 const std::array<float, kNumJoints> &qd) = 0;
};
