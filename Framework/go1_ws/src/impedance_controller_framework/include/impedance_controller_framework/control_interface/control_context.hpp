#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <vector>

/*
 * Author: Cameron Romero
 *
 * Small, dependency-light control context definition
 *
 * - kNumJoints == 12 should map to the GO1 12 actuated joints (4 legs * 3
 * joints)
 * - CartesianTask::wrench is an optional 6D force/torque (fx, fy, fz, tx, ty,
 * tz)
 */

namespace impedance {
namespace control {
// forward declare the constant to keep the mapping explicit
inline constexpr std::size_t kNumJoints = 12u;

/*
 * representation of a cartesian task for one leg
 */
struct CartesianTask {
  int leg_id = -1;
  std::array<float, 3> x_des{};  // desired position
  std::array<float, 3> dx_des{}; // desired velocity

  std::optional<std::array<float, 6>>
      wrench; // optional desired wrench if needed
};

/*
 * holds the robot state and optional references used by the controllers
 *
 * should keep the struct small and header-only to simplify unit tests and
 * compilation
 */
struct ControlContext {
  std::array<float, kNumJoints> q{};  // joint positions
  std::array<float, kNumJoints> qd{}; // joint velocities

  // optional desired joint references (e.g. if controller is given joint space
  // targets)
  std::optional<std::array<float, kNumJoints>> q_des;
  std::optional<std::array<float, kNumJoints>> qd_des;

  // task space references (>=0)
  std::vector<CartesianTask> cartesian_tasks;

  // timestamp for this context
  std::chrono::steady_clock::time_point stamp =
      std::chrono::steady_clock::now();
};
} // namespace control
} // namespace impedance
