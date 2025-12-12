#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include "../control_interface/control_context.hpp"

/*
 * Author: Cameron Romero
 *
 * Base class for impedance controllers
 *
 * - computeTorques(const ControlContext& context) -> array<float, kNumJoints)
 * - thread-safe enabled flag using std::atomic<bool>
 * - priority integer with simple getter/setter (not atomic and protected by
 * mutex if needed by caller)
 */

namespace impedance {
namespace interface {
class ImpedanceControllerBase {
public:
  static constexpr std::size_t kNumJoints =
      control::kNumJoints; // number of joints for current platform (e.g.
                           // unitree go1)

  // non-copyable and non-movable for simplicity sake
  ImpedanceControllerBase() = default;
  virtual ~ImpedanceControllerBase() = default;

  ImpedanceControllerBase(const ImpedanceControllerBase &) = delete;
  ImpedanceControllerBase &operator=(const ImpedanceControllerBase &) = delete;

  /*
   * compute desired torques given the curent control context
   *
   * returns an array of length kNumJoints
   */
  virtual std::array<float, kNumJoints>
  computeTorques(const control::ControlContext &context) = 0;

  // enabled flag for thread-safe, defaulted to true
  void setEnabled(bool e) noexcept {
    enabled_.store(e, std::memory_order_relaxed);
  }
  bool isEnabled() const noexcept {
    return enabled_.load(std::memory_order_relaxed);
  }

  // the priority
  void setPriority(int p) {
    std::lock_guard<std::mutex> lg(priority_mutex_);
    priority_ = p;
  }

  int getPriority() const {
    std::lock_guard<std::mutex> lg(priority_mutex_);
    return priority_;
  }

  // optional human-readable ID for debugging
  void setID(const std::string &id) {
    std::lock_guard<std::mutex> lg(id_mutex_);
    id_ = id;
  }

  std::string getID() const {
    std::lock_guard<std::mutex> lg(id_mutex_);
    return id_;
  }

protected:
  std::atomic<bool> enabled_{
      true}; // controller enabled flag (uses atomic for cheap thread-safety)

private:
  // priority protected by mutex for thread-safe access
  mutable std::mutex priority_mutex_;
  int priority_{0};

  // again, optional human-readable ID
  mutable std::mutex id_mutex_;
  std::string id_{"unnamed controller"};
};
} // namespace interface
} // namespace impedance
