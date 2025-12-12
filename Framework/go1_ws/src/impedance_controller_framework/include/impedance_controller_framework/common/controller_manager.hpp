#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "../impedance_interface/impedance_controller_base.hpp"

/*
 * Author: Cameron Romero
 *
 * thread-safe singleton manager for impedance controllers
 * provides register/create/list capabilities
 */

namespace impedance {
namespace common {
/*
 * manager to register and instantiate impedance controllers by ID
 *
 * thread-safe for register/create operations using internal mutex
 */
class CtrlProcControllerManager {
public:
  using CreatorFn = std::function<
      std::unique_ptr<impedance::interface::ImpedanceControllerBase>()>;

  static CtrlProcControllerManager &instance(); // obtain singleton instance

  // register controller factory
  // if ID already exists, it will be rewritten
  void registerController(const std::string &id, CreatorFn ctor);

  // create new instance of the registered controller
  // returns null ptr if ID isn't found
  std::unique_ptr<impedance::interface::ImpedanceControllerBase>
  create(const std::string &id) const;

  std::vector<std::string>
  listControllers() const; // list registered controller IDs

private:
  CtrlProcControllerManager() = default;
  ~CtrlProcControllerManager() = default;

  // non-copyable
  CtrlProcControllerManager(const CtrlProcControllerManager &) = delete;
  CtrlProcControllerManager &
  operator=(const CtrlProcControllerManager &) = delete;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, CreatorFn> controllers_;
};
} // namespace common
} // namespace impedance
