#include "../../include/common/controller_manager.hpp"
#include <algorithm>
#include <mutex>

namespace impedance {
namespace common {
CtrlProcControllerManager &CtrlProcControllerManager::instance() {
  static CtrlProcControllerManager instance;
  return instance;
}

void CtrlProcControllerManager::registerController(const std::string &id,
                                                   CreatorFn ctor) {
  std::lock_guard<std::mutex> lg(mutex_);
  controllers_[id] = std::move(ctor);
}

std::unique_ptr<impedance::interface::ImpedanceControllerBase>
CtrlProcControllerManager::create(const std::string &id) const {
  std::lock_guard<std::mutex> lg(mutex_);
  auto it = controllers_.find(id);
  if (it == controllers_.end()) {
    return nullptr;
  }
  return (it->second)();
}

std::vector<std::string> CtrlProcControllerManager::listControllers() const {
  std::lock_guard<std::mutex> lg(mutex_);
  std::vector<std::string> out;
  out.reserve(controllers_.size());
  for (const auto &kv : controllers_) {
    out.push_back(kv.first);
  }
  std::sort(out.begin(), out.end());
  return out;
}
} // namespace common
} // namespace impedance
