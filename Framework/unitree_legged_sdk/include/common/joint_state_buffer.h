// common/joint_state_buffer.h
#pragma once
#include <array>
#include <atomic>
#include <mutex>

struct JointState {
  float q;
  float dq;
  float tauEst;
};

class JointStateBuffer {
public:
  static constexpr int kRingSize = 1000;
  static constexpr int kNumJoints = 12;

  void push(const std::array<JointState, kNumJoints>& newStates) {
    int index = bufferIndex.fetch_add(1) % kRingSize;
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[index] = newStates;
  }

  std::array<JointState, kNumJoints> latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int index = (bufferIndex.load() - 1 + kRingSize) % kRingSize;
    return buffer_[index];
  }

private:
  std::array<std::array<JointState, kNumJoints>, kRingSize> buffer_{};
  mutable std::mutex mutex_;
  std::atomic<int> bufferIndex{0};
};

