

// #pragma once
// #include <array>
// #include <atomic>
// #include <mutex>
// #include <vector>
// #include <algorithm>  // std::min

// struct JointState {
//   float q;
//   float dq;
//   float tauEst;
// };

// class JointStateBuffer {
// public:
//   static constexpr int kRingSize = 1000;
//   static constexpr int kNumJoints = 12;

//   void push(const std::array<JointState, kNumJoints>& newStates) {
//     int index = bufferIndex.fetch_add(1) % kRingSize;
//     std::lock_guard<std::mutex> lock(mutex_);
//     buffer_[index] = newStates;
//   }

//   std::array<JointState, kNumJoints> latest() const {
//     std::lock_guard<std::mutex> lock(mutex_);
//     int index = (bufferIndex.load() - 1 + kRingSize) % kRingSize;
//     return buffer_[index];
//   }

//   std::array<JointState, kNumJoints> boxcarAverage(int windowSize) const {
//     std::array<JointState, kNumJoints> avg{};
//     std::lock_guard<std::mutex> lock(mutex_);

//     int end = bufferIndex.load();
//     int size = std::min(windowSize, kRingSize);

//     for (int j = 0; j < kNumJoints; ++j) {
//       float sum_q = 0, sum_dq = 0, sum_tau = 0;

//       for (int i = 0; i < size; ++i) {
//         int idx = (end - 1 - i + kRingSize) % kRingSize;
//         const JointState& s = buffer_[idx][j];
//         sum_q += s.q;
//         sum_dq += s.dq;
//         sum_tau += s.tauEst;
//       }

//       avg[j].q = sum_q / size;
//       avg[j].dq = sum_dq / size;
//       avg[j].tauEst = sum_tau / size;
//     }

//     return avg;
//   }

// private:
//   std::array<std::array<JointState, kNumJoints>, kRingSize> buffer_{};
//   mutable std::mutex mutex_;
//   std::atomic<int> bufferIndex{0};
// };

///////////////////////////////////------------------------------------------/////////////////////////////////
///////////////////////////////////------------------------------------------/////////////////////////////////
///////////////////////////////////------------------------------------------/////////////////////////////////


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "std_srvs/srv/trigger.hpp"
// #include <array>
// #include <atomic>
// #include <mutex>
// #include <algorithm>

// struct JointState {
//   float q;
//   float dq;
//   float tauEst;
// };

// class JointStateBuffer {
// public:
//   static constexpr int kRingSize = 1000;
//   static constexpr int kNumJoints = 12;

//   void push(const std::array<JointState, kNumJoints>& newStates) {
//     int index = bufferIndex.fetch_add(1) % kRingSize;
//     std::lock_guard<std::mutex> lock(mutex_);
//     buffer_[index] = newStates;
//   }

//   std::array<JointState, kNumJoints> boxcarAverage(int windowSize = 20) const {
//     std::array<JointState, kNumJoints> avg{};
//     std::lock_guard<std::mutex> lock(mutex_);

//     int end = bufferIndex.load();
//     int size = std::min(windowSize, kRingSize);

//     for (int j = 0; j < kNumJoints; ++j) {
//       float sum_q = 0, sum_dq = 0, sum_tau = 0;
//       for (int i = 0; i < size; ++i) {
//         int idx = (end - 1 - i + kRingSize) % kRingSize;
//         const JointState& s = buffer_[idx][j];
//         sum_q += s.q;
//         sum_dq += s.dq;
//         sum_tau += s.tauEst;
//       }
//       avg[j].q = sum_q / size;
//       avg[j].dq = sum_dq / size;
//       avg[j].tauEst = sum_tau / size;
//     }
//     return avg;
//   }

// private:
//   std::array<std::array<JointState, kNumJoints>, kRingSize> buffer_{};
//   mutable std::mutex mutex_;
//   std::atomic<int> bufferIndex{0};
// };

// class JointStateBufferNode : public rclcpp::Node {
// public:
//   JointStateBufferNode() : Node("joint_state_buffer_node") {
//     publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
//         "/filtered_joint_state", 10);

//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(2),
//         std::bind(&JointStateBufferNode::publishFilteredJointState, this));
//   }

//   void pushData(const std::array<JointState, 12>& state) {
//     buffer_.push(state);
//   }

// private:
//   void publishFilteredJointState() {
//     auto avg = buffer_.boxcarAverage();

//     auto msg = sensor_msgs::msg::JointState();
//     msg.header.stamp = now();
//     msg.name.resize(12);
//     msg.position.resize(12);
//     msg.velocity.resize(12);
//     msg.effort.resize(12);

//     for (int i = 0; i < 12; ++i) {
//       msg.name[i] = "joint_" + std::to_string(i);
//       msg.position[i] = avg[i].q;
//       msg.velocity[i] = avg[i].dq;
//       msg.effort[i] = avg[i].tauEst;
//     }

//     publisher_->publish(msg);
//   }

//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   JointStateBuffer buffer_;
// };

///////////////////////////////////------------------------------------------/////////////////////////////////
///////////////////////////////////------------------------------------------/////////////////////////////////
///////////////////////////////////------------------------------------------/////////////////////////////////

#pragma once
#include <pthread.h>
#include <array>
#include <cstddef>
#include <atomic>

struct JointState {
    float q;
    float dq;
    float tauEst;
};

struct JointStateBuffer {
    static constexpr int kRingSize = 1000;
    static constexpr int kNumJoints = 12;

    // Synchronization primitive for multi-process access
    pthread_mutex_t mutex;
    int bufferIndex;

    // Fixed-size buffer: [time sample][joint]
    JointState buffer[kRingSize][kNumJoints];

    // Init function to be called once after shm allocation
    void init() {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&mutex, &attr);
        bufferIndex = 0;
    }

    void push(const std::array<JointState, kNumJoints> &newStates) {
        int index = __atomic_fetch_add(&bufferIndex, 1, __ATOMIC_SEQ_CST) % kRingSize;
        pthread_mutex_lock(&mutex);
        for (int j = 0; j < kNumJoints; ++j) {
            buffer[index][j] = newStates[j];
        }
        pthread_mutex_unlock(&mutex);
    }

    std::array<JointState, kNumJoints> boxcarAverage(int windowSize = 20) const {
        std::array<JointState, kNumJoints> avg{};
        pthread_mutex_lock((pthread_mutex_t*)&mutex);

        int end = bufferIndex;
        int size = (windowSize < kRingSize) ? windowSize : kRingSize;

        for (int j = 0; j < kNumJoints; ++j) {
            float sum_q = 0, sum_dq = 0, sum_tau = 0;
            for (int i = 0; i < size; ++i) {
                int idx = (end - 1 - i + kRingSize) % kRingSize;
                const JointState &s = buffer[idx][j];
                sum_q += s.q;
                sum_dq += s.dq;
                sum_tau += s.tauEst;
            }
            avg[j].q = sum_q / size;
            avg[j].dq = sum_dq / size;
            avg[j].tauEst = sum_tau / size;
        }

        pthread_mutex_unlock((pthread_mutex_t*)&mutex);
        return avg;
    }
};


struct TorqueCommandBuffer {
    static constexpr int kNumJoints = 12;
    std::array<std::atomic<float>, kNumJoints> torques;
    std::array<std::atomic<bool>, kNumJoints> enabled;

    void init() {
        for (int i = 0; i < kNumJoints; i++) {
            torques[i].store(0.0f);
            enabled[i].store(false);
        }
    }
};


