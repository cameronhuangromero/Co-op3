#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fcntl.h>      // shm_open
#include <sys/mman.h>   // mmap
#include <unistd.h>     // close
#include <cstring>
#include <stdexcept>
#include "common/joint_state_buffer.h"

class JointStateBufferNode : public rclcpp::Node {
public:
  JointStateBufferNode()
  : Node("joint_state_buffer_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("filtered_joint_states", 10);

    // Attach to existing shared memory created by torque controller
    const char* shm_name = "/joint_state_buffer";
    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd < 0) {
      throw std::runtime_error("Failed to open shared memory object");
    }

    buffer_ = static_cast<JointStateBuffer*>(
      mmap(nullptr, sizeof(JointStateBuffer), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)
    );
    close(fd);

    if (buffer_ == MAP_FAILED) {
      throw std::runtime_error("Failed to mmap shared memory buffer");
    }

    // Publish at 100 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&JointStateBufferNode::publishJointStates, this)
    );
  }

private:
  void publishJointStates() {
    auto smoothed = buffer_->boxcarAverage(10);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name.resize(JointStateBuffer::kNumJoints);
    msg.position.resize(JointStateBuffer::kNumJoints);
    msg.velocity.resize(JointStateBuffer::kNumJoints);
    msg.effort.resize(JointStateBuffer::kNumJoints);

    for (int i = 0; i < JointStateBuffer::kNumJoints; ++i) {
      msg.name[i] = "joint_" + std::to_string(i);
      msg.position[i] = smoothed[i].q;
      msg.velocity[i] = smoothed[i].dq;
      msg.effort[i]   = smoothed[i].tauEst;
    }

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  JointStateBuffer* buffer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<JointStateBufferNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
