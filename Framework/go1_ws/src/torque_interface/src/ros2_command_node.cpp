// not fully setup, can delete if want

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include "common/joint_state_buffer.h"

class TorqueCommandNode : public rclcpp::Node {
public:
  TorqueCommandNode()
  : Node("torque_command_node")
  {
    // Attach to existing torque command buffer
    const char* shm_cmd_name = "/torque_command_buffer";
    int fd = shm_open(shm_cmd_name, O_RDWR, 0666);
    if (fd < 0) {
      throw std::runtime_error("Failed to open torque command shm");
    }

    cmd_buffer_ = static_cast<TorqueCommandBuffer*>(
      mmap(nullptr, sizeof(TorqueCommandBuffer),
           PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)
    );
    close(fd);

    if (cmd_buffer_ == MAP_FAILED) {
      throw std::runtime_error("Failed to mmap torque command buffer");
    }

    // Subscribe to ROS 2 topic with desired torques
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "desired_torques", 10,
      std::bind(&TorqueCommandNode::commandCallback, this, std::placeholders::_1)
    );
  }

private:
  void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != TorqueCommandBuffer::kNumJoints) {
      RCLCPP_ERROR(this->get_logger(), "Expected 12 torques, got %zu", msg->data.size());
      return;
    }

    for (int i = 0; i < TorqueCommandBuffer::kNumJoints; i++) {
      cmd_buffer_->torques[i].store(msg->data[i]);
      cmd_buffer_->enabled[i].store(true);
    }
    RCLCPP_INFO(this->get_logger(), "Updated torque commands");
  }

  TorqueCommandBuffer* cmd_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TorqueCommandNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
