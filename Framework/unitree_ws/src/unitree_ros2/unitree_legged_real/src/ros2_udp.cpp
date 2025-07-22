#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <array>
#include <chrono>


class MicrosecondFrequencyTracker {
public:
    static constexpr size_t WINDOW_SIZE = 100;

    void tick() {
        using clock = std::chrono::steady_clock;
        auto now = clock::now();

        if (init) {
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
            buffer[index] = dt;
            index = (index + 1) % WINDOW_SIZE;
            count = std::min(count + 1, WINDOW_SIZE);
        } else {
            init = true;
        }

        last_time = now;
    }

    double get_frequency_hz() const {
        if (count < 2) return 0.0;

        long total_us = 0;
        for (size_t i = 0; i < count; ++i)
            total_us += buffer[i];

        double avg_us = static_cast<double>(total_us) / count;
        return avg_us > 0.0 ? 1e6 / avg_us : 0.0;
    }

private:
    std::array<long, WINDOW_SIZE> buffer{};
    size_t index = 0;
    size_t count = 0;
    bool init = false;
    std::chrono::steady_clock::time_point last_time;
};
// --------------------------------------------------------------------
using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};
    MicrosecondFrequencyTracker lowcmd_tracker;
    MicrosecondFrequencyTracker highcmd_tracker;

    void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg);
    void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg);

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }
};

Custom custom;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.highcmd_tracker.tick();  // Track time for frequency

    custom.high_cmd = rosMsg2Cmd(msg);

    custom.high_udp.SetSend(custom.high_cmd);
    custom.high_udp.Send();

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    custom.high_udp.Recv();
    custom.high_udp.GetRecv(custom.high_state);

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    double high_freq = custom.highcmd_tracker.get_frequency_hz();
    printf("HighCmd callback frequency: %.2f Hz\n", high_freq);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{
    custom.lowcmd_tracker.tick();  // Track time for frequency

    // printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    custom.low_udp.SetSend(custom.low_cmd);
    custom.low_udp.Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    custom.low_udp.Recv();
    custom.low_udp.GetRecv(custom.low_state);

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low->publish(low_state_ros);

    double low_freq = custom.lowcmd_tracker.get_frequency_hz();
    printf("LowCmd callback frequency: %.2f Hz\n", low_freq);

    // printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
    low_count++;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_ros2_udp");

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        printf("low level running!\n");

        pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);

        auto timer = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [node]() {
            double freq = custom.lowcmd_tracker.get_frequency_hz();
            RCLCPP_INFO(node->get_logger(), "LowCmd callback avg frequency: %.2f Hz", freq);
        });
        
        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level running!\n");

        pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
        sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}
