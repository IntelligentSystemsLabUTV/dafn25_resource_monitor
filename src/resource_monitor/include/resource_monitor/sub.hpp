#ifndef SUB_HPP
#define SUB_HPP

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#include <std_msgs/msg/string.hpp> //! Interface library that we'll use
#include <std_msgs/msg/float64.hpp>

namespace resource_monitor {


class ResourceMonitorSubscriber : public rclcpp::Node {
public:
    ResourceMonitorSubscriber(const rclcpp::NodeOptions & node_opts);

private:
    void cpuCallback(const std_msgs::msg::Float64::SharedPtr msg);

    void memCallback(const std_msgs::msg::Float64::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cpu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mem_sub_;
};

}

#endif