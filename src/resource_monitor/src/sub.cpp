#include "../include/resource_monitor/sub.hpp"

namespace resource_monitor {


ResourceMonitorSubscriber::ResourceMonitorSubscriber(const rclcpp::NodeOptions & node_opts)
: Node("subscriber", node_opts)
{
  cpu_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/resource_monitor/cpu_usage", 10,
            std::bind(&ResourceMonitorSubscriber::cpuCallback, this, std::placeholders::_1)
  );

  mem_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/resource_monitor/memory_usage", 10,
            std::bind(&ResourceMonitorSubscriber::memCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Subscriber started.");
}

void ResourceMonitorSubscriber::cpuCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "CPU usage: %.2f%%", msg->data);
}

void ResourceMonitorSubscriber::memCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Memory usage: %.2f%%", msg->data);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(resource_monitor::ResourceMonitorSubscriber)