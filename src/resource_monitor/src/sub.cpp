#include "../include/resource_monitor/sub.hpp"

ResourceMonitorSubscriber::ResourceMonitorSubscriber()
: Node("subscriber")
{
  cpu_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/resource_monitor/cpu_usage", 10,
            std::bind(&ResourceMonitorSubscriber::cpuCallback, this, std::placeholders::_1)
  );

  mem_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/resource_monitor/memory_usage", 10,
            std::bind(&ResourceMonitorSubscriber::memCallback, this, std::placeholders::_1)
  );
}

void ResourceMonitorSubscriber::cpuCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "CPU usage: %.2f%%", msg->data);
}

void ResourceMonitorSubscriber::memCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Memory usage: %.2f%%", msg->data);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResourceMonitorSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
