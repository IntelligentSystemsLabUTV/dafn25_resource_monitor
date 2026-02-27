#include "../include/resource_monitor/pub.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace resource_monitor {

ResourceMonitorPublisher::ResourceMonitorPublisher(const rclcpp::NodeOptions & node_opts)
: Node("resource_monitor", node_opts)
{
        // Parametri
  this->declare_parameter<bool>("autostart", true);
  this->declare_parameter<int>("sampling.period", 1000);

  autostart_ = this->get_parameter("autostart").as_bool();
  sampling_period_ = this->get_parameter("sampling.period").as_int();

        // Publisher
  cpu_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/cpu_usage", 10);
  mem_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/memory_usage", 10);

        // Service enable/disable
  service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/enable_monitor",
            std::bind(&ResourceMonitorPublisher::enableCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
  );

        // Timer
  if (autostart_) {
    startMonitoring();
  }
}

    // === Service enable/disable ===
void ResourceMonitorPublisher::enableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    startMonitoring();
    response->success = true;
    response->message = "Monitoring enabled";
  } else {
    stopMonitoring();
    response->success = true;
    response->message = "Monitoring disabled";
  }
}

    // === Avvia il timer ===
void ResourceMonitorPublisher::startMonitoring()
{
  if (!timer_) {
    timer_ = this->create_wall_timer(
                std::chrono::milliseconds(sampling_period_),
                std::bind(&ResourceMonitorPublisher::timerCallback, this)
    );
    RCLCPP_INFO(this->get_logger(), "Resource monitoring started.");
  }
}

    // === Ferma il timer ===
void ResourceMonitorPublisher::stopMonitoring()
{
  timer_.reset();
  RCLCPP_INFO(this->get_logger(), "Resource monitoring stopped.");
}

    // === Callback del timer ===
void ResourceMonitorPublisher::timerCallback()
{
  double cpu_usage = getCpuUsage();
  double mem_usage = getMemoryUsage();

  auto cpu_msg = std_msgs::msg::Float64();
  cpu_msg.data = cpu_usage;
  cpu_pub_->publish(cpu_msg);

  auto mem_msg = std_msgs::msg::Float64();
  mem_msg.data = mem_usage;
  mem_pub_->publish(mem_msg);

  // RCLCPP_INFO(this->get_logger(), "CPU: %.2f%%, MEM: %.2f%%", cpu_usage, mem_usage);
}

    // === Calcolo CPU usage ===
double ResourceMonitorPublisher::getCpuUsage()
{
  std::ifstream stat_file("/proc/stat");
  std::string line;
  std::getline(stat_file, line);

  std::istringstream ss(line);
  std::string cpu;
  CpuTimes current;
  ss >> cpu >> current.user >> current.nice >> current.system >> current.idle
           >> current.iowait >> current.irq >> current.softirq >> current.steal;

  if (first_cpu_sample_) {
    prev_cpu_times_ = current;
    first_cpu_sample_ = false;
    return 0.0;
  }

  uint64_t total_delta = current.total() - prev_cpu_times_.total();
  uint64_t active_delta = current.active() - prev_cpu_times_.active();

  prev_cpu_times_ = current;

  return (total_delta > 0) ? (100.0 * active_delta / total_delta) : 0.0;
}

    // === Calcolo MEM usage ===
double ResourceMonitorPublisher::getMemoryUsage()
{
  std::ifstream meminfo_file("/proc/meminfo");
  std::string key;
  uint64_t mem_total = 0, mem_available = 0;
  uint64_t value;
  std::string unit;

  while (meminfo_file >> key >> value >> unit) {
    if (key == "MemTotal:") {mem_total = value;} else if (key == "MemAvailable:") {
      mem_available = value;
    }
    if (mem_total > 0 && mem_available > 0) {break;}
  }

  if (mem_total == 0) {return 0.0;}

  uint64_t used_memory = mem_total - mem_available;
  return  100.0 * used_memory / mem_total;
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(resource_monitor::ResourceMonitorPublisher)