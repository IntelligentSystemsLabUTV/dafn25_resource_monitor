#ifndef PUB_HPP
#define PUB_HPP

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#include <std_msgs/msg/string.hpp> //! Interface library that we'll use
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace resource_monitor {

struct CpuTimes
{
  uint64_t user = 0, nice = 0, system = 0, idle = 0;
  uint64_t iowait = 0, irq = 0, softirq = 0, steal = 0;

  uint64_t total() const
  {
    return user + nice + system + idle + iowait + irq + softirq + steal;
  }

  uint64_t active() const
  {
    return user + nice + system + irq + softirq + steal;
  }
};

class ResourceMonitorPublisher : public rclcpp::Node {
public:
    ResourceMonitorPublisher(const rclcpp::NodeOptions & node_opts);

private:
    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cpu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mem_pub_;

    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parametri
    bool autostart_;
    int sampling_period_;

    // Stato CPU precedente
    CpuTimes prev_cpu_times_;
    bool first_cpu_sample_ = true;

    // === Service enable/disable ===
    void enableCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // === Avvia il timer ===
    void startMonitoring();

    // === Ferma il timer ===
    void stopMonitoring();

    // === Callback del timer ===
    void timerCallback();

    // === Calcolo CPU usage ===
    double getCpuUsage();

    // === Calcolo MEM usage ===
    double getMemoryUsage();
};

}

#endif