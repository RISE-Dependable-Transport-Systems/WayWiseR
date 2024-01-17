#ifndef WAYWISER_TELEOP_EMERGENCY_STOP_MONITOR_HPP_
#define WAYWISER_TELEOP_EMERGENCY_STOP_MONITOR_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using namespace std::chrono;

namespace waywiser_teleop
{
class EmergencyStopMonitor : public rclcpp::Node
{
public:
  explicit EmergencyStopMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr emergency_stop_msg);
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

  bool emergency_stop_value_;
};
}  // namespace waywiser_teleop
#endif  // WAYWISER_TELEOP_EMERGENCY_STOP_MONITOR_HPP_
