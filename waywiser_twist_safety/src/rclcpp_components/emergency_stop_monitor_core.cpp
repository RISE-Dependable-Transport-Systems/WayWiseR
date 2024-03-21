#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "emergency_stop_monitor.hpp"

using std::placeholders::_1;

namespace waywiser_twist_safety
{
EmergencyStopMonitor::EmergencyStopMonitor(const rclcpp::NodeOptions & options)
: Node("emergency_stop_monitor", options)
{
  emergency_stop_value_ = this->declare_parameter("start_with_emergency_stop", true);

  emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10, std::bind(&EmergencyStopMonitor::emergency_stop_callback, this, _1));

  cmd_vel_in_timeout_ = this->declare_parameter("cmd_vel_in_timeout", 0.5);

  twist_watchdog_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 * cmd_vel_in_timeout_)),
    std::bind(&EmergencyStopMonitor::twist_watchdog_callback, this));

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_in", 10, std::bind(&EmergencyStopMonitor::twist_callback, this, _1));
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

  if (emergency_stop_value_) {
    RCLCPP_WARN(get_logger(), "emergency_stop is ACTIVATED at startup.");
  } else {
    RCLCPP_WARN(get_logger(), "emergency_stop is CLEARED at startup.");
  }

  RCLCPP_INFO(
    get_logger(),
    "To activate emergency_stop from command line: ros2 topic pub "
    "--once /emergency_stop std_msgs/msg/Bool \"data: true\"");
  RCLCPP_INFO(
    get_logger(),
    "To clear emergency_stop from command line: ros2 topic pub "
    "--once /emergency_stop std_msgs/msg/Bool \"data: false\"");
}

void EmergencyStopMonitor::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr emergency_stop_msg)
{
  if (emergency_stop_msg->data) {
    if (!emergency_stop_value_) {
      emergency_stop_value_ = true;
      RCLCPP_WARN(get_logger(), "emergency_stop ACTIVATED from /emergency_stop topic.");
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_publisher_->publish(twist_msg);
    }
  } else {
    if (emergency_stop_value_) {
      emergency_stop_value_ = false;
      RCLCPP_WARN(get_logger(), "emergency_stop CLEARED from /emergency_stop topic.");
    }
  }
}

void EmergencyStopMonitor::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  if (twist_watchdog_timer_->is_canceled()) {
    RCLCPP_WARN(get_logger(), "Receiving msgs from cmd_vel_in topic.");
  }
  twist_watchdog_timer_->reset();

  if (emergency_stop_value_) {
    twist_msg->linear.x = 0.0;
    twist_msg->angular.z = 0.0;
  }
  twist_publisher_->publish(*twist_msg);
}

void EmergencyStopMonitor::twist_watchdog_callback()
{
  RCLCPP_WARN(
    get_logger(),
    "cmd_vel_in topic has timedout for %.2f seconds. Publishing zero velocity on cmd_vel_out topic.",
    cmd_vel_in_timeout_);

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  twist_publisher_->publish(twist_msg);

  twist_watchdog_timer_->cancel();
}
}  // namespace waywiser_twist_safety

// Register EmergencyStopMonitor as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(waywiser_twist_safety::EmergencyStopMonitor)
