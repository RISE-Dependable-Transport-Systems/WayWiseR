#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "emergency_stop_monitor.hpp"

using namespace std::chrono;
using std::placeholders::_1;

namespace waywiser_command_control
{
EmergencyStopMonitor::EmergencyStopMonitor(const rclcpp::NodeOptions & options)
: Node("emergency_stop_monitor", options)
{
  emergency_stop_value_ = this->declare_parameter("start_with_emergency_stop", true);

  emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10, std::bind(&EmergencyStopMonitor::emergency_stop_callback, this, _1));

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
  if (emergency_stop_value_) {
    twist_msg->linear.x = 0.0;
    twist_msg->angular.z = 0.0;
  }
  twist_publisher_->publish(*twist_msg);
}
}  // namespace waywiser_command_control

// Register EmergencyStopMonitor as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(waywiser_command_control::EmergencyStopMonitor)
