#include "emergency_stop_monitor.hpp"

namespace waywiser_twist_safety
{
EmergencyStopMonitor::EmergencyStopMonitor(const rclcpp::NodeOptions & options)
: Node("emergency_stop_monitor", options)
{
  current_emergency_stop_state_msg = EmergencyStopState();
  current_emergency_stop_state_msg.sender_id = "emergency_stop_monitor";

  if (this->declare_parameter("start_with_emergency_stop", true)) {
    current_emergency_stop_state_msg.state = EmergencyStopState::ACTIVE;
    RCLCPP_WARN(get_logger(), "Emergency stop is ACTIVATED at startup.");
  } else {
    current_emergency_stop_state_msg.state = EmergencyStopState::CLEAR;
    RCLCPP_WARN(get_logger(), "Emergency stop is CLEARED at startup.");
  }

  emergency_stop_target_state_subscriber_ = this->create_subscription<EmergencyStopState>(
    "/emergency_stop/target_state", 10,
    std::bind(&EmergencyStopMonitor::emergency_stop_target_state_callback, this, _1));

  emergency_stop_current_state_publisher_ = this->create_publisher<EmergencyStopState>(
    "/emergency_stop/current_state", 10);

  emergency_stop_state_publish_rate_ = this->declare_parameter(
    "emergency_stop_state_publish_rate", 10);

  emergency_stop_state_publisher_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 / emergency_stop_state_publish_rate_)),
    std::bind(&EmergencyStopMonitor::emergency_stop_state_publisher_timer_callback, this));

  cmd_vel_in_timeout_ = this->declare_parameter("cmd_vel_in_timeout", 0.5);

  twist_watchdog_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 * cmd_vel_in_timeout_)),
    std::bind(&EmergencyStopMonitor::twist_watchdog_callback, this));

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_in", 10, std::bind(&EmergencyStopMonitor::twist_callback, this, _1));
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

  RCLCPP_INFO(
    get_logger(),
    "To activate emergency_stop from command line: ros2 topic pub --once "
    "/emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState \"{sender_id : 'command_line' , state : 2}\"");
  RCLCPP_INFO(
    get_logger(),
    "To clear emergency_stop from command line: ros2 topic pub --once "
    "/emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState \"{sender_id : 'command_line' , state : 1}\"");

}

void EmergencyStopMonitor::emergency_stop_target_state_callback(
  const EmergencyStopState::SharedPtr emergency_stop_target_state_msg)
{

  if (emergency_stop_target_state_msg->state == EmergencyStopState::ACTIVE) {
    if (!is_emergency_stop_active()) {
      current_emergency_stop_state_msg.state = EmergencyStopState::ACTIVE;
      RCLCPP_INFO(
        get_logger(), "Emergency stop ACTIVATED by %s.",
        emergency_stop_target_state_msg->sender_id.c_str());

      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_publisher_->publish(twist_msg);
    }
  } else {
    if (is_emergency_stop_active()) {
      current_emergency_stop_state_msg.state = EmergencyStopState::CLEAR;
      RCLCPP_INFO(
        get_logger(), "Emergency stop CLEARED by %s.",
        emergency_stop_target_state_msg->sender_id.c_str());
    }
  }
}

void EmergencyStopMonitor::emergency_stop_state_publisher_timer_callback()
{
  current_emergency_stop_state_msg.stamp = this->now();
  emergency_stop_current_state_publisher_->publish(current_emergency_stop_state_msg);
}

void EmergencyStopMonitor::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  if (twist_watchdog_timer_->is_canceled()) {
    RCLCPP_WARN(get_logger(), "Receiving msgs from cmd_vel_in topic.");
  }
  twist_watchdog_timer_->reset();

  if (is_emergency_stop_active()) {
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
