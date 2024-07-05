#ifndef WAYWISER_TWIST_SAFETY_EMERGENCY_STOP_MONITOR_HPP_
#define WAYWISER_TWIST_SAFETY_EMERGENCY_STOP_MONITOR_HPP_

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"

using namespace std::placeholders;
using namespace std::chrono;
using namespace waywiser_twist_safety::msg;

namespace waywiser_twist_safety
{
class EmergencyStopMonitor : public rclcpp::Node
{
public:
  explicit EmergencyStopMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool is_emergency_stop_active()
  {
    return current_emergency_stop_state_msg.state == EmergencyStopState::ACTIVE;
  }
  void emergency_stop_target_state_callback(
    const EmergencyStopState::SharedPtr emergency_stop_target_state_msg);
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void twist_watchdog_callback();
  void emergency_stop_state_publisher_timer_callback();

  rclcpp::Subscription<EmergencyStopState>::SharedPtr emergency_stop_target_state_subscriber_;
  rclcpp::Publisher<EmergencyStopState>::SharedPtr emergency_stop_current_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

  EmergencyStopState current_emergency_stop_state_msg;
  float cmd_vel_in_timeout_;
  int emergency_stop_state_publish_rate_;

  // Timer
  rclcpp::TimerBase::SharedPtr twist_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr emergency_stop_state_publisher_timer_;
};
}  // namespace waywiser_twist_safety
#endif  // WAYWISER_TWIST_SAFETY_EMERGENCY_STOP_MONITOR_HPP_
