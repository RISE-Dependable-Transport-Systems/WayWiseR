#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "waywiser/waywiser_utils.hpp"

#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"

using namespace std::placeholders;
using namespace std::chrono;
using namespace waywiser_twist_safety::msg;

class EmergencyStopMonitor : public rclcpp::Node
{
public:
  EmergencyStopMonitor()
  : Node("emergency_stop_monitor")
  {
    current_emergency_stop_state_msg = EmergencyStopState();
    current_emergency_stop_state_msg.sender_id = "emergency_stop_monitor";

    start_with_emergency_stop_ = this->declare_parameter("start_with_emergency_stop", true);
    emergency_stop_status_topic_ = this->declare_parameter(
      "emergency_stop_status_topic", "/emergency_stop/status");
    emergency_stop_update_topic_ = this->declare_parameter(
      "emergency_stop_update_topic", "/emergency_stop/target_state");

    if (start_with_emergency_stop_) {
      current_emergency_stop_state_msg.state = EmergencyStopState::ACTIVE;
      RCLCPP_WARN(get_logger(), "Emergency stop is ACTIVATED at startup.");
    } else {
      current_emergency_stop_state_msg.state = EmergencyStopState::CLEAR;
      RCLCPP_WARN(get_logger(), "Emergency stop is CLEARED at startup.");
    }

    emergency_stop_target_state_subscriber_ = this->create_subscription<EmergencyStopState>(
      emergency_stop_update_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
      std::bind(&EmergencyStopMonitor::emergency_stop_target_state_callback, this, _1));

    emergency_stop_current_state_publisher_ = this->create_publisher<EmergencyStopState>(
      emergency_stop_status_topic_, 10);

    emergency_stop_state_publish_rate_ = this->declare_parameter(
      "emergency_stop_state_publish_rate", 10);

    auto use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
      if (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /clock to be published...");
      }

      while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      }
      RCLCPP_WARN(this->get_logger(), "Receiving /clock msgs now.");
    }

    emergency_stop_state_publisher_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds((int)std::round(1000.0 / emergency_stop_state_publish_rate_)),
      std::bind(&EmergencyStopMonitor::emergency_stop_state_publisher_timer_callback, this)
    );

    cmd_vel_in_timeout_ = this->declare_parameter("cmd_vel_in_timeout", 0.5);

    twist_watchdog_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds((int)std::round(1000.0 * cmd_vel_in_timeout_)),
      std::bind(&EmergencyStopMonitor::twist_watchdog_callback, this)
    );

    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10, std::bind(&EmergencyStopMonitor::twist_callback, this, _1));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

    RCLCPP_INFO(
      get_logger(),
      "To activate emergency_stop from command line: ros2 topic pub --once "
      "/emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState \"{sender_id : 'command_line' , state : 2}\"");
    RCLCPP_INFO(
      get_logger(),
      "To clear emergency_stop from command line: ros2 topic pub --once "
      "/emergency_stop/target_state waywiser_twist_safety/msg/EmergencyStopState \"{sender_id : 'command_line' , state : 1}\"");
  }

private:
  bool is_emergency_stop_active()
  {
    return current_emergency_stop_state_msg.state == EmergencyStopState::ACTIVE;
  }

  void emergency_stop_target_state_callback(
    const EmergencyStopState::SharedPtr emergency_stop_target_state_msg)
  {
    if (emergency_stop_target_state_msg->state == EmergencyStopState::ACTIVE) {
      if (!is_emergency_stop_active()) {
        current_emergency_stop_state_msg.state = EmergencyStopState::ACTIVE;
        RCLCPP_WARN(
          get_logger(), "Emergency stop ACTIVATED by %s.",
          emergency_stop_target_state_msg->sender_id.c_str());

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        twist_publisher_->publish(twist_msg);
      }
    } else if (emergency_stop_target_state_msg->state == EmergencyStopState::CLEAR) {
      if (is_emergency_stop_active()) {
        current_emergency_stop_state_msg.state = EmergencyStopState::CLEAR;
        RCLCPP_WARN(
          get_logger(), "Emergency stop CLEARED by %s.",
          emergency_stop_target_state_msg->sender_id.c_str());
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Received emergency stop request with unknown state %d from %s. Ignoring.",
        emergency_stop_target_state_msg->state,
        emergency_stop_target_state_msg->sender_id.c_str());
    }
  }

  void emergency_stop_state_publisher_timer_callback()
  {
    static auto previousTimeCalled = this->get_clock()->now();
    auto thisTimeCalled = this->get_clock()->now();
    double timePassedSinceLastCall_ms =
      (thisTimeCalled.nanoseconds() - previousTimeCalled.nanoseconds()) / 1e6;
    // Detect clock reset
    if (timePassedSinceLastCall_ms < 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "Clock reset detected! Resetting emergency_stop_monitor.");

      if (start_with_emergency_stop_) {
        current_emergency_stop_state_msg.state = EmergencyStopState::ACTIVE;
        RCLCPP_WARN(get_logger(), "Emergency stop is ACTIVATED at startup.");
      } else {
        current_emergency_stop_state_msg.state = EmergencyStopState::CLEAR;
        RCLCPP_WARN(get_logger(), "Emergency stop is CLEARED at startup.");
      }
      previousTimeCalled = thisTimeCalled;
      return;
    }

    current_emergency_stop_state_msg.stamp = thisTimeCalled;
    emergency_stop_current_state_publisher_->publish(current_emergency_stop_state_msg);
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
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

  void twist_watchdog_callback()
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

  std::string emergency_stop_status_topic_;
  std::string emergency_stop_update_topic_;

  rclcpp::Subscription<EmergencyStopState>::SharedPtr emergency_stop_target_state_subscriber_;
  rclcpp::Publisher<EmergencyStopState>::SharedPtr emergency_stop_current_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

  EmergencyStopState current_emergency_stop_state_msg;
  float cmd_vel_in_timeout_;
  int emergency_stop_state_publish_rate_;
  bool start_with_emergency_stop_;

  // Timer
  rclcpp::TimerBase::SharedPtr twist_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr emergency_stop_state_publisher_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStopMonitor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
