#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono;
using std::placeholders::_1;

class JoyEmergencyStop : public rclcpp::Node
{
public:
  JoyEmergencyStop()
  : Node("joy_emergency_stop")
  {
    emergency_stop_set_joy_button_index_ = this->declare_parameter(
      "emergency_stop_set_joy_button_index", 5);
    emergency_stop_clear_joy_button_index_ = this->declare_parameter(
      "emergency_stop_clear_joy_button_index", 7);
    joy_emergency_stop_timeout_ = this->declare_parameter("joy_emergency_stop_timeout", 1.0);

    emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyEmergencyStop::joy_callback, this, std::placeholders::_1));

    joy_watchdog_timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds((int)std::round(1000.0 * joy_emergency_stop_timeout_)),
      std::bind(&JoyEmergencyStop::joy_watchdog_callback, this));
    emergency_stop_msg_.data = false;

    RCLCPP_INFO(
      get_logger(),
      "emergency_stop can be ACTIVATED using button %d and CLEARED "
      "using button %d of joystick.",
      emergency_stop_set_joy_button_index_, emergency_stop_clear_joy_button_index_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    last_joy_msg_received_time_ = this->now();
    if (joy_msg->buttons[emergency_stop_set_joy_button_index_] == 1) {
      if (!emergency_stop_msg_.data) {
        emergency_stop_msg_.data = true;
        emergency_stop_publisher_->publish(emergency_stop_msg_);
        RCLCPP_WARN(get_logger(), "emergency_stop ACTIVATED from /joy topic.");
      } else {
        emergency_stop_publisher_->publish(emergency_stop_msg_);
      }
    } else if (joy_msg->buttons[emergency_stop_clear_joy_button_index_] == 1) {
      if (emergency_stop_msg_.data) {
        emergency_stop_msg_.data = false;
        emergency_stop_publisher_->publish(emergency_stop_msg_);
        RCLCPP_WARN(get_logger(), "emergency_stop CLEARED from /joy topic.");
      } else {
        emergency_stop_publisher_->publish(emergency_stop_msg_);
      }
    }
  }

  void joy_watchdog_callback()
  {
    if (last_joy_msg_received_time_.seconds() > 0) {
      // Check if joy has started publishing messages since node is started
      // Calculate the time elapsed since the last message was received
      auto time_since_last_msg = (this->now() - last_joy_msg_received_time_).seconds();
      if (time_since_last_msg > joy_emergency_stop_timeout_) {
        if (is_joy_alive_) {
          emergency_stop_msg_.data = true;
          emergency_stop_publisher_->publish(emergency_stop_msg_);
          RCLCPP_WARN(
            get_logger(),
            "/joy topic has stopped publishing for %.2f seconds. "
            "emergency_stop ACTIVATED.",
            time_since_last_msg);
          is_joy_alive_ = false;
        }
      } else {
        if (!is_joy_alive_) {
          is_joy_alive_ = true;
          if (emergency_stop_msg_.data) {
            RCLCPP_WARN(
              get_logger(),
              "Receiving messages from /joy topic now, "
              "but emergency_stop is ACTIVATED.");
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Receiving messages from /joy topic now. "
              "Monitoring emergency_stop button.");
          }
        }
      }
    }
  }

  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Timer
  rclcpp::TimerBase::SharedPtr joy_watchdog_timer_;

  // Parameters
  int emergency_stop_set_joy_button_index_;
  int emergency_stop_clear_joy_button_index_;
  float joy_emergency_stop_timeout_;
  bool is_joy_alive_ = false;
  std_msgs::msg::Bool emergency_stop_msg_;
  rclcpp::Time last_joy_msg_received_time_ = rclcpp::Time(
    (int64_t)-1,
    rcl_clock_type_t::RCL_ROS_TIME);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyEmergencyStop>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
