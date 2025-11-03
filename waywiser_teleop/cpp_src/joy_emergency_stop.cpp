#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"
#include "waywiser/waywiser_utils.hpp"

using namespace std::placeholders;
using namespace waywiser_twist_safety::msg;

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

    auto use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time) {
      if (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /clock to be published...");
      }

      while (rclcpp::ok() && this->get_clock()->now().nanoseconds() == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      }
      RCLCPP_INFO(this->get_logger(), "Receiving /clock msgs now.");
    }

    emergency_stop_request_publisher_ = this->create_publisher<EmergencyStopState>(
      "/emergency_stop/target_state", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyEmergencyStop::joy_callback, this, _1));

    joy_watchdog_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds((int)std::round(1000.0 * joy_emergency_stop_timeout_)),
      std::bind(&JoyEmergencyStop::joy_watchdog_callback, this)
    );

    emergency_stop_target_state_msg_.sender_id = "twist_joy";
    emergency_stop_target_state_msg_.state = EmergencyStopState::ACTIVE;

    RCLCPP_INFO(
      get_logger(),
      "emergency_stop can be ACTIVATED using button %d and CLEARED "
      "using button %d of joystick.",
      emergency_stop_set_joy_button_index_, emergency_stop_clear_joy_button_index_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_watchdog_timer_->is_canceled()) {
      RCLCPP_WARN(
        get_logger(),
        "Receiving messages from /joy topic now. "
        "Monitoring emergency_stop button.");
    }
    joy_watchdog_timer_->reset();

    if (joy_msg->buttons[emergency_stop_set_joy_button_index_] == 1) {
      emergency_stop_target_state_msg_.state = EmergencyStopState::ACTIVE;
      emergency_stop_request_publisher_->publish(emergency_stop_target_state_msg_);
    } else if (joy_msg->buttons[emergency_stop_clear_joy_button_index_] == 1) {
      emergency_stop_target_state_msg_.state = EmergencyStopState::CLEAR;
      emergency_stop_request_publisher_->publish(emergency_stop_target_state_msg_);
    }
  }

  void joy_watchdog_callback()
  {
    emergency_stop_target_state_msg_.state = EmergencyStopState::ACTIVE;
    emergency_stop_request_publisher_->publish(emergency_stop_target_state_msg_);
    RCLCPP_WARN(
      get_logger(),
      "/joy topic has stopped publishing for %.2f seconds. "
      "emergency_stop ACTIVATED.",
      joy_emergency_stop_timeout_);

    joy_watchdog_timer_->cancel();
  }

  // Publishers and subscribers
  rclcpp::Publisher<EmergencyStopState>::SharedPtr emergency_stop_request_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Timer
  rclcpp::TimerBase::SharedPtr joy_watchdog_timer_;

  // Parameters
  int emergency_stop_set_joy_button_index_;
  int emergency_stop_clear_joy_button_index_;
  float joy_emergency_stop_timeout_;
  EmergencyStopState emergency_stop_target_state_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyEmergencyStop>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
