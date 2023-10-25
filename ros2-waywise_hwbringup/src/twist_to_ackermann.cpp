/*
 *     Adapted from: http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
 *     Modifier: 2022 Rickard Häll      rickard.hall@ri.se
 */

#include <memory>
#include <string>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono;

/* This class converts <geometry_msgs::msg::Twist> messages into <ackermann_msgs::msg::AckermannDriveStamped> */

class TwistToAckermann : public rclcpp::Node
{
public:
  TwistToAckermann()
      : Node("twist_to_ackermann")
  {
    steering_angle_to_servo_gain_ = this->declare_parameter("steering_angle_to_servo_gain", 0.0);
    steering_angle_to_servo_offset_ = this->declare_parameter("steering_angle_to_servo_offset", 0.5);
    servo_min_ = this->declare_parameter("servo_min", 0.0);
    servo_max_ = this->declare_parameter("servo_max", 1.0);
    speed_to_erpm_gain_ = this->declare_parameter("speed_to_erpm_gain", 0.0);
    speed_to_erpm_offset_ = this->declare_parameter("speed_to_erpm_offset", 0.0);
    erpm_speed_min_ = this->declare_parameter("speed_min", 0.0);
    erpm_speed_max_ = this->declare_parameter("speed_max", 0.0);
    wheelbase_ = this->declare_parameter("wheelbase", 0.33);
    ack_msg_frame_id_ = this->declare_parameter("ack_msg_frame_id", "odom");
    odom_timeout_ = this->declare_parameter("odom_timeout", 1.0);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&TwistToAckermann::topic_callback, this, _1));

    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ackermann_cmd", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TwistToAckermann::odom_callback, this, std::placeholders::_1));

    odom_watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)std::round(1000.0 * odom_timeout_)), std::bind(&TwistToAckermann::odom_watchdog_callback, this));

    zero_ackermann_cmd.header.frame_id = ack_msg_frame_id_;
    zero_ackermann_cmd.drive.speed = 0;
    zero_ackermann_cmd.drive.steering_angle = 0;
  }

private:
  float clip_min_max(float value, float min_value, float max_value) const
  {
    return std::min(
        std::max(
            value,
            (min_value + std::numeric_limits<float>::epsilon())),
        (max_value - std::numeric_limits<float>::epsilon()));
  }

  float convert_trans_rot_vel_to_steering_angle(float linVel, float angVel) const
  {
    if (angVel == 0 || linVel == 0)
      return 0;
    float turningRadius = linVel / angVel;
    return atan(wheelbase_ / turningRadius);
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twi_msg) const
  {
    auto ack_msg = ackermann_msgs::msg::AckermannDriveStamped();
    ack_msg.header.stamp.sec = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
    ack_msg.header.stamp.nanosec = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
    ack_msg.header.frame_id = ack_msg_frame_id_;

    float erpm = speed_to_erpm_gain_ * twi_msg->linear.x + speed_to_erpm_offset_;
    float linear_vel = (clip_min_max(erpm, erpm_speed_min_, erpm_speed_max_) - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    float steering_angle_from_twist = convert_trans_rot_vel_to_steering_angle(linear_vel, twi_msg->angular.z);
    ack_msg.drive.speed = linear_vel;
    float servo_position = std::abs(steering_angle_to_servo_gain_ * steering_angle_from_twist + steering_angle_to_servo_offset_);
    ack_msg.drive.steering_angle = (clip_min_max(servo_position, servo_min_, servo_max_) - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;
    publisher_->publish(ack_msg);
  }

  void odom_callback([[maybe_unused]] const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    last_odom_msg_received_time_ = this->now();
  }

  void odom_watchdog_callback()
  {
    // Calculate the time elapsed since the last message was received
    auto time_since_last_msg = (this->now() - last_odom_msg_received_time_).seconds();
    if (time_since_last_msg > odom_timeout_)
    {
      if (is_odom_alive_)
      {
        is_odom_alive_ = false;
      }
      publish_zero_ackermann_cmd();
    }
    else
    {
      if (!is_odom_alive_)
      {
        is_odom_alive_ = true;
      }
    }
  }

  void publish_zero_ackermann_cmd()
  {
    zero_ackermann_cmd.header.stamp.sec = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
    zero_ackermann_cmd.header.stamp.nanosec = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
    publisher_->publish(zero_ackermann_cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  float steering_angle_to_servo_gain_, steering_angle_to_servo_offset_, servo_min_, servo_max_;
  float speed_to_erpm_gain_, speed_to_erpm_offset_, erpm_speed_min_, erpm_speed_max_;
  float wheelbase_, odom_timeout_;
  std::string ack_msg_frame_id_;
  rclcpp::TimerBase::SharedPtr odom_watchdog_timer_;
  bool is_odom_alive_ = false;
  rclcpp::Time last_odom_msg_received_time_ = this->now();
  ackermann_msgs::msg::AckermannDriveStamped zero_ackermann_cmd = ackermann_msgs::msg::AckermannDriveStamped();
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckermann>());
  rclcpp::shutdown();
  return 0;
}
