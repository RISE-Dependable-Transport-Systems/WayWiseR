#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include "std_msgs/msg/bool.hpp"

using namespace std::placeholders;

class EmulatedAngleSensor : public rclcpp::Node
{
public:
  EmulatedAngleSensor()
  : Node("emulated_angle_sensor")
  {
    primary_frame_ = this->declare_parameter<std::string>("primary_frame", "truck");
    secondary_frame_ = this->declare_parameter<std::string>("secondary_frame", "semitrailer");
    topic_ = this->declare_parameter<std::string>("angle_sensor_topic", "sensors/angle");
    publish_rate_ = this->declare_parameter<int>("publish_rate", 10);
    invert_angle_ = this->declare_parameter<bool>("invert_angle", false);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, 10);
    sim_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/carla/simulation_ready", 10, std::bind(
        &EmulatedAngleSensor::simReadyCallback, this, _1));


    // Initialize the timer for the publishing rate
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_),
      std::bind(&EmulatedAngleSensor::calculateAngle, this));


    last_sim_time_ = this->get_clock()->now();
  }

private:
  void simReadyCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      is_sim_ready_ = true;
    }
  }

  void calculateAngle()
  {
    if (is_sim_ready_) {
      auto now = this->get_clock()->now();

      // Detect clock reset
      if (now < last_sim_time_) {
        RCLCPP_WARN(this->get_logger(), "Clock reset detected! Resetting initial angle.");
        initial_angle_set_ = false;
        is_sim_ready_ = false;
        last_sim_time_ = now;
        return;
      }

      last_sim_time_ = now;

      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform(
          primary_frame_, secondary_frame_, tf2::TimePointZero);

        double dx = transformStamped.transform.translation.x;
        double dy = transformStamped.transform.translation.y;
        double angle_radians = atan2(dy, dx);
        double angle_degrees = angle_radians * (180.0 / M_PI);

        if (!initial_angle_set_) {
          initial_angle_ = angle_degrees;
          initial_angle_set_ = true;
        }

        double relative_angle = angle_degrees - initial_angle_;

        // Ensure the angle is within the range [-180, 180]
        if (relative_angle > 180.0) {
          relative_angle -= 360.0;
        } else if (relative_angle < -180.0) {
          relative_angle += 360.0;
        }

        // Invert the angle if the parameter is set
        if (invert_angle_) {
          relative_angle = -relative_angle;
        }

        std_msgs::msg::Float32 angle_msg;
        angle_msg.data = relative_angle;

        angle_pub_->publish(angle_msg);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "Could not transform %s to %s: %s",
          secondary_frame_.c_str(), primary_frame_.c_str(), ex.what());
      }
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_ready_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string primary_frame_;
  std::string secondary_frame_;
  std::string topic_;
  int publish_rate_;
  bool invert_angle_;

  bool initial_angle_set_;
  bool is_sim_ready_ = false;
  double initial_angle_;
  rclcpp::Time last_sim_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmulatedAngleSensor>());
  rclcpp::shutdown();
  return 0;
}
