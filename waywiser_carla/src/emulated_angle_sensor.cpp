#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class EmulatedAngleSensor : public rclcpp::Node
{
public:
  EmulatedAngleSensor()
  : Node("emulated_angle_sensor"), warmup_elapsed_(false)
  {
    primary_frame_ = this->declare_parameter<std::string>("primary_frame", "truck");
    secondary_frame_ = this->declare_parameter<std::string>("secondary_frame", "semitrailer");
    topic_ = this->declare_parameter<std::string>("topic", "sensors/angle");
    publish_rate_ = this->declare_parameter<int>("publish_rate", 10);
    invert_angle_ = this->declare_parameter<bool>("invert_angle", false);
    warmup_time_ = this->declare_parameter<double>("warmup_time", 10.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, 10);

    // Initialize the timer for the publishing rate
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_),
      std::bind(&EmulatedAngleSensor::calculateAngle, this));

    // Initialize the start time
    start_time_ = this->get_clock()->now();
  }

private:
  void calculateAngle()
  {
    auto current_time = this->get_clock()->now();
    auto elapsed_time = (current_time - start_time_).seconds();

    if (elapsed_time < warmup_time_) {
      // During warm-up time, no data is published
      return;
    } else {
      // Warm-up period has elapsed
      if (!warmup_elapsed_) {
        warmup_elapsed_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "Angle sensor warm-up period elapsed. Publishing data now.");
      }

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

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string primary_frame_;
  std::string secondary_frame_;
  std::string topic_;
  int publish_rate_;
  bool invert_angle_;
  double warmup_time_;

  bool initial_angle_set_;
  bool warmup_elapsed_;
  double initial_angle_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmulatedAngleSensor>());
  rclcpp::shutdown();
  return 0;
}
