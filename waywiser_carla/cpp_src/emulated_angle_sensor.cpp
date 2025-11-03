#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include "std_msgs/msg/bool.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "waywiser/waywiser_utils.hpp"

using namespace std::placeholders;

class EmulatedAngleSensor : public rclcpp::Node
{
public:
  EmulatedAngleSensor()
  : Node("emulated_angle_sensor_node")
  {
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "truck");
    trailer_base_frame_ = this->declare_parameter<std::string>("trailer_base_frame", "semitrailer");
    topic_ = this->declare_parameter<std::string>("angle_sensor_topic", "sensors/angle");
    publish_rate_ = this->declare_parameter<int>("publish_rate", 10);
    invert_angle_ = this->declare_parameter<bool>("invert_angle", false);
    angle_offset_ = this->declare_parameter<double>("angle_offset", 0.0);

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

    // Rear axle to base
    auto vector3_param = get_vector3_param(this, "rear_axle_frame_to_base_frame_offset");
    if (vector3_param) {
      rear_axle_frame_to_base_frame_offset_ = vector3_param->to_type<vector3_t>();
    }
    vector3_param = get_vector3_param(this, "rear_axle_frame_to_hitch_frame_offset");
    if (vector3_param) {
      rear_axle_frame_to_hitch_frame_offset_ = vector3_param->to_type<vector3_t>();
    }
    base_frame_to_hitch_frame_offset_ = rear_axle_frame_to_hitch_frame_offset_ -
      rear_axle_frame_to_base_frame_offset_;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, 10);

    // Initialize the timer for the publishing rate
    timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds(1000 / publish_rate_),
      std::bind(&EmulatedAngleSensor::calculateAngle, this)
    );
  }

private:
  void calculateAngle()
  {
    try {
      geometry_msgs::msg::TransformStamped baseTransformStamped = tf_buffer_->lookupTransform(
        base_frame_, world_frame_, this->get_clock()->now(),
        rclcpp::Duration(std::chrono::duration<double>(1.0 / publish_rate_)));

      geometry_msgs::msg::TransformStamped trailerBaseTransformStamped =
        tf_buffer_->lookupTransform(
        trailer_base_frame_, world_frame_, this->get_clock()->now(),
        rclcpp::Duration(std::chrono::duration<double>(1.0 / publish_rate_)));

      double baseYaw = getYawFromTransform(baseTransformStamped);
      double trailerYaw = getYawFromTransform(trailerBaseTransformStamped);


      double yawDiff = normalizeAngle(trailerYaw - baseYaw);

      // Invert the angle if the parameter is set
      if (invert_angle_) {
        yawDiff = -yawDiff;
      }

      std_msgs::msg::Float32 angle_msg;
      angle_msg.data = yawDiff * 180.0 / M_PI + angle_offset_;
      angle_pub_->publish(angle_msg);
    } catch (tf2::TransformException & ex) {
      // do nothing
    }
  }

  double getYawFromTransform(const geometry_msgs::msg::TransformStamped & t)
  {
    tf2::Quaternion q;
    tf2::fromMsg(t.transform.rotation, q);
    return tf2::getYaw(q);
  }

  double normalizeAngle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string world_frame_;
  std::string base_frame_;
  std::string trailer_base_frame_;
  std::string topic_;
  int publish_rate_;
  bool invert_angle_;
  double angle_offset_;
  vector3_t rear_axle_frame_to_base_frame_offset_;
  vector3_t rear_axle_frame_to_hitch_frame_offset_;
  vector3_t base_frame_to_hitch_frame_offset_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmulatedAngleSensor>());
  rclcpp::shutdown();
  return 0;
}
