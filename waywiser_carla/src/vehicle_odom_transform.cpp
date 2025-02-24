#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

class VehicleOdomTransform : public rclcpp::Node
{
public:
  VehicleOdomTransform()
  : Node("vehicle_odom_transform")
  {
    base_frame_ = this->declare_parameter(
      "base_frame",
      "base_link");
    odom_frame_ = this->declare_parameter(
      "odom_frame",
      "odom");
    input_odom_topic_ = this->declare_parameter("input_odom_topic", "/odom_in");
    output_odom_topic_ = this->declare_parameter("output_odom_topic", "/odom");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 10, std::bind(&VehicleOdomTransform::odom_callback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    static bool intialized = false;
    static tf2::Transform inital_odom_tf;
    if (!intialized) {
      auto linear_velocity = std::hypot(
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y,
        odom_msg->twist.twist.linear.z
      );
      if (linear_velocity < 0.01) {
        intialized = true;
        RCLCPP_INFO(this->get_logger(), "Initial odom pose is set.");

        auto odom_to_base_link = geometry_msgs::msg::Transform();
        odom_to_base_link.translation.x = odom_msg->pose.pose.position.x;
        odom_to_base_link.translation.y = odom_msg->pose.pose.position.y;
        odom_to_base_link.translation.z = odom_msg->pose.pose.position.z;
        odom_to_base_link.rotation = odom_msg->pose.pose.orientation;
        tf2::fromMsg(odom_to_base_link, inital_odom_tf);
      }
    } else {
      try {
        tf2::Transform map_to_base_link_tf, odom_to_base_link_tf;
        geometry_msgs::msg::TransformStamped map_to_base_link =
          tf_buffer_->lookupTransform("map", base_frame_, tf2::TimePoint());
        tf2::fromMsg(map_to_base_link.transform, map_to_base_link_tf);

        auto odom_to_base_link = geometry_msgs::msg::Transform();
        odom_to_base_link.translation.x = odom_msg->pose.pose.position.x;
        odom_to_base_link.translation.y = odom_msg->pose.pose.position.y;
        odom_to_base_link.translation.z = odom_msg->pose.pose.position.z;
        odom_to_base_link.rotation = odom_msg->pose.pose.orientation;
        tf2::fromMsg(odom_to_base_link, odom_to_base_link_tf);

        odom_to_base_link_tf = inital_odom_tf.inverseTimes(odom_to_base_link_tf);

        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = this->now();
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = odom_frame_;
        tf2::toMsg(map_to_base_link_tf * odom_to_base_link_tf.inverse(), map_to_odom.transform);

        // -- Publish Transform
        tf_pub_->sendTransform(map_to_odom);

        geometry_msgs::msg::Pose transformed_pose;
        tf2::doTransform(odom_msg->pose.pose, transformed_pose, map_to_odom);
        auto out_odom_msg = std::make_shared<nav_msgs::msg::Odometry>(*odom_msg);
        out_odom_msg->header.frame_id = odom_frame_;
        out_odom_msg->pose.pose = transformed_pose;
        auto rotation_component = (std::sqrt(
            map_to_odom.transform.rotation.x * map_to_odom.transform.rotation.x +
            map_to_odom.transform.rotation.y * map_to_odom.transform.rotation.y +
            map_to_odom.transform.rotation.z * map_to_odom.transform.rotation.z
        ));
        if (rotation_component > 1.01) {
          RCLCPP_WARN(
            this->get_logger(),
            "map_to_odom transform with not supported. Ignoring the rotation component %f in odom transformation.",
            rotation_component);
        }
        odom_pub_->publish(*out_odom_msg);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
      }
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::string base_frame_, odom_frame_, input_odom_topic_, output_odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleOdomTransform>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
