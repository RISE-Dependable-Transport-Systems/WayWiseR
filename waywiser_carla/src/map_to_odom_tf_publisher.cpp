#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class MapToOdomTfPublisher : public rclcpp::Node
{
public:
  MapToOdomTfPublisher()
  : Node("map_to_odom_tf_publisher")
  {
    base_link_frame_ = this->declare_parameter(
      "base_link_frame",
      "base_link");
    odom_frame_ = this->declare_parameter(
      "odom_frame",
      "odom");
    odom_topic_ = this->declare_parameter("odom_topic", "/odom");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&MapToOdomTfPublisher::odom_callback, this, _1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    try {
      tf2::Transform map_to_base_link_tf, odom_to_base_link_tf;
      geometry_msgs::msg::TransformStamped map_to_base_link =
        tf_buffer_->lookupTransform("map", base_link_frame_, tf2::TimePoint());
      tf2::fromMsg(map_to_base_link.transform, map_to_base_link_tf);

      auto odom_to_base_link = geometry_msgs::msg::Transform();
      odom_to_base_link.translation.x = odom_msg->pose.pose.position.x;
      odom_to_base_link.translation.y = odom_msg->pose.pose.position.y;
      odom_to_base_link.translation.z = odom_msg->pose.pose.position.z;
      odom_to_base_link.rotation = odom_msg->pose.pose.orientation;
      tf2::fromMsg(odom_to_base_link, odom_to_base_link_tf);

      static tf2::Transform inital_odom_tf = odom_to_base_link_tf;
      odom_to_base_link_tf = inital_odom_tf.inverseTimes(odom_to_base_link_tf);

      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = this->now();
      map_to_odom.header.frame_id = "map";
      map_to_odom.child_frame_id = odom_frame_;
      tf2::toMsg(map_to_base_link_tf * odom_to_base_link_tf.inverse(), map_to_odom.transform);

      // -- Publish Transform
      tf_pub_->sendTransform(map_to_odom);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::string base_link_frame_, odom_frame_, odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapToOdomTfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
