#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "point_cloud_transformer.hpp"

namespace waywiser_perception
{
PointCloudTransformer::PointCloudTransformer(const rclcpp::NodeOptions & options)
: Node("point_cloud_transformer", options)
{
  output_frame_ = this->declare_parameter("output_frame", "base_link");
  static_tranformation_ = this->declare_parameter("static_tranformation", false);

  if (static_tranformation_) {
    static_translation_xyz_ =
      this->declare_parameter("static_translation_xyz", std::vector<double>({0.0, 0.0, 0.0}));
    static_rotation_xyzw_ =
      this->declare_parameter("static_rotation_xyzw", std::vector<double>({0.0, 0.0, 1.0, 0.0}));

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = output_frame_;
    transform_stamped.transform.translation.x = static_translation_xyz_[0];
    transform_stamped.transform.translation.y = static_translation_xyz_[1];
    transform_stamped.transform.translation.z = static_translation_xyz_[2];
    transform_stamped.transform.rotation.x = static_rotation_xyzw_[0];
    transform_stamped.transform.rotation.y = static_rotation_xyzw_[1];
    transform_stamped.transform.rotation.z = static_rotation_xyzw_[2];
    transform_stamped.transform.rotation.w = static_rotation_xyzw_[3];
  } else {
    // Initialize Transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Initialize Publishers and subscribers
  std::string input_point_cloud_topic = this->declare_parameter(
    "input_point_cloud_topic",
    "points_raw");
  std::string output_point_cloud_topic = this->declare_parameter(
    "output_point_cloud_topic",
    "points");
  publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(output_point_cloud_topic, 10);
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_point_cloud_topic, 10,
    std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
}

void PointCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    // Lookup transformation from input_frame to output_frame
    if (static_tranformation_) {
      if (transform_stamped.child_frame_id == "") {
        transform_stamped.child_frame_id = msg->header.frame_id;
      }
      transform_stamped.header.stamp = this->now();
    } else {
      transform_stamped =
        tf_buffer_->lookupTransform(
        output_frame_, msg->header.frame_id, msg->header.stamp, std::chrono::seconds(
          1));
      RCLCPP_DEBUG(
        this->get_logger(), "PC Transform - translation: [%f, %f, %f], rotation: [%f,  %f,  %f,  %f]", transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y, transform_stamped.transform.translation.z,
        transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);
    }

    // Transform the input point cloud to the output frame
    tf2::doTransform(*msg, transformed_pointCloud, transform_stamped);

    // Publish the transformed point cloud
    publisher_->publish(transformed_pointCloud);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
  }
}
}  // namespace waywiser_perception

// Register the PointCloudTransformer as a plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(waywiser_perception::PointCloudTransformer)
