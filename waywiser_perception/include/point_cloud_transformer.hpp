#ifndef WAYWISER_PERCEPTION_POINT_CLOUD_TRANSFORMER_HPP_
#define WAYWISER_PERCEPTION_POINT_CLOUD_TRANSFORMER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace waywiser_perception
{
class PointCloudTransformer : public rclcpp::Node
{
  // Transforms point cloud data to a specific frame either by using desired static transformation or by looking up tranform.

public:
  explicit PointCloudTransformer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PointCloudTransformer() {}

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

  // Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string output_frame_;
  bool static_tranformation_;
  std::vector<double> static_translation_xyz_, static_rotation_xyzw_;
  sensor_msgs::msg::PointCloud2 transformed_pointCloud;
  geometry_msgs::msg::TransformStamped transform_stamped;
};
}  // namespace waywiser_perception
#endif  // WAYWISER_PERCEPTION_POINT_CLOUD_TRANSFORMER_HPP_
