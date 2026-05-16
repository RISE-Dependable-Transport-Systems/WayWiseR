#include <map>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::placeholders;

class EmulatedRangeSensorArray : public rclcpp::Node
{
public:
  EmulatedRangeSensorArray()
  : Node("emulated_range_sensor_array_node")
  {
    sensor_ids_ = this->declare_parameter<std::vector<std::string>>(
      "sensor_ids", {}, rcl_interfaces::msg::ParameterDescriptor{});
    publish_rate_ = this->declare_parameter<int>("publish_rate", 10);
    if (sensor_ids_.size() > 1) {
      for (const auto & sensor_id : sensor_ids_) {
        RangeSensorInfo sensor_info;
        sensor_info.frame_id_ = this->declare_parameter<std::string>(
          sensor_id + ".frame_id", "range_sensor");
        sensor_info.depth_topic_ = this->declare_parameter<std::string>(
          sensor_id + ".depth_topic", "sensors/depth_camera/image");
        sensor_info.range_topic_ = this->declare_parameter<std::string>(
          sensor_id + ".range_topic", "sensors/range");
        sensor_info.min_range_ = this->declare_parameter<double>(
          sensor_id + ".min_range", 0.1);
        sensor_info.max_range_ = this->declare_parameter<double>(
          sensor_id + ".max_range", 50.0);
        sensor_info.field_of_view_ = this->declare_parameter<double>(
          sensor_id + ".field_of_view", 12.0) * (M_PI / 180);
        sensor_info.radiation_type_ = this->declare_parameter<int>(
          sensor_id + ".radiation_type", sensor_msgs::msg::Range::ULTRASOUND);
        sensor_array_info_[sensor_id] = sensor_info;

        range_pubs_[sensor_id] = this->create_publisher<sensor_msgs::msg::Range>(
          sensor_info.range_topic_, 10);
        depth_subs_[sensor_id] = this->create_subscription<sensor_msgs::msg::Image>(
          sensor_info.depth_topic_, 10,
          [this, sensor_id](const sensor_msgs::msg::Image::SharedPtr msg) {
            this->depthCallback(msg, sensor_id);
          });
      }
    }
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string & sensor_id)
  {
    try {
      // Direct pointer to image data, no copy
      cv::Mat depth_image = cv_bridge::toCvShare(
        msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

      const auto & sensor_info = sensor_array_info_[sensor_id];

      // Construct mask within range
      cv::Mat valid_mask =
        (depth_image >= sensor_info.min_range_) & (depth_image <= sensor_info.max_range_);

      // Compute min distance
      double min_distance = -1;
      cv::minMaxLoc(depth_image, &min_distance, nullptr, nullptr, nullptr, valid_mask);

      // Validate result
      if (min_distance < sensor_info.min_range_ || min_distance > sensor_info.max_range_) {
        min_distance = std::numeric_limits<double>::quiet_NaN();
      }

      // Prepare and publish range message
      sensor_msgs::msg::Range range_msg;
      range_msg.header.stamp = this->get_clock()->now();
      range_msg.header.frame_id = sensor_info.frame_id_;
      range_msg.radiation_type = sensor_info.radiation_type_;
      range_msg.field_of_view = sensor_info.field_of_view_;
      range_msg.min_range = sensor_info.min_range_;
      range_msg.max_range = sensor_info.max_range_;
      range_msg.range = min_distance;

      range_pubs_[sensor_id]->publish(range_msg);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_subs_;

  std::vector<std::string> sensor_ids_;
  int publish_rate_;
  struct RangeSensorInfo
  {
    std::string frame_id_;
    std::string depth_topic_;
    std::string range_topic_;
    double min_range_;
    double max_range_;
    double field_of_view_;
    int radiation_type_;
  };
  std::map<std::string, RangeSensorInfo> sensor_array_info_;
  cv::Mat depth_image_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmulatedRangeSensorArray>());
  rclcpp::shutdown();
  return 0;
}
