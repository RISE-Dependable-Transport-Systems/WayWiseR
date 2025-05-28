#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::placeholders;

class EmulatedRangeSensor : public rclcpp::Node
{
public:
  EmulatedRangeSensor()
  : Node("emulated_range_sensor")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "range_sensor");
    depth_topic_ =
      this->declare_parameter<std::string>("depth_topic", "sensors/depth_camera/image");
    range_topic_ = this->declare_parameter<std::string>("range_topic", "sensors/range");
    publish_rate_ = this->declare_parameter<int>("publish_rate", 10);
    min_range_ = this->declare_parameter<double>("min_range", 0.1);
    max_range_ = this->declare_parameter<double>("max_range", 50.0);
    field_of_view_ = this->declare_parameter<double>("field_of_view", 12.0);
    radiation_type_ = this->declare_parameter<int>(
      "radiation_type",
      sensor_msgs::msg::Range::ULTRASOUND);

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, 10,
      std::bind(&EmulatedRangeSensor::depthCallback, this, _1));

    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>(range_topic_, 10);

    sim_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/carla/simulation_ready", 10, std::bind(&EmulatedRangeSensor::simReadyCallback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_),
      std::bind(&EmulatedRangeSensor::publishRange, this));

    last_sim_time_ = this->get_clock()->now();
  }

private:
  void simReadyCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      is_sim_ready_ = true;
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        msg,
        sensor_msgs::image_encodings::TYPE_32FC1);
      depth_image_ = cv_ptr->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void publishRange()
  {
    if (!is_sim_ready_ || depth_image_.empty()) {
      return;
    }

    auto now = this->get_clock()->now();

    if (now < last_sim_time_) {
      RCLCPP_WARN(this->get_logger(), "Clock reset detected! Resetting range sensor.");
      is_sim_ready_ = false;
      last_sim_time_ = now;
      return;
    }

    last_sim_time_ = now;

    cv::Mat valid_mask = (depth_image_ >= min_range_) & (depth_image_ <= max_range_);
    cv::Mat masked_depth;
    depth_image_.copyTo(masked_depth, valid_mask);
    double min_distance = -1;
    cv::minMaxLoc(masked_depth, &min_distance, nullptr, nullptr, nullptr, valid_mask);
    if (min_distance < min_range_ || min_distance > max_range_) {
      min_distance = std::numeric_limits<double>::quiet_NaN();
    }

    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = now;
    range_msg.header.frame_id = frame_id_;
    range_msg.radiation_type = radiation_type_;
    range_msg.field_of_view = field_of_view_ * (M_PI / 180);
    range_msg.min_range = min_range_;
    range_msg.max_range = max_range_;
    range_msg.range = min_distance;

    range_pub_->publish(range_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_ready_sub_;

  std::string frame_id_;
  std::string depth_topic_;
  std::string range_topic_;
  int publish_rate_;
  double min_range_;
  double max_range_;
  double field_of_view_;
  int radiation_type_;

  bool is_sim_ready_ = false;
  rclcpp::Time last_sim_time_;
  cv::Mat depth_image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmulatedRangeSensor>());
  rclcpp::shutdown();
  return 0;
}
