#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "waywiser/waywiser_utils.hpp"
#include "waywiser_core_utils.hpp"

#include "waywiser_core/msg/nav_sat_fix_extended.hpp"

using namespace std::placeholders;

class TfsToNavsatfixfused : public rclcpp::Node
{
public:
  TfsToNavsatfixfused(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "tfs_to_navsatfixfused_node")
  : Node(node_name, options)
  {
    // Check if the ROS clock is available
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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS parameters
    world_frame_ = this->declare_parameter("world_frame", "map");
    tf_object_frame_ = this->declare_parameter("tf_object_frame", "base_link");
    gnss_reference_frame_ = this->declare_parameter(
      "gnss_reference_frame", "gnss_base_link");
    nav_sat_fix_extended_topic_ = this->declare_parameter(
      "nav_sat_fix_extended_topic", "/nav_sat_fix_extended");
    gnss_message_rate_ = this->declare_parameter("gnss_message_rate", 10);
    auto vector3_param = get_vector3_param(this, "enuref");
    if (vector3_param) {
      enuref_ = vector3_param->to_type<llh_t>();
    }

    gnss_fix_type_ =
      this->declare_parameter("gnss_fix_type", static_cast<uint8_t>(GNSS_FIX_TYPE::FIX_3D));
    gnss_horizontal_accuracy_ =
      this->declare_parameter("gnss_horizontal_accuracy", std::numeric_limits<double>::infinity());
    gnss_vertical_accuracy_ =
      this->declare_parameter("gnss_vertical_accuracy", std::numeric_limits<double>::infinity());
    gnss_heading_accuracy_ =
      this->declare_parameter("gnss_heading_accuracy", std::numeric_limits<double>::infinity());
    gnss_last_rtcm_correction_age_ =
      this->declare_parameter("gnss_last_rtcm_correction_age", 0);
    gnss_num_satellites_ =
      this->declare_parameter("gnss_num_satellites", 0);
    gnss_is_fused_on_chip_ =
      this->declare_parameter("gnss_is_fused_on_chip", true);

    // Publishers
    nav_sat_fix_extended_pub_ = this->create_publisher<waywiser_core::msg::NavSatFixExtended>(
      nav_sat_fix_extended_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);

    // Timers
    node_management_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds(1000 / gnss_message_rate_),
      std::bind(&TfsToNavsatfixfused::node_management_timer_callback, this)
    );

    // Prepare nav_sat_fix_extended_msg
    nav_sat_fix_extended_msg.header.frame_id = gnss_reference_frame_;
    nav_sat_fix_extended_msg.is_fused_on_chip = gnss_is_fused_on_chip_;
    nav_sat_fix_extended_msg.fix_type = gnss_fix_type_;
    nav_sat_fix_extended_msg.horizontal_accuracy = gnss_horizontal_accuracy_;
    nav_sat_fix_extended_msg.vertical_accuracy = gnss_vertical_accuracy_;
    nav_sat_fix_extended_msg.heading_accuracy = gnss_heading_accuracy_;
    nav_sat_fix_extended_msg.last_rtcm_correction_age = gnss_last_rtcm_correction_age_;
    nav_sat_fix_extended_msg.num_satellites = gnss_num_satellites_;

    RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
  }

private:
  // Callback methods
  void node_management_timer_callback()
  {
    xyz_t xyz;
    double yawRad;
    static bool transform_warning_logged = false;
    try {
      geometry_msgs::msg::TransformStamped map_to_object_msg_tfs = tf_buffer_->lookupTransform(
        world_frame_, tf_object_frame_, this->get_clock()->now(), tf2::durationFromSec(1.0));
      xyz.x = map_to_object_msg_tfs.transform.translation.x;
      xyz.y = map_to_object_msg_tfs.transform.translation.y;
      xyz.z = map_to_object_msg_tfs.transform.translation.z;
      yawRad = tf2::getYaw(map_to_object_msg_tfs.transform.rotation);
      if (transform_warning_logged) {
        RCLCPP_INFO(
          get_logger(), "Transform from %s to %s is available now.",
          world_frame_.c_str(), tf_object_frame_.c_str());
        transform_warning_logged = false;
      }
    } catch (tf2::TransformException & ex) {
      if (!transform_warning_logged) {
        RCLCPP_WARN(
          get_logger(), "Transform from %s to %s is not available yet!",
          world_frame_.c_str(), tf_object_frame_.c_str());
        transform_warning_logged = true;
      }
      return;
    }

    // Publish navSatFixExt
    nav_sat_fix_extended_msg.header.stamp = this->now();

    llh_t llh = coordinateTransforms::enuToLlh(enuref_, xyz);
    nav_sat_fix_extended_msg.latitude = llh.latitude;   // Latitude in degrees
    nav_sat_fix_extended_msg.longitude = llh.longitude;   // Longitude in degrees
    nav_sat_fix_extended_msg.altitude = llh.height;   // Altitude in meters
    nav_sat_fix_extended_msg.heading = coordinateTransforms::yawENUtoNED(yawRad * 180.0 / M_PI);   // degrees

    nav_sat_fix_extended_pub_->publish(nav_sat_fix_extended_msg);
  }

// Parameters
  std::string world_frame_;
  std::string tf_object_frame_;
  std::string gnss_reference_frame_;
  std::string nav_sat_fix_extended_topic_;
  int gnss_message_rate_;   // [Hz]
  llh_t enuref_;   // [lat, lon, height]
  uint8_t gnss_fix_type_;
  double gnss_horizontal_accuracy_;
  double gnss_vertical_accuracy_;
  double gnss_heading_accuracy_;
  uint8_t gnss_last_rtcm_correction_age_;
  uint8_t gnss_num_satellites_;
  bool gnss_is_fused_on_chip_;

// Publishers
  rclcpp::Publisher<waywiser_core::msg::NavSatFixExtended>::SharedPtr nav_sat_fix_extended_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr node_management_timer_;

// Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// Internal variables
  waywiser_core::msg::NavSatFixExtended nav_sat_fix_extended_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfsToNavsatfixfused>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
