#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/sensors/gnss/gnssreceiver.h"

#include "waywiser/waywiser_utils.hpp"
#include "waywiser_core/msg/nav_sat_fix_extended.hpp"

using namespace std::placeholders;

class NavSatFixExtendedWrapper : public rclcpp::Node
{
public:
  NavSatFixExtendedWrapper(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "navsatfix_extended_wrapper_node")
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
      RCLCPP_WARN(this->get_logger(), "Receiving /clock msgs now.");
    }

    // ROS parameters
    frame_prefix_ = this->declare_parameter("frame_prefix", "/");
    conversion_mode_ = this->declare_parameter("conversion_mode", "tf");
    std::string frame_id_override_base = this->declare_parameter("frame_id_override", "");
    if (!frame_id_override_base.empty()) {
      if (frame_id_override_base != "/") {
        frame_id_override_ = frame_prefix_ + frame_id_override_base;
      } else {
        frame_id_override_ = frame_prefix_;
        if (!frame_id_override_.empty() && frame_id_override_.back() == '/') {
          frame_id_override_.pop_back();
        }
      }
    }
    nav_sat_fix_extended_topic_ = this->declare_parameter(
      "nav_sat_fix_extended_topic", "/nav_sat_fix_extended");

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

    // Publishers
    nav_sat_fix_extended_pub_ = this->create_publisher<waywiser_core::msg::NavSatFixExtended>(
      nav_sat_fix_extended_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);

    // Prepare nav_sat_fix_extended_msg
    if (!frame_id_override_.empty()) {
      nav_sat_fix_extended_msg_.header.frame_id = frame_id_override_;
    }
    nav_sat_fix_extended_msg_.fix_type = gnss_fix_type_;
    nav_sat_fix_extended_msg_.horizontal_accuracy = gnss_horizontal_accuracy_;
    nav_sat_fix_extended_msg_.vertical_accuracy = gnss_vertical_accuracy_;
    nav_sat_fix_extended_msg_.heading_accuracy = gnss_heading_accuracy_;
    nav_sat_fix_extended_msg_.last_rtcm_correction_age = gnss_last_rtcm_correction_age_;
    nav_sat_fix_extended_msg_.num_satellites = gnss_num_satellites_;

    // Initialize based on conversion mode
    if (conversion_mode_ == "tf") {
      initialize_tf_mode();
      nav_sat_fix_extended_msg_.is_fused_on_chip = true;
    } else if (conversion_mode_ == "navsatfix") {
      initialize_navsatfix_mode();
      nav_sat_fix_extended_msg_.is_fused_on_chip = false;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Invalid conversion_mode: '%s'. Must be 'tf' or 'navsatfix'",
        conversion_mode_.c_str());
      throw std::runtime_error("Invalid conversion_mode parameter");
    }

    RCLCPP_INFO(
      get_logger(), "%s is initialized in '%s' mode!",
      this->get_name(), conversion_mode_.c_str());
  }

private:
  void initialize_tf_mode()
  {
    world_frame_ = this->declare_parameter("world_frame", "map");
    tf_object_frame_ = this->declare_parameter("tf_object_frame", "base_link");
    gnss_message_rate_ = this->declare_parameter("gnss_message_rate", 10);

    auto vector3_param = RosUtils::get_vector3_param(this, "enuref");
    if (vector3_param) {
      enuref_ = vector3_param->to_type<llh_t>();
    }

    if (frame_id_override_.empty()) {
      nav_sat_fix_extended_msg_.header.frame_id = tf_object_frame_;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timers
    node_management_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(), // uses sim time if enabled
      std::chrono::milliseconds(1000 / gnss_message_rate_),
      std::bind(&NavSatFixExtendedWrapper::tf_timer_callback, this)
    );

    RCLCPP_INFO(
      get_logger(), "TF mode: converting %s->%s to NavSatFixExtended",
      world_frame_.c_str(), tf_object_frame_.c_str());
  }

  void initialize_navsatfix_mode()
  {
    nav_sat_fix_input_topic_ = this->declare_parameter(
      "nav_sat_fix_input_topic", "/nav_sat_fix");

    nav_sat_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      nav_sat_fix_input_topic_, 10,
      std::bind(&NavSatFixExtendedWrapper::navsat_fix_callback, this, _1));

    RCLCPP_INFO(
      get_logger(), "NavSatFix mode: converting %s to NavSatFixExtended",
      nav_sat_fix_input_topic_.c_str());
  }

  // Callback methods
  void tf_timer_callback()
  {
    xyz_t xyz;
    double rollRad, pitchRad, yawRad;
    static bool transform_warning_logged = false;
    try {
      geometry_msgs::msg::TransformStamped map_to_object_msg_tfs = tf_buffer_->lookupTransform(
        world_frame_, tf_object_frame_, this->get_clock()->now(), tf2::durationFromSec(1.0));
      xyz.x = map_to_object_msg_tfs.transform.translation.x;
      xyz.y = map_to_object_msg_tfs.transform.translation.y;
      xyz.z = map_to_object_msg_tfs.transform.translation.z;
      tf2::Quaternion q_map_to_object;
      tf2::fromMsg(map_to_object_msg_tfs.transform.rotation, q_map_to_object);
      q_map_to_object.normalize();
      tf2::Matrix3x3(q_map_to_object).getRPY(rollRad, pitchRad, yawRad);
      if (transform_warning_logged) {
        RCLCPP_WARN(
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
    nav_sat_fix_extended_msg_.header.stamp = this->now();

    llh_t llh = coordinateTransforms::enuToLlh(enuref_, xyz);
    nav_sat_fix_extended_msg_.latitude = llh.latitude;   // Latitude in degrees
    nav_sat_fix_extended_msg_.longitude = llh.longitude;   // Longitude in degrees
    nav_sat_fix_extended_msg_.altitude = llh.height;   // Altitude in meters
    nav_sat_fix_extended_msg_.roll = rollRad * RAD2DEG;
    nav_sat_fix_extended_msg_.pitch = -pitchRad * RAD2DEG; // negative due to ENU to NED conversion
    nav_sat_fix_extended_msg_.yaw = coordinateTransforms::yawENUtoNED(yawRad * RAD2DEG);
    nav_sat_fix_extended_pub_->publish(nav_sat_fix_extended_msg_);
  }

  void navsat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // Copy standard NavSatFix data to NavSatFixExtended
    nav_sat_fix_extended_msg_.header = msg->header;
    if (!frame_id_override_.empty()) {
      nav_sat_fix_extended_msg_.header.frame_id = frame_id_override_;  // Override frame_id
    }
    nav_sat_fix_extended_msg_.latitude = msg->latitude;
    nav_sat_fix_extended_msg_.longitude = msg->longitude;
    nav_sat_fix_extended_msg_.altitude = msg->altitude;

    nav_sat_fix_extended_pub_->publish(nav_sat_fix_extended_msg_);
  }

// Parameters
  std::string frame_prefix_;
  std::string conversion_mode_;
  std::string world_frame_;
  std::string tf_object_frame_;
  std::string frame_id_override_;
  std::string nav_sat_fix_extended_topic_;
  std::string nav_sat_fix_input_topic_;
  int gnss_message_rate_;   // [Hz]
  llh_t enuref_;   // [lat, lon, height]
  uint8_t gnss_fix_type_;
  double gnss_horizontal_accuracy_;
  double gnss_vertical_accuracy_;
  double gnss_heading_accuracy_;
  uint8_t gnss_last_rtcm_correction_age_;
  uint8_t gnss_num_satellites_;

// Publishers
  rclcpp::Publisher<waywiser_core::msg::NavSatFixExtended>::SharedPtr nav_sat_fix_extended_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr node_management_timer_;

// Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// Internal variables
  waywiser_core::msg::NavSatFixExtended nav_sat_fix_extended_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavSatFixExtendedWrapper>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
