#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <mutex>
#include <cmath>
#include <memory>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"
#include "waywiser/waywiser_utils.hpp"

class OctomapOcclusionsNode : public rclcpp::Node
{
public:
  OctomapOcclusionsNode()
  : Node("octomap_occlusions_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("octomap_topic", "octomap_full");
    this->declare_parameter<std::string>("frame_id", "camera_link");
    this->declare_parameter<double>("tf_timeout_sec", 0.5);

    // Occlusion detection parameters
    this->declare_parameter<double>("volume_threshold", 1.2);
    this->declare_parameter<int>("horizontal_steps", 20);
    this->declare_parameter<int>("vertical_steps", 15);

    // frustum parameters
    this->declare_parameter<double>("horizontal_fov", 127.0);
    this->declare_parameter<double>("vertical_fov", 79.5);
    this->declare_parameter<double>("min_range", 0.4);
    this->declare_parameter<double>("max_range", 5.0);

    // Retrieve parameters
    this->get_parameter("octomap_topic", octomap_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("volume_threshold", volume_threshold_);
    this->get_parameter("horizontal_steps", horizontal_steps_);
    this->get_parameter("vertical_steps", vertical_steps_);
    this->get_parameter("horizontal_fov", horizontal_fov_);
    this->get_parameter("vertical_fov", vertical_fov_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("max_range", max_range_);

    // Subscribe to OctoMap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_,
      rclcpp::QoS(1).transient_local().reliable(),          // Keep only the latest message in the buffer
      std::bind(&OctomapOcclusionsNode::octomapCallback, this, std::placeholders::_1)
    );

    // Initialise emergency stop publisher
    emergency_stop_publisher_ =
      this->create_publisher<waywiser_twist_safety::msg::EmergencyStopState>(
      "/emergency_stop/target_state",
      QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS
      );

    // Initialise emergency stop message
    emergency_stop_msg_.sender_id = "octomap_occlusions";
    emergency_stop_msg_.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;

    RCLCPP_INFO(
      this->get_logger(),
      "OctomapOcclusionsNode initialised with threshold: %.2f m³",
      volume_threshold_);
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    double known_volume = 0.0;

    try {
      transform_stamped = tf_buffer_.lookupTransform(
        msg->header.frame_id,           // target
        frame_id_,                      // source
        msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_)
      );

      const octomap::point3d sensor_origin(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
      );

      // Extract forward direction
      tf2::Quaternion q;
      tf2::fromMsg(transform_stamped.transform.rotation, q);        // Convert geometry_msgs quaternion to tf2::Quaternion

      tf2::Vector3 forward_vector(1, 0, 0);         // X, Y, Z (in camera frame)

      // Rotate forward vector by quaternion to get the camera's actual orientation in map frame
      tf2::Vector3 tf_forward = tf2::quatRotate(q, forward_vector);
      octomap::point3d direction(tf_forward.x(), tf_forward.y(), tf_forward.z());       // Convert back to octomap::point3d
      direction = direction.normalized();

      if (frustum_volume_uninitialised_) {  // Compute sensor height above base_link (assumes flat ground)
        auto base_cam_tf = tf_buffer_.lookupTransform(
          "base_link",
          frame_id_,
          msg->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_)
        );

        const double sensor_height = base_cam_tf.transform.translation.z;

        frustum_volume_ = computeFrustumVolumeAboveGround(sensor_height);
        frustum_volume_uninitialised_ = 0;  // Set flag to avoid recomputing
      }

      {
        std::lock_guard<std::mutex> lock(map_mutex_);

        auto * abstract_tree = octomap_msgs::msgToMap(*msg);
        auto * octree_stamped = dynamic_cast<octomap::OcTreeStamped *>(abstract_tree);

        if (!octree_stamped) {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to convert OctoMap message (not an OcTreeStamped)");
          delete abstract_tree;            // Clean up allocated memory
          return;
        }

        octree_.reset(octree_stamped);

        if (octree_->size() == 0) {
          RCLCPP_INFO(this->get_logger(), "OcTree is empty");
          return;
        }

        // Fill frustum with free space
        fillFrustumWithFreeSpace(sensor_origin, direction);

        // Update occupancy of all inner nodes after filling
        octree_->updateInnerOccupancy();

        known_volume = calculateKnownVolume();
      } // lock released

      const double unknown_volume = std::max(0.0, frustum_volume_ - known_volume);
      if (unknown_volume > volume_threshold_) {
        emergency_stop_msg_.stamp = this->get_clock()->now();
        emergency_stop_msg_.reason = std::to_string(unknown_volume) + "m³ out of " + std::to_string(
          frustum_volume_) + "m³ is unknown (threshold: " + std::to_string(volume_threshold_) +
          "m³).";
        emergency_stop_publisher_->publish(emergency_stop_msg_);

        RCLCPP_INFO(
          this->get_logger(),
          "Emergency stop triggered — %.2fm³ out of %.2fm³ is unknown (threshold: %.2fm³)",
          unknown_volume,
          frustum_volume_,
          volume_threshold_
        );
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  double computeFrustumVolumeAboveGround(double sensor_height) const  // assumes camera is not tilted
  {
    // Convert degrees to radians
    const double horizontal_fov_rad = horizontal_fov_ * M_PI / 180.0;
    const double vertical_fov_rad = vertical_fov_ * M_PI / 180.0;

    // Compute tangents
    const double tan_half_h = tan(horizontal_fov_rad * 0.5);
    const double tan_half_v = tan(vertical_fov_rad * 0.5);

    // Frustum dimensions at min/max range
    const double min_width = 2.0 * min_range_ * tan_half_h;
    const double min_height = 2.0 * min_range_ * tan_half_v;
    const double max_width = 2.0 * max_range_ * tan_half_h;
    const double max_height = 2.0 * max_range_ * tan_half_v;

    // Frustum base and top areas
    const double min_area = min_width * min_height;
    const double max_area = max_width * max_height;

    // Theoretical pyramid frustum volume (in an empty space)
    const double height = max_range_ - min_range_;
    const double total_volume = (1.0 / 3.0) * height *
      (min_area + max_area + std::sqrt(min_area * max_area));

    // Compute potential subground volume
    double subground_volume = 0.0;

    // Find distance r where bottom edge of frustum meets ground plane
    const double r_ground_intersect = sensor_height / tan_half_v;

    // Define the integration range
    const double r_start = std::max(min_range_, r_ground_intersect);
    const double r_end = max_range_;

    // Compute subground volume via definite integral: F(r_end) - F(r_start)
    if (r_start < r_end) {  // If r_start >= ray_end the frustum is entirely above ground
      /*
      The area below ground at any distance r is:
        A(r) = Width(r) x SubgroundHeight(r)

      Where:
        Width(r) = 2 * r * tan_half_h
        SubgroundHeight(r) = HeightBottom(r) - sensor_height = r * tan_half_v - sensor_height

      Therefore:
        A(r) = (2 * r * tan_half_h) * (r * tan_half_v - sensor_height)

      The total subground volume is the integral of A(r) from r_start to r_end.
      */

      subground_volume =
        calculateSubgroundIntegral(
        r_end, tan_half_h, tan_half_v,
        sensor_height) - calculateSubgroundIntegral(r_start, tan_half_h, tan_half_v, sensor_height);
    }

    return std::max(0.0, total_volume - subground_volume);
  }

  // Helper function to calculate the indefinite integral of the subground volume
  static double calculateSubgroundIntegral(
    double r, double tan_half_h, double tan_half_v,
    double sensor_height)
  {
    // V(r) = (2/3 * tan_half_h * tan_half_v * r^3) - (tan_half_h * sensor_height * r^2)

    const double r_cubed = r * r * r;
    const double r_squared = r * r;

    double expansion_term = (2.0 / 3.0) * tan_half_h * tan_half_v * r_cubed;
    double offset_term = tan_half_h * sensor_height * r_squared;

    return expansion_term - offset_term;
  }

  // Cast rays at regular intervals to fill the theoretical frustum volume with free space
  void fillFrustumWithFreeSpace(
    const octomap::point3d & sensor_origin,
    const octomap::point3d & direction) const
  {
    for (int i = 0; i <= horizontal_steps_; i++) {
      const double horizontal_angle = -half_horizontal_fov_ +
        (horizontal_fov_ * i / horizontal_steps_);
      const double horizontal_angle_rad = horizontal_angle * M_PI / 180.0;

      for (int j = 0; j <= vertical_steps_; j++) {
        const double vertical_angle = -half_vertical_fov_ + (vertical_fov_ * j / vertical_steps_);
        const double vertical_angle_rad = vertical_angle * M_PI / 180.0;

        // Calculate ray end point
        octomap::point3d ray_dir = direction;
        ray_dir.rotate_IP(0, vertical_angle_rad, horizontal_angle_rad);         // Rotate vector around Roll (X), Pitch, Yaw (Z)

        const octomap::point3d ray_start = sensor_origin + ray_dir * min_range_;
        const octomap::point3d ray_end = sensor_origin + ray_dir * max_range_;

        // Insert ray as free space
        octree_->insertRay(
          ray_start,
          ray_end,
          false // lazy_eval false for immediate update
        );
      }
    }
  }

  // Iterate through leaf node voxels containing occupancy information
  double calculateKnownVolume() const
  {
    double known_volume = 0.0;

    for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
      const double voxel_size = it.getSize();
      const double voxel_volume = voxel_size * voxel_size * voxel_size;

      known_volume += voxel_volume;
    }
    return known_volume;
  }

  // Safe initialisers for cached values (before parameters are loaded)
  std::string octomap_topic_ {"/octomap_full"};
  std::string frame_id_ {"camera_link"};
  double tf_timeout_sec_ {0.5};
  double volume_threshold_ {1.2};
  int horizontal_steps_{20};
  int vertical_steps_{15};
  double horizontal_fov_{127.0};
  double vertical_fov_{79.5};
  double min_range_{0.4};
  double max_range_{5.0};
  double half_horizontal_fov_{horizontal_fov_ * 0.5};
  double half_vertical_fov_{vertical_fov_ * 0.5};
  double frustum_volume_{0.0};
  int frustum_volume_uninitialised_{1};  // Flag to only compute frustum volume once

  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS components
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Octree storage
  std::unique_ptr<octomap::OcTreeStamped> octree_;
  std::mutex map_mutex_;

  // Emergency stop
  rclcpp::Publisher<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr
    emergency_stop_publisher_;
  waywiser_twist_safety::msg::EmergencyStopState emergency_stop_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapOcclusionsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
