#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <cmath>
#include <mutex>

class OctomapNode : public rclcpp::Node
{
public:
  OctomapNode()
  : Node("octomap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {

    // Declare parameters with default values
    this->declare_parameter<std::string>("map_frame", "odom");
    this->declare_parameter<std::string>("cloud_topic", "sensors/camera/color/points");
    this->declare_parameter<double>("tf_timeout_sec", 0.5);
    this->declare_parameter<double>("resolution", 0.10);
    this->declare_parameter<double>("max_range", 5.0);
    this->declare_parameter<bool>("enable_degrade", true);
    this->declare_parameter<double>("degrade_timeout", 2.0);
    this->declare_parameter<int>("degrade_every_n_scans", 5);

    // Retrieve parameters
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("resolution", resolution_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("enable_degrade", enable_degrade_);
    this->get_parameter("degrade_timeout", degrade_timeout_);
    this->get_parameter("degrade_every_n_scans", degrade_every_n_scans_);

    // Initialise OcTreeStamped
    octree_ = std::make_unique<octomap::OcTreeStamped>(resolution_);
    // OctoMap sensor model parameters (tune if needed to improve map quality)
    octree_->setProbHit(0.7);                   // probability for occupied voxel (default: 0.7)
    octree_->setProbMiss(0.4);                  // probability for free voxel (default: 0.4)
    octree_->setClampingThresMin(0.1192);       // minimum occupancy probability (default: 0.1192)
    octree_->setClampingThresMax(0.971);        // maximum occupancy probability (default: 0.971)

    // Subscribe to PointCloud2 published by the OAK-D driver
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OctomapNode::pointcloudCallback, this, std::placeholders::_1));

    // Publisher for RViz visualisation
    auto qos = rclcpp::QoS(1).transient_local().reliable();      // Cache only the latest map for late-joining RViz instances
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", qos);

    RCLCPP_INFO(
      this->get_logger(),
      "Octomap node initialised with parameters: resolution=%.2f, max_range=%.2f, degrade_timeout=%.2f, enable_degrade=%s",
      resolution_, max_range_, degrade_timeout_, enable_degrade_ ? "true" : "false");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)       // Runs on every PointCloud2 arrival
  {
    try {
      // Transform PointCloud2 to target frame
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));

      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(*msg, transformed_cloud, transform_stamped);

      // Convert PointCloud2 to OctoMap Pointcloud
      octomap::Pointcloud cloud = toOctoCloud(transformed_cloud);

      // Compute sensor origin
      octomap::point3d sensor_origin(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);

      {         // Insert point cloud into OcTreeStamped
        std::lock_guard<std::mutex> lock(map_mutex_);
        octree_->insertPointCloud(
          cloud, sensor_origin, max_range_, false /*lazy_eval*/,
          false /*discretise*/);

        // Degrate outdated nodes every N scans for dynamic updates
        if (enable_degrade_ && degrade_timeout_ > 0.0) {
          const uint32_t c = ++scan_count_;

          if (c % static_cast<uint32_t>(degrade_every_n_scans_) == 0) {
            octree_->degradeOutdatedNodes(static_cast<unsigned>(std::round(degrade_timeout_)));
            octree_->updateInnerOccupancy();                // Ensure leaf node changes are propagated through the tree's inner nodes
          }
        }
      }         // lock released

      publishOctomap(transform_stamped.header.stamp);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  static octomap::Pointcloud toOctoCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    octomap::Pointcloud octo_cloud;

    octo_cloud.reserve(cloud.width * cloud.height);      // Pre-allocate memory for efficiency

    // Iterate through PointCloud2's XYZ fields (assumes float32 data type)
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(
      cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {      // Advance all iterators (same length) in lockstep
      const float x = *iter_x, y = *iter_y, z = *iter_z;        // dereference iterators to get point coordinates
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;                                                                       // Filter out points with NaN or Inf coordinates

      }
      octo_cloud.push_back(x, y, z);        // Appends valid points to OctoMap Pointcloud
    }

    return octo_cloud;
  }

  void publishOctomap(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);       // Lock while serialising to ensure a consistent view of the tree

    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.frame_id = map_frame_;        // Declare which TF frame the map is expressed in
    octomap_msg.header.stamp = stamp;

    if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {        // Serialise tree into the octomap_msg
      octomap_pub_->publish(octomap_msg);
    }
  }

  // Safe initialisers for cached values (before parameters are loaded)
  std::string map_frame_;
  std::string cloud_topic_;
  double tf_timeout_sec_;
  double resolution_;
  double max_range_;
  bool enable_degrade_;
  double degrade_timeout_;
  int degrade_every_n_scans_;
  uint32_t scan_count_{0};

  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

  // OcTree storage
  std::unique_ptr<octomap::OcTreeStamped> octree_;      // unique_ptr ensures the OcTree is solely owned by this node
  std::mutex map_mutex_;    // Mutex to protect octree_ during updates and serialisation
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapNode>();

  rclcpp::executors::MultiThreadedExecutor exec;    // Lets parameter services run on a different thread while a sensor callback is busy
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
