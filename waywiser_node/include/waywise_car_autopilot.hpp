#ifndef WAYWISE_CAR_AUTOPILOT_HPP_
#define WAYWISE_CAR_AUTOPILOT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QFile>
#include <QObject>
#include <QString>
#include <QXmlStreamReader>
#include <Eigen/Geometry>

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/autopilot/followpoint.h"
#include "WayWise/sensors/gnss/gnssreceiver.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavsdk/mavsdk.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "urdf/model.h"
#include "visualization_msgs/msg/marker_array.hpp"

class WaywiseCarAutopilot : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WaywiseCarAutopilot(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_car_autopilot_node")
  : QObject(), Node(node_name, options) {}

  virtual ~WaywiseCarAutopilot() = default;

  virtual void initialize_node();

protected:
  virtual void setup_parameters();
  virtual void setup_publishers();
  virtual void setup_subscribers();
  virtual void setup_timers();
  virtual void setup_autopilot();
  void setup_autopilot(QSharedPointer<CarState> carState);
  virtual void provide_parameters_to_parameter_server();

  // Callback methods
  void autopilot_timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void enu_reference_callback(const geometry_msgs::msg::Vector3::SharedPtr enuRef_msg);
  void autopilot_state_control_callback(const std_msgs::msg::Bool::SharedPtr bool_msg);

  // Utility methods
  virtual void update_world_positon(geometry_msgs::msg::Pose world_pose);
  bool loadURDFFile();
  Eigen::Vector3d getLinkPosition(
    const urdf::Model & urdfModel,
    const std::string & link_name) const;
  virtual double update_joint_states_msg(
    sensor_msgs::msg::JointState & joint_state_msg,
    double timePassed_ms);
  QList<PosPoint> read_route_from_XMLFile(const std::string xml_filepath_);
  void start_waypoint_follower(QList<PosPoint> & waypointList);
  void stop_waypoint_follower();
  void update_waypoint_follower_route(QList<PosPoint> & waypointList);
  void publish_route_markers();

  // ROS parameters
  std::string odom_topic_;
  float speed_to_erpm_factor_;
  float length_, width_, wheelbase_, min_turning_radius_;
  int autopilot_cmd_publish_rate_;
  std::string waywise_control_tower_address_;
  int waywise_control_tower_port_;
  float purepursuit_radius_;
  bool update_world_position_with_odom_;
  bool update_world_position_with_tf_;
  float standstill_velocity_threshold_;
  float max_angular_velocity_;
  bool publish_joint_states_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string world_frame_;
  std::string rear_axle_frame_;
  std::string center_frame_;
  std::string rear_end_frame_;

  std::string enu_refernce_topic_;
  std::string vehicle_pose_topic_;

  std::string urdf_file_;
  std::vector<std::string> front_steering_joint_names_;
  std::vector<std::string> front_wheel_joint_names_;
  std::vector<std::string> rear_wheel_joint_names_;

  std::string preplanned_route_filepath_;
  std::string autopilot_state_control_topic_;
  bool start_with_autopilot_;
  float desired_linear_velocity_;
  std::string mission_status_topic_;
  int end_goal_alignment_type_;
  std::string vehicle_alignment_reference_point_topic_;
  std::string autopilot_center_pose_topic_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    vehicle_alignment_reference_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr autopilot_center_pose_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr enu_refernce_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autopilot_state_control_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr autopilot_timer_;

  // Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<GNSSReceiver> mGNSSReceiver;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;
  QSharedPointer<FollowPoint> mFollowPoint;
  QList<PosPoint> mWaypointList;

  // Internal variables
  urdf::Model urdfModel;
  bool is_on_mission_ = false;
  bool received_first_odom_msg_ = false;
  bool waiting_for_a_route_ = false;
};

#endif  // WAYWISE_CAR_AUTOPILOT_HPP_
