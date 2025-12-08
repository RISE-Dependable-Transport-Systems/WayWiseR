#ifndef WAYWISER_CAR_NODE_CORE_HPP_
#define WAYWISER_CAR_NODE_CORE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>
#include <QString>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "waywiser_core/msg/battery_state.hpp"
#include "waywiser_core/msg/car_control_command.hpp"
#include "waywiser_core/msg/mission_state.hpp"
#include "waywiser_core/msg/nav_sat_fix_extended.hpp"
#include "waywiser_core/msg/path_with_twists.hpp"
#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"

#include "car_autopilot_component.hpp"
#include "car_interface_component.hpp"
#include "waywiser_core_utils.hpp"
#include "waywiser/waywiser_utils.hpp"
#include "waywiser_description/waywiser_description_utils.hpp"

using namespace std::placeholders;

class WaywiserCar : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WaywiserCar(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywiser_car_node")
  : QObject(), Node(node_name, options) {}

  virtual ~WaywiserCar() = default;

  virtual void initialize_node();
  void initialize_node(
    QSharedPointer<CarState> carState,
    QSharedPointer<CarInterfaceComponent> carInterfaceComponent,
    QSharedPointer<CarAutopilotComponent> carAutopilotComponent);

signals:

protected:
  virtual void setup_parameters();
  virtual void setup_publishers();
  virtual void setup_subscribers();
  virtual void setup_timers();

  // Callback methods
  virtual void node_management_timer_callback();
  void autopilot_state_control_callback(const std_msgs::msg::Bool::SharedPtr bool_msg);
  void emergency_stop_status_callback(
    const waywiser_twist_safety::msg::EmergencyStopState::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg); // for EXT_SIMULATED interface
  virtual void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void path_with_twists_callback(const waywiser_core::msg::PathWithTwists::SharedPtr msg);
  void fused_nav_sat_fix_extended_callback(
    const waywiser_core::msg::NavSatFixExtended::SharedPtr msg);

  // Publish helper methods
  virtual void publish_odom();
  virtual void publish_tfs();
  virtual void publish_world_pose();
  void publish_route_markers();
  void publish_autopilot_markers();
  virtual void publish_joint_states(double timePassed_ms);

  // Utility methods
  virtual double update_joint_states_msg(
    sensor_msgs::msg::JointState & joint_state_msg, double timePassedSinceLastCall_ms);
  void process_twist_msg(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

  // ROS parameters
  std::string urdf_file_;
  std::vector<std::string> front_steering_joint_names_;
  std::vector<std::string> front_wheel_joint_names_;
  std::vector<std::string> rear_wheel_joint_names_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string world_frame_;
  std::string rear_axle_frame_; // this is vehicle reference point for waywise
  std::string chassis_frame_;
  std::string front_end_frame_;
  std::string rear_end_frame_;
  std::string left_end_frame_;
  std::string right_end_frame_;

  std::string battery_state_topic_;
  std::string odom_topic_;
  std::string fused_nav_sat_fix_extended_topic_;
  std::string vehicle_pose_topic_;
  std::string emergency_stop_update_topic_;
  std::string car_control_command_topic_;

  std::string mission_status_topic_;
  std::string vehicle_alignment_reference_point_topic_;
  std::string autopilot_center_pose_topic_;

  std::string emergency_stop_status_topic_;
  std::string autopilot_state_control_topic_;

  std::vector<std::string> tof_sensor_names_;
  std::map<std::string, std::string> tof_sensor_topics_;

  bool enable_autopilot_component_;
  bool enable_visualization_msgs_;
  // bool update_world_position_with_odom_;
  // bool update_world_position_with_tf_;
  bool publish_odom_to_baselink_tf_;
  bool publish_world_to_odom_tf_;
  bool waypoint_follower_bypass_mux_;
  bool invert_steering_feedback_from_odom_;
  int joint_states_publish_rate_;
  double wheel_diameter_;
  bool invert_steering_joint_state_;
  llh_t enuref_;   // [lat, lon, height]

  // Publishers
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<waywiser_core::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr
    emergency_stop_update_pub_;
  rclcpp::Publisher<waywiser_core::msg::CarControlCommand>::SharedPtr car_control_command_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> tof_pubs_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autopilot_twist_pub_;
  rclcpp::Publisher<waywiser_core::msg::MissionState>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    vehicle_alignment_reference_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr autopilot_center_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr autopilot_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Subscribers
  rclcpp::Subscription<waywiser_core::msg::NavSatFixExtended>::SharedPtr
    fused_nav_sat_fix_extended_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr
    emergency_stop_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autopilot_state_control_sub_;
  rclcpp::Subscription<waywiser_core::msg::PathWithTwists>::SharedPtr path_with_twists_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr node_management_timer_;
  rclcpp::TimerBase::SharedPtr autopilot_state_machine_timer_;

  // Transform buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;

  // Internal variables
  QSharedPointer<CarInterfaceComponent> mCarInterfaceComponent;
  QSharedPointer<CarAutopilotComponent> mCarAutopilotComponent;
  QSharedPointer<urdf::Model> mUrdfModel;
  bool received_first_odom_msg_ = false;
  double min_target_speed = 0.0;
  double max_target_speed = 0.0;
};

#endif  // WAYWISER_CAR_NODE_CORE_HPP_
