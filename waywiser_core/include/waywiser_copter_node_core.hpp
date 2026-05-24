#ifndef WAYWISER_COPTER_NODE_CORE_HPP_
#define WAYWISER_COPTER_NODE_CORE_HPP_

#include <array>
#include <memory>
#include <string>

#include <QObject>
#include <urdf/model.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/actuator_armed.hpp"
#include "px4_msgs/msg/health_report.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "waywiser_core/msg/battery_state.hpp"
#include "waywiser_core/msg/mission_state.hpp"
#include "waywiser_core/msg/nav_sat_fix_extended.hpp"
#include "waywiser_core/msg/path_with_twists.hpp"
#include "waywiser_core/msg/quadcopter_state.hpp"
#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"

#include "WayWise/vehicles/copterstate.h"
#include "copter_autopilot_component.hpp"
#include "copter_interface_component.hpp"
#include "qobject_node.hpp"

using namespace std::placeholders;

class WaywiserCopter : public QObjectNode
{
  Q_OBJECT

public:
  WaywiserCopter(
    const std::string & node_name = "waywiser_copter_node",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : QObjectNode(node_name, options) {}

  virtual ~WaywiserCopter() = default;

  enum class HighLevelState : uint8_t
  {
    STARTING_UP = waywiser_core::msg::QuadcopterState::STARTING_UP,
    READY_TO_ARM = waywiser_core::msg::QuadcopterState::READY_TO_ARM,
    ARMING = waywiser_core::msg::QuadcopterState::ARMING,
    ARMED = waywiser_core::msg::QuadcopterState::ARMED,
    IN_FLIGHT = waywiser_core::msg::QuadcopterState::IN_FLIGHT,
    LANDING = waywiser_core::msg::QuadcopterState::LANDING,
    EMERGENCY = waywiser_core::msg::QuadcopterState::EMERGENCY,
    LIFTING_OFF = waywiser_core::msg::QuadcopterState::LIFTING_OFF,
    HOVERING = waywiser_core::msg::QuadcopterState::HOVERING,
    IDLE_DESCENT = waywiser_core::msg::QuadcopterState::IDLE_DESCENT,
    AUTO_LIFTING_OFF = waywiser_core::msg::QuadcopterState::AUTO_LIFTING_OFF,
    ON_MISSION = waywiser_core::msg::QuadcopterState::ON_MISSION,
    RETURNING_HOME = waywiser_core::msg::QuadcopterState::RETURNING_HOME
  };

  void initialize_node();

protected:
  void setup_parameters();
  void setup_publishers();
  void setup_subscribers();
  void setup_timers();

  void node_management_timer_callback();
  void emergency_stop_status_callback(
    const waywiser_twist_safety::msg::EmergencyStopState::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void px4_actuator_armed_callback(const px4_msgs::msg::ActuatorArmed::SharedPtr msg);
  void px4_health_report_callback(const px4_msgs::msg::HealthReport::SharedPtr msg);
  void px4_vehicle_land_detected_callback(
    const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void px4_vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void px4_vehicle_local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void px4_vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void arm_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void autopilot_state_control_callback(const std_msgs::msg::Bool::SharedPtr bool_msg);
  void mission_status_callback(const waywiser_core::msg::MissionState::SharedPtr msg);
  void control_tower_heartbeat_callback(const std_msgs::msg::Header::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void path_with_twists_callback(const waywiser_core::msg::PathWithTwists::SharedPtr msg);
  void fused_nav_sat_fix_extended_callback(
    const waywiser_core::msg::NavSatFixExtended::SharedPtr msg);
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);

  void request_arm_state(bool arm);
  void send_arm_command(bool arm);
  void send_offboard_mode_command();
  void send_return_home_command();
  void process_twist_msg(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void publish_command();
  void update_control_tower_heartbeat_failsafe();
  void refresh_in_flight_status();
  void publish_odom();
  void publish_quadcopter_state();
  void publish_tfs();
  void publish_world_pose();
  void publish_route_markers();
  void publish_autopilot_markers();
  static void qtMessageHandler(
    QtMsgType type, const QMessageLogContext &, const QString & msg);

  // PX4 offboard setpoint bridge
  void px4_vehicle_command_ack_callback(
    const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void setpoint_timer_callback();
  void publish_offboard_control_mode(bool use_position_hold);
  void publish_trajectory_setpoint();
  bool local_position_ready() const;
  bool local_position_stable();
  bool command_requests_motion();
  bool engagement_requested();
  bool is_offboard_px4() const;

  rcl_interfaces::msg::SetParametersResult on_parameter_set(
    const std::vector<rclcpp::Parameter> & parameters);

  std::string urdf_file_;
  std::string frame_prefix_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string world_frame_;

  std::string odom_topic_;
  std::string input_odom_topic_;
  std::string arm_command_topic_;
  std::string fused_nav_sat_fix_extended_topic_;
  std::string vehicle_pose_topic_;
  std::string range_topic_;
  std::string quadcopter_state_topic_;
  std::string battery_state_topic_;
  std::string emergency_stop_status_topic_;
  std::string emergency_stop_update_topic_;
  std::string autopilot_state_control_topic_;
  std::string mission_status_topic_;
  std::string control_tower_heartbeat_topic_;
  std::string joint_states_topic_;

  bool enable_autopilot_component_ = false;
  bool enable_px4_bridge_ = true;
  bool publish_odom_to_baselink_tf_ = true;
  bool publish_world_to_odom_tf_ = false;
  bool received_first_odom_msg_ = false;
  float in_flight_range_threshold_ = 0.15F;
  float min_steering_height_ = 0.5F;
  geometry_msgs::msg::Twist current_cmd_vel_out_;

  llh_t enuref_;

  rclcpp::Publisher<waywiser_core::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<waywiser_core::msg::QuadcopterState>::SharedPtr quadcopter_state_pub_;
  rclcpp::Publisher<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr
    emergency_stop_update_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<waywiser_core::msg::MissionState>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr autopilot_marker_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_command_sub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorArmed>::SharedPtr px4_actuator_armed_sub_;
  rclcpp::Subscription<px4_msgs::msg::HealthReport>::SharedPtr px4_health_report_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    px4_vehicle_land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    px4_vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr px4_vehicle_command_ack_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<waywiser_core::msg::NavSatFixExtended>::SharedPtr
    fused_nav_sat_fix_extended_sub_;
  rclcpp::Subscription<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr
    emergency_stop_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autopilot_state_control_sub_;
  rclcpp::Subscription<waywiser_core::msg::MissionState>::SharedPtr mission_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr control_tower_heartbeat_sub_;
  rclcpp::Subscription<waywiser_core::msg::PathWithTwists>::SharedPtr path_with_twists_sub_;

  rclcpp::TimerBase::SharedPtr node_management_timer_;
  rclcpp::TimerBase::SharedPtr autopilot_state_machine_timer_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  QSharedPointer<CopterState> mCopterState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<CopterInterfaceComponent> mCopterInterfaceComponent;
  QSharedPointer<CopterAutopilotComponent> mCopterAutopilotComponent;
  QSharedPointer<urdf::Model> mUrdfModel;
  bool has_px4_altitude_ = false;
  float latest_px4_altitude_ = 0.0F;
  bool in_flight_ = false;
  bool px4_ground_contact_ = false;
  bool px4_maybe_landed_ = false;
  bool px4_landed_ = false;
  bool ready_for_takeoff_ = false;
  bool ready_for_offboard_ = false;
  bool ready_to_arm_ = false;
  bool armed_ = false;
  bool force_arm_ = false;
  bool px4_dist_bottom_valid_ = false;
  float px4_dist_bottom_ = 0.0F;
  bool received_range_data_ = false;
  bool has_px4_vertical_velocity_ = false;
  float latest_px4_vertical_velocity_ = 0.0F;
  uint64_t px4_health_warning_flags_ = 0;
  uint64_t px4_health_error_flags_ = 0;
  uint64_t px4_arming_check_warning_flags_ = 0;
  uint64_t px4_arming_check_error_flags_ = 0;
  bool preflight_all_pass_logged_ = false;
  int64_t last_preflight_failure_log_time_ns_ = 0;

  bool auto_arm_enabled_ = true;
  bool auto_landing_active_ = false;
  bool hover_hold_on_idle_ = true;
  HighLevelState current_state_ = HighLevelState::STARTING_UP;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  bool received_active_mission_status_ = false;
  rclcpp::Time last_active_mission_status_time_;
  double mission_state_timeout_ = 1.0;

  bool return_home_on_control_tower_timeout_ = true;
  double control_tower_heartbeat_timeout_ = 2.0;
  double return_home_command_retry_period_ = 2.0;
  bool received_control_tower_heartbeat_ = false;
  bool control_tower_timeout_return_home_active_ = false;
  bool has_last_return_home_request_time_ = false;
  rclcpp::Time last_control_tower_heartbeat_time_;
  rclcpp::Time last_return_home_request_time_;

  bool last_arm_request_value_ = false;
  double last_arm_request_time_ = 0.0;
  double last_input_command_time_ = 0.0;
  const double arm_request_debounce_period_ = 1.0;

  // Extended local position state (for setpoint publishing)
  bool has_vehicle_local_position_ = false;
  float vehicle_heading_ = 0.0F;
  float local_position_x_ = 0.0F;
  float local_position_y_ = 0.0F;
  float local_position_z_ = 0.0F;
  bool local_position_xy_valid_ = false;
  bool local_position_z_valid_ = false;
  bool local_position_v_xy_valid_ = false;
  bool local_position_v_z_valid_ = false;
  bool has_local_position_ready_since_ = false;
  rclcpp::Time local_position_ready_since_;
  uint8_t px4_nav_state_ = 0;

  // Offboard setpoint streaming state
  int setpoint_count_ = 0;
  int required_setpoint_count_ = 40;
  bool was_command_active_ = false;
  bool has_hold_position_ = false;
  std::array<float, 3> hold_position_ned_{};
  bool has_last_mode_request_time_ = false;
  rclcpp::Time last_mode_request_time_;

  // Offboard bridge parameters
  double command_timeout_ = 0.5;
  double setpoint_rate_ = 20.0;
  double offboard_prestream_duration_ = 1.5;
  bool require_local_position_before_arm_ = false;
  double local_position_ready_duration_ = 1.0;
  double hold_velocity_epsilon_ = 1e-4;
  double idle_descent_rate_ = 0.5;
  bool auto_offboard_ = false;
  bool require_motion_before_engage_ = true;
  double request_retry_period_ = 1.0;
  bool publish_waypoint_markers_ = false;  // disabled by default; control tower draws its own route

  static rclcpp::Logger node_logger_;
};

#endif  // WAYWISER_COPTER_NODE_CORE_HPP_
