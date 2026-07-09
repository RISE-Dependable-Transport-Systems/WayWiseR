#include "waywiser_copter_node_core.hpp"
#include "moc_waywiser_copter_node_core.cpp"

#include <QDateTime>

#include <array>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "waywiser/waywiser_utils.hpp"
#include "copter_interface_component.hpp"

rclcpp::Logger WaywiserCopter::node_logger_ = rclcpp::get_logger("waywiser_copter_node");

namespace
{
constexpr float kVerticalMotionInAirThreshold = 0.35F;
constexpr float kLandingHeightMargin = 0.15F;
constexpr float kTakeoffHeightMargin = 0.10F;
constexpr double kReturnHomeLandingHorizontalSpeedThreshold = 0.15;
constexpr double kDefaultHomePositionTolerance = 2.0;
constexpr double kInputCommandThreshold = 0.001;
constexpr double kInputCommandTimeout = 0.25;

double steady_time_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

float constrained_descent_velocity(
  double target_descent_velocity,
  bool has_vertical_velocity,
  double vertical_velocity_up)
{
  const float target = std::max(0.0F, static_cast<float>(target_descent_velocity));
  if (!has_vertical_velocity || !std::isfinite(vertical_velocity_up)) {
    return target;
  }

  const float measured_descent = std::max(0.0F, static_cast<float>(-vertical_velocity_up));
  if (measured_descent <= target) {
    return target;
  }

  return std::clamp(2.0F * target - measured_descent, -target, target);
}

PosType parse_pos_type(const std::string & value)
{
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

  if (normalized == "fused") {
    return PosType::fused;
  }
  if (normalized == "gnss") {
    return PosType::GNSS;
  }
  if (normalized == "uwb") {
    return PosType::UWB;
  }
  if (normalized == "simulated") {
    return PosType::simulated;
  }
  return PosType::odom;
}

bool mission_state_is_route_active(MissionState state)
{
  switch (state) {
    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteClimb:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
    case MissionState::FollowRouteApproachingEndGoalZ:
      return true;
    default:
      return false;
  }
}

bool mission_state_is_return_home_active(MissionState state)
{
  switch (state) {
    case MissionState::ReturnHomeInit:
    case MissionState::ReturnHomeClimb:
    case MissionState::ReturnHomeCruising:
    case MissionState::ReturnHomeLanding:
      return true;
    default:
      return false;
  }
}

bool mission_state_is_active(MissionState state)
{
  return mission_state_is_route_active(state) ||
         mission_state_is_return_home_active(state);
}
}  // namespace

void WaywiserCopter::initialize_node()
{
  node_logger_ = this->get_logger();
  qInstallMessageHandler(qtMessageHandler);

  mCopterState.reset(new CopterState());
  mCopterInterfaceComponent.reset(new CopterInterfaceComponent(this, mCopterState));

  enable_autopilot_component_ = declare_parameter("enable_autopilot_component", true);
  mCopterAutopilotComponent.reset(new CopterAutopilotComponent(this, mCopterState));
  mRthAutopilotComponent.reset(new CopterAutopilotComponent(this, mCopterState));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  setup_parameters();
  mCopterInterfaceComponent->setup_vehicle_interface();
  mEmergencyStopState = mCopterInterfaceComponent->getEmergencyStopState();

  if (enable_autopilot_component_) {
    mCopterAutopilotComponent->setupAutopilot(mEmergencyStopState);
    if (mCopterAutopilotComponent->getEnableMavlinkInterface()) {
      mCopterAutopilotComponent->provideParametersToParameterServer();
    }
  }
  mRthAutopilotComponent->setupAutopilot(mEmergencyStopState);

  setup_publishers();
  setup_subscribers();
  setup_timers();
  heartbeat_monitor_start_time_ = get_clock()->now();
  heartbeat_monitor_started_ = true;

  if (mCopterAutopilotComponent) {
    mCopterAutopilotComponent->reset();
  }
  if (mRthAutopilotComponent) {
    mRthAutopilotComponent->reset();
  }
  mCopterInterfaceComponent->reset();

  RCLCPP_INFO(get_logger(), "%s is initialized!", this->get_name());
}

void WaywiserCopter::setup_parameters()
{
  CoreUtils::declare_read_only_waywise_object_type_param(this, mCopterState);

  frame_prefix_ = declare_parameter("frame_prefix", "");
  urdf_file_ = declare_parameter("urdf_file", "");
  mUrdfModel = URDFUtils::getURDFModel(urdf_file_);

  world_frame_ = declare_parameter("world_frame", "map");
  odom_frame_ = RosUtils::joinFrame(frame_prefix_, declare_parameter("odom_frame", "odom"));
  base_frame_ = RosUtils::joinFrame(frame_prefix_, declare_parameter("base_frame", "base_link"));

  odom_topic_ = declare_parameter("odom_topic", "/odometry");
  input_odom_topic_ = declare_parameter("input_odom_topic", odom_topic_);
  fused_nav_sat_fix_extended_topic_ = declare_parameter(
    "fused_nav_sat_fix_extended_topic", "/nav_sat_fix_extended");
  vehicle_pose_topic_ = declare_parameter("vehicle_pose_topic", "/copter_pose");
  home_pose_topic_ = declare_parameter("home_pose_topic", "home_pose");
  range_topic_ = declare_parameter("range_topic", "sensors/range/down");
  quadcopter_state_topic_ = declare_parameter("quadcopter_state_topic", "quadcopter_state");
  observed_quadcopter_state_topic_ =
    declare_parameter("observed_quadcopter_state_topic", std::string(""));
  battery_state_topic_ = declare_parameter("battery_state_topic", "/battery_state");
  emergency_stop_status_topic_ = declare_parameter(
    "emergency_stop_status_topic", "/emergency_stop/current_state");
  emergency_stop_update_topic_ = declare_parameter(
    "emergency_stop_update_topic", "/emergency_stop/target_state");
  autopilot_state_control_topic_ = declare_parameter(
    "autopilot_state_control_topic", "/autopilot_state_control");
  mission_status_topic_ = declare_parameter("mission_status_topic", "/mission_status");
  control_tower_heartbeat_topic_ =
    declare_parameter("control_tower_heartbeat_topic", "control_tower_heartbeat");
  control_tower_heartbeat_rx_state_topic_ = declare_parameter(
    "control_tower_heartbeat_rx_state_topic", "control_tower_heartbeat_rx_state");
  joint_states_topic_ = declare_parameter("joint_states_topic", "/joint_states");

  publish_odom_to_baselink_tf_ = declare_parameter("publish_odom_to_baselink_tf", true);
  publish_world_to_odom_tf_ = declare_parameter("publish_world_to_odom_tf", false);
  in_flight_range_threshold_ = declare_parameter("in_air_range_threshold", 0.2);
  min_steering_height_ = declare_parameter("min_steering_height", 0.5);
  feature_auto_arm_enabled_ = declare_parameter("feature_auto_arm_enabled", true);
  feature_auto_land_enabled_ = declare_parameter("feature_auto_land_enabled", true);
  auto_landing_active_ = false;
  auto_landing_descent_velocity_ = std::max(
    0.0, declare_parameter("auto_landing_descent_velocity", 0.3));
  feature_hover_hold_enabled_ = declare_parameter("feature_hover_hold_enabled", false);

  setpoint_rate_ = std::max(2.0, declare_parameter("setpoint_rate", 20.0));
  offboard_prestream_duration_ = declare_parameter("offboard_prestream_duration", 1.5);
  required_setpoint_count_ = static_cast<int>(
    std::max(10.0, std::ceil(setpoint_rate_ * offboard_prestream_duration_)));
  command_timeout_ = declare_parameter("command_timeout", 0.5);
  require_local_position_before_arm_ =
    declare_parameter("require_local_position_before_arm", false);
  local_position_ready_duration_ = declare_parameter("local_position_ready_duration", 1.0);
  hold_velocity_epsilon_ = declare_parameter("hold_velocity_epsilon", 1e-4);
  idle_descent_rate_ = std::max(0.0, declare_parameter("idle_descent_rate", 0.5));
  require_motion_before_engage_ = declare_parameter("require_motion_before_engage", true);
  request_retry_period_ = declare_parameter("request_retry_period", 1.0);
  mission_state_timeout_ = declare_parameter("mission_state_timeout", 1.0);
  return_home_on_control_tower_timeout_ =
    declare_parameter("return_home_on_control_tower_timeout", true);
  configure_px4_home_on_climb_ = declare_parameter("configure_px4_home_on_climb", true);
  publish_px4_aux_global_position_ = declare_parameter("publish_px4_aux_global_position", true);
  px4_aux_global_position_eph_ = std::max(
    0.01, declare_parameter("px4_aux_global_position_eph", 0.05));
  px4_aux_global_position_epv_ = std::max(
    0.01, declare_parameter("px4_aux_global_position_epv", 0.05));
  rcl_interfaces::msg::ParameterDescriptor optional_return_home_coordinate_descriptor;
  optional_return_home_coordinate_descriptor.dynamic_typing = true;
  const auto return_home_x_param = declare_parameter(
    "return_home_x", rclcpp::ParameterValue{}, optional_return_home_coordinate_descriptor);
  const auto return_home_y_param = declare_parameter(
    "return_home_y", rclcpp::ParameterValue{}, optional_return_home_coordinate_descriptor);
  return_home_x_configured_ =
    return_home_x_param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  return_home_y_configured_ =
    return_home_y_param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  if (return_home_x_configured_) {
    return_home_x_ = return_home_x_param.get<double>();
  }
  if (return_home_y_configured_) {
    return_home_y_ = return_home_y_param.get<double>();
  }
  return_home_height_ = std::max(0.0, declare_parameter("return_home_height", 10.0));
  const auto home_position_x_param = declare_parameter(
    "home_position_x", rclcpp::ParameterValue{}, optional_return_home_coordinate_descriptor);
  const auto home_position_y_param = declare_parameter(
    "home_position_y", rclcpp::ParameterValue{}, optional_return_home_coordinate_descriptor);
  home_position_x_configured_ =
    home_position_x_param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  home_position_y_configured_ =
    home_position_y_param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  home_position_x_ = home_position_x_configured_ ? home_position_x_param.get<double>() :
    (return_home_x_configured_ ? return_home_x_ : 0.0);
  home_position_y_ = home_position_y_configured_ ? home_position_y_param.get<double>() :
    (return_home_y_configured_ ? return_home_y_ : 0.0);
  home_position_initialized_ =
    (return_home_x_configured_ || home_position_x_configured_) &&
    (return_home_y_configured_ || home_position_y_configured_);
  control_tower_heartbeat_timeout_ =
    std::max(0.1, declare_parameter("control_tower_heartbeat_timeout", 2.0));
  return_home_command_retry_period_ =
    std::max(0.1, declare_parameter("return_home_command_retry_period", 2.0));
  // Disabled by default: the control tower already visualises the route it sent.
  // Set to true in the YAML to publish markers for RViz2 or other consumers.
  publish_waypoint_markers_ = declare_parameter("publish_waypoint_markers", false);
  mCopterAutopilotComponent->setAutoClimbEnabled(
    declare_parameter("feature_auto_climb_enabled", true));
  mCopterAutopilotComponent->setAutoClimbActive(false);
  mCopterAutopilotComponent->setAutoClimbHeight(declare_parameter("auto_climb_height", 2.0));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&WaywiserCopter::on_parameter_set, this, std::placeholders::_1));

  mCopterInterfaceComponent->setLength(declare_parameter("length", 0.52));
  mCopterInterfaceComponent->setWidth(declare_parameter("width", 0.52));
  mCopterInterfaceComponent->setVehicleStatePollRate(
    declare_parameter(
      "vehicle_state_poll_rate",
      30));
  mCopterInterfaceComponent->setVehicleInterfaceType(
    parse_vehicle_interface_type(
      declare_parameter("vehicle_interface_type", std::string("ext_simulated"))));
  declare_parameter("control_tower_selectable", true);
  enable_px4_bridge_ = declare_parameter("enable_px4_bridge", true);

  const int autopilot_timer_rate = declare_parameter("autopilot_timer_rate", 10);
  const bool enable_mavlink_interface = declare_parameter("enable_mavlink_interface", true);
  const std::string waywise_control_tower_address = declare_parameter("waywise_control_tower_address", std::string("127.0.0.1"));
  const int waywise_control_tower_port = declare_parameter("waywise_control_tower_port", 14540);
  const PosType mission_pos_type = parse_pos_type(declare_parameter("mission_position_type", std::string("odom")));
  const bool require_gnss_for_mission = declare_parameter("require_gnss_for_mission", false);
  const double mission_waypoint_proximity_xy = declare_parameter("mission_waypoint_proximity_xy", 0.5);
  const double mission_waypoint_proximity_z = declare_parameter("mission_waypoint_proximity_z", 1.0);
  const double mission_end_goal_alignment_threshold_xy = declare_parameter("mission_end_goal_alignment_threshold_xy", 0.25);
  const double mission_end_goal_alignment_threshold_z = declare_parameter("mission_end_goal_alignment_threshold_z", 0.25);
  const double mission_stop_speed_threshold = declare_parameter("mission_stop_speed_threshold", 0.2);
  const double mission_cruise_speed = declare_parameter("mission_cruise_speed", 1.0);
  const double mission_max_speed = declare_parameter("mission_max_speed", 2.0);
  const double mission_min_approach_speed = declare_parameter("mission_min_approach_speed", 0.1);
  const double mission_approach_slowdown_radius = declare_parameter("mission_approach_slowdown_radius", 1.5);
  const bool mission_face_travel_direction = declare_parameter("mission_face_travel_direction", true);
  const double mission_yaw_gain = declare_parameter("mission_yaw_gain", 1.5);
  const double mission_max_yaw_rate = declare_parameter("mission_max_yaw_rate", 1.0);
  const double mission_vertical_height_tolerance = declare_parameter("mission_vertical_height_tolerance", 0.10);
  const double mission_vertical_proportional_gain = declare_parameter("mission_vertical_proportional_gain", 0.8);
  const double mission_vertical_integral_gain = declare_parameter("mission_vertical_integral_gain", 0.04);
  const double mission_vertical_derivative_gain = declare_parameter("mission_vertical_derivative_gain", 0.5);
  const double mission_vertical_integral_limit = declare_parameter("mission_vertical_integral_limit", 2.0);
  const double position_accuracy_threshold_for_mission = declare_parameter("position_accuracy_threshold_for_mission", 0.5);
  const double yaw_accuracy_threshold_for_mission = declare_parameter("yaw_accuracy_threshold_for_mission", 5.0);

  auto configure_autopilot = [&](QSharedPointer<CopterAutopilotComponent> comp) {
    if (!comp) return;
    comp->setAutopilotTimerRate(autopilot_timer_rate);
    comp->setEnableMavlinkInterface(enable_mavlink_interface);
    comp->setWaywiseControlTowerAddress(waywise_control_tower_address);
    comp->setWaywiseControlTowerPort(waywise_control_tower_port);
    comp->setMissionPosTypeUsed(mission_pos_type);
    comp->setRequireGnssForMission(require_gnss_for_mission);
    comp->setWaypointProximityXY(mission_waypoint_proximity_xy);
    comp->setWaypointProximityZ(mission_waypoint_proximity_z);
    comp->setEndGoalAlignmentThresholdXY(mission_end_goal_alignment_threshold_xy);
    comp->setEndGoalAlignmentThresholdZ(mission_end_goal_alignment_threshold_z);
    comp->setStopSpeedThreshold(mission_stop_speed_threshold);
    comp->setCruiseSpeed(mission_cruise_speed);
    comp->setMaxMissionSpeed(mission_max_speed);
    comp->setDescentSpeed(auto_landing_descent_velocity_);
    comp->setMinApproachSpeed(mission_min_approach_speed);
    comp->setApproachSlowdownRadius(mission_approach_slowdown_radius);
    comp->setFaceTravelDirection(mission_face_travel_direction);
    comp->setYawGain(mission_yaw_gain);
    comp->setMaxYawRate(mission_max_yaw_rate);
    comp->setVerticalHeightTolerance(mission_vertical_height_tolerance);
    comp->setVerticalProportionalGain(mission_vertical_proportional_gain);
    comp->setVerticalIntegralGain(mission_vertical_integral_gain);
    comp->setVerticalDerivativeGain(mission_vertical_derivative_gain);
    comp->setVerticalIntegralLimit(mission_vertical_integral_limit);
    comp->setPositionAccuracyThresholdForMission(position_accuracy_threshold_for_mission);
    comp->setYawAccuracyThresholdForMission(yaw_accuracy_threshold_for_mission);
  };

  if (enable_autopilot_component_)
  {
    configure_autopilot(mCopterAutopilotComponent);
  }
  configure_autopilot(mRthAutopilotComponent);

  auto vector3_param = RosUtils::get_vector3_param(this, "enuref");
  if (vector3_param) {
    enuref_ = vector3_param->to_type<llh_t>();
    mCopterState->setEnuRef(enuref_);
  }
}

void WaywiserCopter::setup_publishers()
{
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  if (!battery_state_topic_.empty()) {
    battery_state_pub_ =
      create_publisher<waywiser_core::msg::BatteryState>(battery_state_topic_, 10);
  }
  if (!odom_topic_.empty()) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  }
  if (!vehicle_pose_topic_.empty()) {
    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vehicle_pose_topic_, 10);
  }
  if (!home_pose_topic_.empty()) {
    home_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      home_pose_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    publish_home_pose();
  }
  if (mCopterInterfaceComponent->getVehicleInterfaceType() == VehicleInterfaceType::EXT_SIMULATED &&
    enable_px4_bridge_)
  {
    px4_vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);
    auto offboard_qos = rclcpp::QoS(rclcpp::KeepLast(7)).reliable().durability_volatile();
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", offboard_qos);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", offboard_qos);
    if (publish_px4_aux_global_position_) {
      px4_aux_global_position_pub_ = create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/in/aux_global_position", offboard_qos);
    }
  }
  if (!quadcopter_state_topic_.empty()) {
    quadcopter_state_pub_ = create_publisher<waywiser_core::msg::QuadcopterState>(
      quadcopter_state_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    publish_quadcopter_state();
  }
  if (!emergency_stop_update_topic_.empty()) {
    emergency_stop_update_pub_ =
      create_publisher<waywiser_twist_safety::msg::EmergencyStopState>(
      emergency_stop_update_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
  }
  if (!mission_status_topic_.empty()) {
    mission_status_pub_ =
      create_publisher<waywiser_core::msg::MissionState>(
      mission_status_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    if (enable_autopilot_component_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::WaitingForVehicleInit);
      mission_status_pub_->publish(missionStateMsg);
    }
  }
  if (!control_tower_heartbeat_rx_state_topic_.empty()) {
    control_tower_heartbeat_rx_state_pub_ =
      create_publisher<waywiser_core::msg::HeartbeatRxState>(
      control_tower_heartbeat_rx_state_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
  }

  if (enable_autopilot_component_ && mission_status_pub_) {

    QObject::connect(
      mCopterAutopilotComponent.get(), &CopterAutopilotComponent::gnssFixAccuracyAssertionFailed,
      [&](GnssFixStatus gnssFixStatus) {
        if (emergency_stop_update_pub_ && !mEmergencyStopState->is_active()) {
          std::stringstream emergency_stop_reason;
          emergency_stop_reason << "GNSS accuracy dropped below thresholds: " <<
            std::fixed << std::setprecision(2) << gnssFixStatus.horizontalAccuracy <<
            " m and " << std::fixed << std::setprecision(2) <<
            gnssFixStatus.headingAccuracy << " deg.";

          auto emergency_stop_msg = waywiser_twist_safety::msg::EmergencyStopState();
          emergency_stop_msg.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;
          emergency_stop_msg.sender_id = this->get_name();
          emergency_stop_msg.stamp = this->get_clock()->now();
          emergency_stop_msg.reason = emergency_stop_reason.str();
          emergency_stop_update_pub_->publish(emergency_stop_msg);
        }
      });
  }

  if (enable_autopilot_component_) {
    route_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "waypoint_markers", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
    autopilot_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "autopilot_markers", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);

    QObject::connect(
      mCopterAutopilotComponent.get(), &CopterAutopilotComponent::updatedMissionState,
      [&](MissionState state) {
        switch (state) {
          case MissionState::FollowRouteInit:
            publish_route_markers();
            break;
          case MissionState::Idle:
          case MissionState::FollowRouteFinished:
            if (mCopterAutopilotComponent->getAutoClimbActive()) {
              current_cmd_vel_in_ = geometry_msgs::msg::Twist();
              current_cmd_vel_out_ = geometry_msgs::msg::Twist();
              last_input_command_time_ = 0.0;
              climb_active_ = false;
              climb_saved_v_up_ = 0.0F;
              if (has_vehicle_local_position_) {
                hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
                has_hold_position_ = true;
              }
              RCLCPP_INFO(get_logger(), "Auto climb finished. Switching to hover hold.");
              publish_quadcopter_state();
            }
            publish_autopilot_markers();
            break;
          default:
            break;
        }
      });
  }

  cmd_vel_out_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);
}

void WaywiserCopter::setup_subscribers()
{
  if (!input_odom_topic_.empty()) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 10, std::bind(&WaywiserCopter::odom_callback, this, _1));
  }

  if (mCopterInterfaceComponent->getVehicleInterfaceType() == VehicleInterfaceType::EXT_SIMULATED &&
    enable_px4_bridge_)
  {
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(7))
      .reliable()
      .durability_volatile();
    px4_actuator_armed_sub_ = create_subscription<px4_msgs::msg::ActuatorArmed>(
      "/fmu/out/actuator_armed", px4_qos,
      std::bind(&WaywiserCopter::px4_actuator_armed_callback, this, _1));
    px4_health_report_sub_ = create_subscription<px4_msgs::msg::HealthReport>(
      "/fmu/out/health_report", px4_qos,
      std::bind(&WaywiserCopter::px4_health_report_callback, this, _1));
    px4_vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_land_detected_callback, this, _1));
    px4_vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_odometry_callback, this, _1));
    px4_vehicle_global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_global_position_callback, this, _1));
    px4_vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_local_position_callback, this, _1));
    px4_vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_status_callback, this, _1));
    px4_home_position_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
      "/fmu/out/home_position", px4_qos,
      std::bind(&WaywiserCopter::px4_home_position_callback, this, _1));
    px4_vehicle_command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", px4_qos,
      std::bind(&WaywiserCopter::px4_vehicle_command_ack_callback, this, _1));

    if (!range_topic_.empty()) {
      range_sub_ = create_subscription<sensor_msgs::msg::Range>(
        range_topic_, 10, std::bind(&WaywiserCopter::range_callback, this, _1));
    }

  }

  if (!control_tower_heartbeat_topic_.empty()) {
    control_tower_heartbeat_sub_ = create_subscription<std_msgs::msg::Header>(
      control_tower_heartbeat_topic_, 10,
      std::bind(&WaywiserCopter::control_tower_heartbeat_callback, this, _1));
  }

  if (!fused_nav_sat_fix_extended_topic_.empty()) {
    fused_nav_sat_fix_extended_sub_ =
      create_subscription<waywiser_core::msg::NavSatFixExtended>(
      fused_nav_sat_fix_extended_topic_, 10,
      std::bind(&WaywiserCopter::fused_nav_sat_fix_extended_callback, this, _1));
  }

  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_in", 10, std::bind(&WaywiserCopter::twist_callback, this, _1));

  if (!emergency_stop_status_topic_.empty()) {
    emergency_stop_status_sub_ =
      create_subscription<waywiser_twist_safety::msg::EmergencyStopState>(
      emergency_stop_status_topic_, 10,
      std::bind(&WaywiserCopter::emergency_stop_status_callback, this, _1));
  }

  if (enable_autopilot_component_) {
    if (!enable_px4_bridge_ && !observed_quadcopter_state_topic_.empty()) {
      observed_quadcopter_state_sub_ =
        create_subscription<waywiser_core::msg::QuadcopterState>(
        observed_quadcopter_state_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
        std::bind(&WaywiserCopter::observed_quadcopter_state_callback, this, _1));
    }

    if (!autopilot_state_control_topic_.empty()) {
      autopilot_state_control_sub_ = create_subscription<std_msgs::msg::Bool>(
        autopilot_state_control_topic_,
        QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
        std::bind(&WaywiserCopter::autopilot_state_control_callback, this, _1));
    }

    path_with_twists_sub_ = create_subscription<waywiser_core::msg::PathWithTwists>(
      "waywiser_path", QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
      std::bind(&WaywiserCopter::path_with_twists_callback, this, _1));
  } else if (!mission_status_topic_.empty()) {
    mission_status_sub_ = create_subscription<waywiser_core::msg::MissionState>(
      mission_status_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS,
      std::bind(&WaywiserCopter::mission_status_callback, this, _1));
  }

  return_home_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/return_home",
    std::bind(&WaywiserCopter::handle_return_home_request, this, std::placeholders::_1, std::placeholders::_2));

  auto_land_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/auto_landing",
    std::bind(&WaywiserCopter::handle_auto_land_request, this, std::placeholders::_1, std::placeholders::_2));

  arm_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/arm",
    std::bind(&WaywiserCopter::handle_arm_request, this, std::placeholders::_1, std::placeholders::_2));

  auto_climb_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/auto_climb",
    std::bind(&WaywiserCopter::handle_auto_climb_request, this, std::placeholders::_1, std::placeholders::_2));
}

void WaywiserCopter::setup_timers()
{
  const auto timer_rate = std::max(1, mCopterInterfaceComponent->getVehicleStatePollRate());
  node_management_timer_ = rclcpp::create_timer(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    this->get_clock(),
    std::chrono::milliseconds(1000 / timer_rate),
    std::bind(&WaywiserCopter::node_management_timer_callback, this));

  if (offboard_control_mode_pub_) {
    setpoint_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      std::chrono::milliseconds(static_cast<int>(1000.0 / setpoint_rate_)),
      std::bind(&WaywiserCopter::setpoint_timer_callback, this));
  }

  const int autopilot_timer_rate = get_parameter("autopilot_timer_rate").as_int();
  autopilot_state_machine_timer_ = rclcpp::create_timer(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    this->get_clock(),
    std::chrono::milliseconds(1000 / autopilot_timer_rate),
    std::bind(&WaywiserCopter::process_autopilots, this)
  );
}

void WaywiserCopter::process_autopilots()
{
  if (enable_autopilot_component_ && mCopterAutopilotComponent) {
    if (waypoint_follower_route_activation_allowed()) {
      const auto current_mission_state = mCopterAutopilotComponent->getCurrentMissionState();
      if (!mCopterAutopilotComponent->getWaypointList().isEmpty() &&
        !mCopterAutopilotComponent->isActive() &&
        !mission_state_is_route_active(current_mission_state))
      {
        mCopterAutopilotComponent->switchAutopilot(true);
      }
      mCopterAutopilotComponent->processMissionStateMachine();
    }
  }
  if (mRthAutopilotComponent) {
    mRthAutopilotComponent->processMissionStateMachine();
  }
}

bool WaywiserCopter::waypoint_follower_route_activation_allowed() const
{
  if (enable_px4_bridge_ || observed_quadcopter_state_topic_.empty()) {
    return true;
  }

  return observed_quadcopter_state_received_ && observed_drone_route_activation_allowed_;
}

void WaywiserCopter::node_management_timer_callback()
{
  const auto now_time = this->get_clock()->now();
  if (has_last_node_management_time_ && now_time < last_node_management_time_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Simulation time reset detected. Clearing runtime home pose until odometry reseeds it.");
    received_first_odom_msg_ = false;
    if (!return_home_x_configured_ || !return_home_y_configured_) {
      home_position_initialized_ =
        (return_home_x_configured_ || home_position_x_configured_) &&
        (return_home_y_configured_ || home_position_y_configured_);
    }
    has_climb_position_ = false;
    has_landed_after_climb_ = false;
    px4_home_position_set_ = false;
    if (home_pose_pub_) {
      home_pose_pub_.reset();
    }
    if (!home_pose_topic_.empty()) {
      home_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        home_pose_topic_, QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS);
      publish_home_pose();
    }
    // Clear waypoint follower route and reset autopilot state so the node
    // returns to idle after a Gazebo world/time reset instead of continuing
    // to issue stale motion commands from the previous run.
    if (enable_autopilot_component_ && mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
      RCLCPP_INFO(get_logger(), "Autopilot route cleared due to simulation time reset.");
    }
    has_hold_position_ = false;
    last_input_command_time_ = -1.0;
    was_command_active_ = false;
    climb_active_ = false;
    climb_saved_v_up_ = 0.0F;
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    last_node_management_time_ = now_time;
    return;
  }
  has_last_node_management_time_ = true;
  last_node_management_time_ = now_time;

  update_control_tower_heartbeat_failsafe();
  publish_command();
  publish_quadcopter_state();
  publish_control_tower_heartbeat_rx_state();

  const bool has_fused_pose = !mCopterState->getPosition(PosType::fused).getTime().isNull();
  if (has_fused_pose || received_first_odom_msg_) {
    publish_world_pose();
  }

  if (mCopterInterfaceComponent->getVehicleInterfaceType() == VehicleInterfaceType::EXT_SIMULATED &&
    enable_px4_bridge_ &&
    !received_first_odom_msg_)
  {
    if (enable_autopilot_component_ && mission_status_pub_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::WaitingForVehicleInit);
      mission_status_pub_->publish(missionStateMsg);
    }
    return;
  }

  if (publish_odom_to_baselink_tf_ || publish_world_to_odom_tf_) {
    publish_tfs();
  }

  if (enable_autopilot_component_ && mission_status_pub_) {
    const auto current_mission_state = mCopterAutopilotComponent->getCurrentMissionState();
    const bool auto_climb_active = mCopterAutopilotComponent->getAutoClimbActive();
    const bool route_mission_active =
      mission_state_is_route_active(current_mission_state) &&
      !auto_climb_active;
    const bool route_pending = !mCopterAutopilotComponent->getWaypointList().isEmpty();
    const bool vehicle_starting_up = enable_px4_bridge_ &&
      derive_quadcopter_state() == HighLevelState::STARTING_UP;
    if (!enable_px4_bridge_ && is_return_home_active()) {
      // Do not publish mission status from the waypoint follower node during RTH.
      // Let the primary drone node publish the RTH mission state.
    } else if (!enable_px4_bridge_ &&
      route_pending &&
      !waypoint_follower_route_activation_allowed())
    {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::WaitingForVehicleInit);
      mission_status_pub_->publish(missionStateMsg);
    } else if (!enable_px4_bridge_ &&
      !observed_quadcopter_state_topic_.empty() &&
      (!observed_quadcopter_state_received_ || observed_drone_starting_up_))
    {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::WaitingForVehicleInit);
      mission_status_pub_->publish(missionStateMsg);
    } else if (vehicle_starting_up) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::WaitingForVehicleInit);
      mission_status_pub_->publish(missionStateMsg);
    } else if (!enable_px4_bridge_ && auto_climb_active) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    } else if (!enable_px4_bridge_ &&
      observed_quadcopter_state_received_ &&
      observed_drone_has_been_on_mission_ &&
      !observed_drone_on_mission_ &&
      !observed_drone_returning_home_ &&
      !route_mission_active)
    {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    } else {
      if (!enable_px4_bridge_ && (observed_drone_on_mission_ || observed_drone_returning_home_) && !route_mission_active) {
        return; // Let the primary drone node publish its mission state (e.g. RTH)
      }
      waywiser_core::msg::MissionState missionStateMsg;
      if (is_return_home_active()) {
        missionStateMsg.state =
          static_cast<uint8_t>(derive_return_home_mission_state());
      } else if (auto_landing_active_) {
        missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      } else if (return_home_completed_) {
        // RTH is done; publish Idle so the UI clears the RTH Landing state.
        missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      } else {
        missionStateMsg.state =
          static_cast<uint8_t>(current_mission_state);
      }
      mission_status_pub_->publish(missionStateMsg);
    }
  } else if (
    !enable_autopilot_component_ &&
    mission_status_pub_)
  {
    waywiser_core::msg::MissionState missionStateMsg;
    if (is_return_home_active()) {
      missionStateMsg.state = static_cast<uint8_t>(derive_return_home_mission_state());
    } else if (auto_landing_active_) {
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
    } else if (return_home_completed_) {
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
    } else {
      return; // nothing RTH-related to publish
    }
    mission_status_pub_->publish(missionStateMsg);
  }

  if (enable_autopilot_component_ && autopilot_marker_pub_) {
    publish_autopilot_markers();
  }
}

void WaywiserCopter::emergency_stop_status_callback(
  const waywiser_twist_safety::msg::EmergencyStopState::SharedPtr msg)
{
  if (msg->state == waywiser_twist_safety::msg::EmergencyStopState::ACTIVE) {
    mCopterInterfaceComponent->activate_emergency_stop(msg->sender_id, msg->reason);
  } else if (msg->state == waywiser_twist_safety::msg::EmergencyStopState::CLEAR) {
    mCopterInterfaceComponent->clear_emergency_stop(msg->sender_id);
  }
}

void WaywiserCopter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  const bool first_odom_msg = !received_first_odom_msg_;
  if (first_odom_msg) {
    received_first_odom_msg_ = true;
    RCLCPP_INFO(this->get_logger(), "Received first odom message.");
  }

  CoreUtils::update_pospoint_from_pose(
    mCopterState, {0.0, 0.0, 0.0}, odom_msg->pose.pose, PosType::odom);
  if (!home_position_initialized_ && (!armed_ || first_odom_msg) && !has_climb_position_ &&
    (!return_home_x_configured_ || !return_home_y_configured_))
  {
    const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
    update_home_position_parameters(
      return_home_x_configured_ ? return_home_x_ :
      (home_position_x_configured_ ? home_position_x_ : odom_position.getX()),
      return_home_y_configured_ ? return_home_y_ :
      (home_position_y_configured_ ? home_position_y_ : odom_position.getY()));
  }
  if (mCopterAutopilotComponent &&
    mCopterAutopilotComponent->getMissionPosTypeUsed() == PosType::odom)
  {
    mCopterAutopilotComponent->setVehicleInitialized(true);
  }
  if (mRthAutopilotComponent &&
    mRthAutopilotComponent->getMissionPosTypeUsed() == PosType::odom)
  {
    mRthAutopilotComponent->setVehicleInitialized(true);
  }

  mCopterState->setVelocity(
    xyz_t{
    odom_msg->twist.twist.linear.x,
    odom_msg->twist.twist.linear.y,
    odom_msg->twist.twist.linear.z});
  mCopterState->setSpeed(
    std::hypot(
      odom_msg->twist.twist.linear.x,
      odom_msg->twist.twist.linear.y));
  mCopterState->setSteering(odom_msg->twist.twist.angular.z);
  publish_px4_aux_global_position();
  publish_odom();
}

void WaywiserCopter::publish_px4_aux_global_position()
{
  if (!px4_aux_global_position_pub_ || !mCopterState->isEnuReferenceSet()) {
    return;
  }

  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const llh_t llh = coordinateTransforms::enuToLlh(
    mCopterState->getEnuRef(), odom_position.getXYZ());
  const uint64_t timestamp_us = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);

  px4_msgs::msg::VehicleGlobalPosition msg{};
  msg.timestamp = timestamp_us;
  msg.timestamp_sample = timestamp_us;
  msg.lat = llh.latitude;
  msg.lon = llh.longitude;
  msg.alt = static_cast<float>(llh.height);
  msg.alt_ellipsoid = static_cast<float>(llh.height);
  msg.lat_lon_valid = true;
  msg.alt_valid = true;
  msg.eph = static_cast<float>(px4_aux_global_position_eph_);
  msg.epv = static_cast<float>(px4_aux_global_position_epv_);
  msg.terrain_alt = static_cast<float>(llh.height - odom_position.getHeight());
  msg.terrain_alt_valid = true;
  msg.dead_reckoning = false;

  px4_aux_global_position_pub_->publish(msg);
}

void WaywiserCopter::px4_actuator_armed_callback(
  const px4_msgs::msg::ActuatorArmed::SharedPtr msg)
{
  ready_to_arm_ = msg->ready_to_arm;
  publish_quadcopter_state();
}

void WaywiserCopter::px4_health_report_callback(
  const px4_msgs::msg::HealthReport::SharedPtr msg)
{
  px4_health_warning_flags_ = msg->health_warning_flags;
  px4_health_error_flags_ = msg->health_error_flags;
  px4_arming_check_warning_flags_ = msg->arming_check_warning_flags;
  px4_arming_check_error_flags_ = msg->arming_check_error_flags;

  const uint64_t can_arm_flags = static_cast<uint64_t>(msg->can_arm_mode_flags);
  ready_for_takeoff_ = bool(
    can_arm_flags & (uint64_t{1} << px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF));

  ready_for_offboard_ = bool(
    can_arm_flags & (uint64_t{1} << px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD));

  if (!ready_for_takeoff_ && ready_for_offboard_) {
    RCLCPP_DEBUG(this->get_logger(), "Vehicle not ready for AUTO_TAKEOFF but ready for OFFBOARD.");
  }

  const bool has_preflight_failure =
    !ready_for_takeoff_ || (px4_health_error_flags_ != 0) || (px4_arming_check_error_flags_ != 0);

  if (has_preflight_failure) {
    preflight_all_pass_logged_ = false;
    const int64_t now_ns = now().nanoseconds();
    if (
      last_preflight_failure_log_time_ns_ == 0 ||
      (now_ns - last_preflight_failure_log_time_ns_) >= 1000000000LL)
    {
      last_preflight_failure_log_time_ns_ = now_ns;
      const std::string summary = format_px4_preflight_summary(
        ready_for_takeoff_,
        ready_for_offboard_,
        ready_to_arm_,
        px4_health_error_flags_,
        px4_health_warning_flags_,
        px4_arming_check_error_flags_,
        px4_arming_check_warning_flags_,
        can_arm_flags);
      RCLCPP_WARN(
        this->get_logger(),
        "PX4 preflight checks failing.\n%s",
        summary.c_str());
    }
  } else if (!preflight_all_pass_logged_) {
    preflight_all_pass_logged_ = true;
    const std::string summary = format_px4_preflight_summary(
      ready_for_takeoff_,
      ready_for_offboard_,
      ready_to_arm_,
      px4_health_error_flags_,
      px4_health_warning_flags_,
      px4_arming_check_error_flags_,
      px4_arming_check_warning_flags_,
      can_arm_flags);
    RCLCPP_INFO(
      this->get_logger(),
      "PX4 preflight checks passed.\n%s",
      summary.c_str());
  }

  publish_quadcopter_state();
}

void WaywiserCopter::px4_vehicle_land_detected_callback(
  const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  px4_ground_contact_ = msg->ground_contact;
  px4_maybe_landed_ = msg->maybe_landed;
  px4_landed_ = msg->landed;
  refresh_in_flight_status();
  if (return_home_landed_at_home()) {
    const bool was_heartbeat_failsafe =
      control_tower_timeout_return_home_active_ && !manual_return_home_active_;
    clear_return_home_failsafe();
    manual_return_home_active_ = false;
    return_home_completed_ = true;
    if (was_heartbeat_failsafe) {
      publish_control_tower_timeout_emergency_stop(control_tower_heartbeat_monitor_age());
    }
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    RCLCPP_INFO(get_logger(), "Return-home completed: vehicle landed at home.");
  }
  if (return_home_completed_ && armed_) {
    const auto now_time = get_clock()->now();
    const bool disarm_request_due = !has_last_return_home_disarm_request_time_ ||
      (now_time - last_return_home_disarm_request_time_).seconds() >= return_home_command_retry_period_;
    if (disarm_request_due) {
      return_home_disarm_requested_ = true;
      has_last_return_home_disarm_request_time_ = true;
      last_return_home_disarm_request_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Return-home landing complete. Requesting forced disarm.");
      send_arm_command(false, true);
      last_arm_request_value_ = false;
    }
  }
  publish_quadcopter_state();
}

void WaywiserCopter::px4_vehicle_odometry_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  if (std::isfinite(msg->position[2])) {
    latest_px4_altitude_ = -msg->position[2];
    has_px4_altitude_ = true;
  }

  if (std::isfinite(msg->velocity[2])) {
    latest_px4_vertical_velocity_ = -msg->velocity[2];
    has_px4_vertical_velocity_ = true;
  }

  refresh_in_flight_status();
}

void WaywiserCopter::px4_vehicle_global_position_callback(
  const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  global_position_lat_lon_valid_ = msg->lat_lon_valid;
  global_position_alt_valid_ = msg->alt_valid;
}

void WaywiserCopter::px4_vehicle_local_position_callback(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  has_vehicle_local_position_ = true;
  local_position_xy_valid_ = msg->xy_valid;
  local_position_z_valid_ = msg->z_valid;
  local_position_xy_global_ = msg->xy_global;
  local_position_z_global_ = msg->z_global;
  local_position_v_xy_valid_ = msg->v_xy_valid;
  local_position_v_z_valid_ = msg->v_z_valid;
  if (std::isfinite(msg->x)) {local_position_x_ = msg->x;}
  if (std::isfinite(msg->y)) {local_position_y_ = msg->y;}
  if (std::isfinite(msg->z)) {
    local_position_z_ = msg->z;
    latest_px4_altitude_ = -msg->z;
    has_px4_altitude_ = true;
  }
  if (std::isfinite(msg->heading)) {vehicle_heading_ = msg->heading;}

  if (std::isfinite(msg->vz)) {
    latest_px4_vertical_velocity_ = -msg->vz;
    has_px4_vertical_velocity_ = true;
  }

  if (!received_range_data_) {
    px4_dist_bottom_valid_ = msg->dist_bottom_valid;
    if (msg->dist_bottom_valid && std::isfinite(msg->dist_bottom)) {
      px4_dist_bottom_ = msg->dist_bottom;
    }
  }

  // Track local position stability for arm gating
  if (local_position_ready()) {
    if (!has_local_position_ready_since_) {
      has_local_position_ready_since_ = true;
      local_position_ready_since_ = get_clock()->now();
    }
  } else {
    has_local_position_ready_since_ = false;
  }
  if (local_position_stable() && !has_hold_position_) {
    hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
    has_hold_position_ = true;
  }

  refresh_in_flight_status();
  publish_quadcopter_state();
}

void WaywiserCopter::px4_vehicle_status_callback(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  if (!armed_) {
    has_climb_position_ = false;
    if (auto_landing_active_) {
      auto_landing_active_ = false;
      auto_landing_disarm_requested_ = false;
      inhibit_auto_arm_after_auto_land_ = true;
      current_cmd_vel_in_ = geometry_msgs::msg::Twist();
      current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    }
  }
  px4_nav_state_ = msg->nav_state;
  ready_to_arm_ = msg->pre_flight_checks_pass;
  refresh_in_flight_status();
  publish_quadcopter_state();
}

void WaywiserCopter::px4_home_position_callback(const px4_msgs::msg::HomePosition::SharedPtr msg)
{
  px4_home_position_hpos_valid_ = msg->valid_hpos;
  px4_home_position_alt_valid_ = msg->valid_alt;
  if (msg->valid_hpos && msg->valid_alt) {
    px4_home_position_set_ = true;
  }
}

void WaywiserCopter::observed_quadcopter_state_callback(
  const waywiser_core::msg::QuadcopterState::SharedPtr msg)
{
  const bool was_on_mission =
    !observed_quadcopter_state_received_ || observed_drone_on_mission_;
  observed_quadcopter_state_received_ = true;
  observed_drone_starting_up_ =
    msg->state_code == waywiser_core::msg::QuadcopterState::STARTING_UP;
  switch (msg->state_code) {
    case waywiser_core::msg::QuadcopterState::READY_TO_ARM:
    case waywiser_core::msg::QuadcopterState::ARMED:
    case waywiser_core::msg::QuadcopterState::IN_FLIGHT:
    case waywiser_core::msg::QuadcopterState::LANDING:
    case waywiser_core::msg::QuadcopterState::CLIMBING:
    case waywiser_core::msg::QuadcopterState::HOVERING:
    case waywiser_core::msg::QuadcopterState::ON_MISSION:
    case waywiser_core::msg::QuadcopterState::RETURNING_HOME:
      observed_drone_route_activation_allowed_ = true;
      break;
    default:
      observed_drone_route_activation_allowed_ = false;
      break;
  }
  observed_drone_on_mission_ =
    msg->state_code == waywiser_core::msg::QuadcopterState::ON_MISSION;
  observed_drone_returning_home_ =
    msg->state_code == waywiser_core::msg::QuadcopterState::RETURNING_HOME;
  const bool observed_drone_landing =
    msg->state_code == waywiser_core::msg::QuadcopterState::LANDING;
  const bool observed_drone_climbing =
    msg->state_code == waywiser_core::msg::QuadcopterState::CLIMBING;
  const auto current_mission_state = mCopterAutopilotComponent ?
    mCopterAutopilotComponent->getCurrentMissionState() :
    MissionState::Idle;
  const bool route_mission_active =
    mCopterAutopilotComponent &&
    mission_state_is_route_active(current_mission_state) &&
    !mCopterAutopilotComponent->getAutoClimbActive();

  // Only block cmd_vel and bail out if THIS node still has auto_landing active.
  // If auto_landing was already cancelled (e.g. by a climb request sent to the main node
  // just before this callback fires), do NOT zero cmd_vel — the drone needs to hold/climb.
  if (observed_drone_landing && auto_landing_active_ &&
      !(mCopterAutopilotComponent && mCopterAutopilotComponent->getAutoClimbActive())) {
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    if (mission_status_pub_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    }
    mission_idle_published_after_observed_drone_left_mission_ = true;
    return;
  }

  if (waypoint_follower_route_activation_allowed() &&
    mCopterAutopilotComponent &&
    !mCopterAutopilotComponent->getWaypointList().isEmpty() &&
    !mCopterAutopilotComponent->isActive() &&
    !route_mission_active)
  {
    mCopterAutopilotComponent->switchAutopilot(true);
    mCopterAutopilotComponent->processMissionStateMachine();
  }

  if (observed_drone_on_mission_) {
    observed_drone_has_been_on_mission_ = true;
    mission_idle_published_after_observed_drone_left_mission_ = false;
    return;
  }

  if (route_mission_active && observed_drone_climbing) {
    return;
  }

  if (observed_drone_has_been_on_mission_ && was_on_mission) {
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    if (mission_status_pub_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    }
    mission_idle_published_after_observed_drone_left_mission_ = true;
  }
}

bool WaywiserCopter::request_arm_state(bool arm)
{
  if (arm) {
    if (auto_landing_active_ || auto_landing_disarm_requested_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 2000,
        "Ignoring arm request: auto landing is active.");
      return false;
    }
    if (armed_) {
      return true;
    }
    if (!px4_ready_for_arm_command(ready_for_takeoff_, ready_for_offboard_, ready_to_arm_)) {
      return false;
    }
    if (control_tower_heartbeat_timed_out()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring arm request: Control Tower heartbeat is timed out.");
      return false;
    }
  } else {
    if (!armed_) {
      return true;
    }
    if (in_flight_) {
      RCLCPP_WARN(this->get_logger(), "Ignoring disarm request: vehicle is still in flight.");
      return false;
    }
  }

  double now_monotonic = steady_time_seconds();
  if (
    arm && last_arm_request_value_ &&
    (now_monotonic - last_arm_request_time_) < arm_transition_grace_period_)
  {
    return true;
  }
  if (
    last_arm_request_value_ == arm &&
    (now_monotonic - last_arm_request_time_) < arm_request_debounce_period_)
  {
    return true;
  }

  last_arm_request_value_ = arm;
  last_arm_request_time_ = now_monotonic;

  if (arm) {
    clear_return_home_failsafe();
    manual_return_home_active_ = false;
    return_home_completed_ = false;
    return_home_disarm_requested_ = false;
    has_last_return_home_disarm_request_time_ = false;
    has_landed_after_climb_ = false;
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    send_offboard_mode_command();
    send_arm_command(true);
    RCLCPP_INFO(this->get_logger(), "Sent PX4 offboard mode and arm commands.");
  } else {
    send_arm_command(false);
    RCLCPP_INFO(this->get_logger(), "Sent PX4 disarm command.");
  }
  return true;
}

void WaywiserCopter::send_arm_command(bool arm, bool force)
{
  if (!px4_vehicle_command_pub_) {
    RCLCPP_WARN(this->get_logger(), "Cannot send PX4 arm/disarm command: publisher unavailable.");
    return;
  }

  px4_msgs::msg::VehicleCommand command{};
  command.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  command.param1 = arm ? 1.0F : 0.0F;
  command.param2 = force ? 21196.0F : 0.0F;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 191;
  command.from_external = true;
  px4_vehicle_command_pub_->publish(command);
}

void WaywiserCopter::send_offboard_mode_command()
{
  if (!px4_vehicle_command_pub_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot send PX4 offboard mode command: publisher unavailable.");
    return;
  }

  px4_msgs::msg::VehicleCommand command{};
  command.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  command.param1 = 1.0F;
  command.param2 = 6.0F;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 1;
  command.from_external = true;
  px4_vehicle_command_pub_->publish(command);
}

bool WaywiserCopter::send_waywiser_return_home_command()
{
  if (!enable_px4_bridge_) {
    return true; // Waypoint follower node should not generate RTH routes.
  }

  if (return_home_position_reached()) {
    if (send_px4_land_at_current_position_command()) {
      return_home_landing_phase_active_ = true;
      return true;
    }
    return false;
  }

  if (!mRthAutopilotComponent) {
    return false;
  }

  QList<PosPoint> returnHomeRoute;
  PosPoint returnHomeApproach = mCopterState->getPosition(PosType::odom);
  const double target_home_x = home_position_x_;
  const double target_home_y = home_position_y_;
  const double cruise_speed = mRthAutopilotComponent->getCruiseSpeed();
  PosPoint returnHomeCruiseStart = returnHomeApproach;
  returnHomeCruiseStart.setHeight(return_home_height_);
  returnHomeCruiseStart.setSpeed(cruise_speed);
  returnHomeRoute.append(returnHomeCruiseStart);
  returnHomeApproach.setHeight(return_home_height_);
  returnHomeApproach.setX(target_home_x);
  returnHomeApproach.setY(target_home_y);
  returnHomeApproach.setSpeed(cruise_speed);
  returnHomeRoute.append(returnHomeApproach);
  PosPoint returnHomeLanding = returnHomeApproach;
  returnHomeLanding.setHeight(0.0);
  returnHomeLanding.setSpeed(auto_landing_descent_velocity_);
  returnHomeRoute.append(returnHomeLanding);
  mRthAutopilotComponent->startWaypointFollowerRouteFromBeginning(returnHomeRoute);
  return_home_landing_phase_active_ = false;
  RCLCPP_WARN(
    this->get_logger(),
    "Requesting WayWiseR return-home: stage to %.2f m, cruise to (%.2f, %.2f), then land at %.2f m/s.",
    return_home_height_,
    target_home_x,
    target_home_y,
    auto_landing_descent_velocity_);
  return true;
}

bool WaywiserCopter::return_home_landed_at_home() const
{
  if (!is_return_home_active()) {
    return false;
  }

  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const double landed_height_threshold = std::max(
    0.0, static_cast<double>(in_flight_range_threshold_ + kLandingHeightMargin));
  const bool odom_near_ground = odom_position.getHeight() <= landed_height_threshold;
  const bool vertical_motion_inactive = !has_px4_vertical_velocity_ ||
    std::fabs(latest_px4_vertical_velocity_) <= kVerticalMotionInAirThreshold;
  const bool landed = !in_flight_ || px4_landed_ || (px4_ground_contact_ && px4_maybe_landed_) ||
    (odom_near_ground && vertical_motion_inactive);
  if (!landed) {
    return false;
  }

  const double home_distance = std::hypot(
    odom_position.getX() - home_position_x_, odom_position.getY() - home_position_y_);
  const double home_tolerance = mRthAutopilotComponent
    ? std::max(1.0, mRthAutopilotComponent->getWaypointProximityXY())
    : 1.0;
  return home_distance <= home_tolerance;
}

bool WaywiserCopter::return_home_position_reached() const
{
  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const double home_distance = std::hypot(
    odom_position.getX() - home_position_x_, odom_position.getY() - home_position_y_);
  double home_tolerance = kDefaultHomePositionTolerance;
  if (mRthAutopilotComponent) {
    home_tolerance = std::max(home_tolerance, mRthAutopilotComponent->getWaypointProximityXY());
  }
  return home_distance <= home_tolerance;
}

bool WaywiserCopter::return_home_cruise_height_reached() const
{
  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const double height_tolerance = mRthAutopilotComponent ?
    mRthAutopilotComponent->getVerticalHeightTolerance() : 0.5;
  return odom_position.getHeight() + height_tolerance >= return_home_height_;
}

bool WaywiserCopter::return_home_ready_to_land() const
{
  if (!return_home_position_reached()) {
    return false;
  }
  if (!return_home_cruise_height_reached()) {
    return false;
  }

  const auto velocity = mCopterState->getVelocity();
  const double horizontal_speed = std::hypot(velocity.x, velocity.y);
  const double speed_tolerance = mRthAutopilotComponent ?
    std::max(
      kReturnHomeLandingHorizontalSpeedThreshold,
      mRthAutopilotComponent->getMinApproachSpeed()) :
    kReturnHomeLandingHorizontalSpeedThreshold;
  return horizontal_speed <= speed_tolerance;
}

MissionState WaywiserCopter::derive_return_home_mission_state() const
{
  if (return_home_landing_phase_active_ || return_home_land_request_sent_) {
    return MissionState::ReturnHomeLanding;
  }

  if (!return_home_request_sent_) {
    return MissionState::ReturnHomeInit;
  }

  const bool px4_climbing =
    has_px4_vertical_velocity_ && std::isfinite(latest_px4_vertical_velocity_) &&
    latest_px4_vertical_velocity_ > kVerticalMotionInAirThreshold;
  if (px4_climbing || !return_home_cruise_height_reached()) {
    return MissionState::ReturnHomeClimb;
  }

  if (return_home_ready_to_land()) {
    return MissionState::ReturnHomeLanding;
  }

  return MissionState::ReturnHomeCruising;
}

void WaywiserCopter::clear_return_home_failsafe()
{
  control_tower_timeout_return_home_active_ = false;
  waiting_for_heartbeat_mission_active_ = false;
  ignore_non_rth_mission_status_until_idle_ = false;
  has_last_return_home_request_time_ = false;
  return_home_request_sent_ = false;
  return_home_land_request_sent_ = false;
  return_home_landing_phase_active_ = false;
}

void WaywiserCopter::cancel_return_home_failsafe_on_heartbeat_restore()
{
  // Only clears heartbeat-timeout-specific flags. RTH completion state
  // (return_home_completed_, manual_return_home_active_) is owned by the RTH
  // state machine and must NOT be touched here.
  clear_return_home_failsafe();
  control_tower_heartbeat_timeout_emergency_stop_active_ = false;
  return_home_disarm_requested_ = false;
  has_last_return_home_disarm_request_time_ = false;
  if (mCopterAutopilotComponent) {
    mCopterAutopilotComponent->clearWaypointFollowerRoute();
  }
  if (armed_ && in_flight_) {
    if (has_vehicle_local_position_) {
      hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
      has_hold_position_ = true;
      publish_offboard_control_mode(feature_hover_hold_enabled_);
      publish_trajectory_setpoint();
    }
    send_offboard_mode_command();
  }
}

bool WaywiserCopter::send_return_home_command()
{
  return send_waywiser_return_home_command();
}

bool WaywiserCopter::send_px4_land_at_current_position_command()
{
  if (!px4_vehicle_command_pub_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot send PX4 land command: publisher unavailable.");
    return false;
  }

  px4_msgs::msg::VehicleCommand command{};
  command.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 191;
  command.from_external = true;
  px4_vehicle_command_pub_->publish(command);
  return_home_land_request_sent_ = true;
  if (mCopterAutopilotComponent) {
    mCopterAutopilotComponent->clearWaypointFollowerRoute();
  }
  if (mRthAutopilotComponent) {
    mRthAutopilotComponent->clearWaypointFollowerRoute();
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Return-home XY reached; requesting PX4 NAV_LAND.");
  return true;
}

bool WaywiserCopter::configure_px4_global_origin()
{
  if (!px4_vehicle_command_pub_) {
    RCLCPP_WARN(this->get_logger(), "Cannot configure PX4 global origin: publisher unavailable.");
    return false;
  }

  if (!mCopterState->isEnuReferenceSet()) {
    RCLCPP_WARN(this->get_logger(), "Cannot configure PX4 global origin: ENU reference is not set.");
    return false;
  }

  const llh_t enu_ref = mCopterState->getEnuRef();
  px4_msgs::msg::VehicleCommand command{};
  px4_home_position_set_ = false;
  command.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 191;
  command.from_external = true;
  command.param5 = static_cast<double>(enu_ref.latitude);
  command.param6 = static_cast<double>(enu_ref.longitude);
  command.param7 = static_cast<float>(enu_ref.height);
  px4_vehicle_command_pub_->publish(command);
  RCLCPP_INFO(
    this->get_logger(),
    "Configured PX4 global origin to ENU reference (%.7f, %.7f, %.2f).",
    enu_ref.latitude,
    enu_ref.longitude,
    enu_ref.height);
  return true;
}

void WaywiserCopter::configure_px4_home_position()
{
  if (!configure_px4_home_on_climb_) {
    return;
  }

  if (!px4_vehicle_command_pub_) {
    RCLCPP_WARN(this->get_logger(), "Cannot configure PX4 home: publisher unavailable.");
    return;
  }

  if (!mCopterState->isEnuReferenceSet()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot configure PX4 RTL home: ENU reference is not set.");
    return;
  }

  if (!(global_position_lat_lon_valid_ && global_position_alt_valid_ &&
    local_position_xy_global_ && local_position_z_global_))
  {
    configure_px4_global_origin();
    return;
  }

  const double target_home_x = home_position_x_;
  const double target_home_y = home_position_y_;
  const double target_home_z = 0.0;
  const llh_t target_home_llh = coordinateTransforms::enuToLlh(
    mCopterState->getEnuRef(), {target_home_x, target_home_y, target_home_z});

  px4_msgs::msg::VehicleCommand command{};
  command.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 191;
  command.from_external = true;
  command.param1 = 0.0F;
  command.param5 = target_home_llh.latitude;
  command.param6 = target_home_llh.longitude;
  command.param7 = static_cast<float>(target_home_llh.height);
  px4_home_position_set_ = false;
  px4_vehicle_command_pub_->publish(command);
  RCLCPP_INFO(
    this->get_logger(),
    "Configured PX4 RTL home to WayWiseR home (%.2f, %.2f).",
    target_home_x,
    target_home_y);
}

void WaywiserCopter::update_home_position_parameters(double x, double y)
{
  const bool was_initialized = home_position_initialized_;
  if (std::abs(home_position_x_ - x) <= 1e-6 && std::abs(home_position_y_ - y) <= 1e-6) {
    home_position_initialized_ = true;
    if (!was_initialized) {
      publish_home_pose();
    }
    return;
  }

  home_position_x_ = x;
  home_position_y_ = y;
  home_position_initialized_ = true;
  px4_home_position_set_ = false;
  updating_home_position_parameters_ = true;
  set_parameter(rclcpp::Parameter("home_position_x", home_position_x_));
  set_parameter(rclcpp::Parameter("home_position_y", home_position_y_));
  updating_home_position_parameters_ = false;
  publish_home_pose();
}

void WaywiserCopter::refresh_in_flight_status()
{
  if (!armed_) {
    in_flight_ = false;
    has_climb_position_ = false;
    return;
  }

  const bool was_in_flight = in_flight_;

  const float threshold = std::max(0.0F, in_flight_range_threshold_);
  const float landed_height_threshold = std::max(0.0F, threshold + kLandingHeightMargin);
  const float takeoff_height_threshold = std::max(0.0F, threshold + kTakeoffHeightMargin);
  const bool range_above_takeoff_threshold =
    px4_dist_bottom_valid_ && std::isfinite(px4_dist_bottom_) &&
    (px4_dist_bottom_ > takeoff_height_threshold);
  const bool altitude_above_takeoff_threshold =
    has_px4_altitude_ && std::isfinite(latest_px4_altitude_) &&
    (latest_px4_altitude_ > takeoff_height_threshold);
  const bool range_near_ground =
    px4_dist_bottom_valid_ && std::isfinite(px4_dist_bottom_) &&
    (px4_dist_bottom_ <= landed_height_threshold);
  const bool altitude_near_ground =
    has_px4_altitude_ && std::isfinite(latest_px4_altitude_) &&
    (latest_px4_altitude_ <= landed_height_threshold);
  const bool vertical_motion_active =
    has_px4_vertical_velocity_ && std::isfinite(latest_px4_vertical_velocity_) &&
    (std::fabs(latest_px4_vertical_velocity_) > kVerticalMotionInAirThreshold);
  const bool climb_command_active =
    current_cmd_vel_in_.linear.z > 0.01 ||
    current_cmd_vel_out_.linear.z > 0.01 ||
    climb_active_ ||
    (mCopterAutopilotComponent && mCopterAutopilotComponent->getAutoClimbActive());
  const bool takeoff_evidence =
    vertical_motion_active ||
    (climb_command_active && (range_above_takeoff_threshold || altitude_above_takeoff_threshold));
  const bool touchdown_cue = px4_ground_contact_ || px4_maybe_landed_ || px4_landed_;
  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const bool odom_near_ground = odom_position.getHeight() <= landed_height_threshold;
  const bool touchdown_evidence =
    !climb_command_active &&
    (px4_landed_ ||
     (touchdown_cue && !vertical_motion_active && (range_near_ground || altitude_near_ground)) ||
     (odom_near_ground && !vertical_motion_active));

  if (in_flight_) {
    in_flight_ = !touchdown_evidence;
    if (!in_flight_ && has_climb_position_) {
      has_landed_after_climb_ = true;
    }
    return;
  }

  in_flight_ = takeoff_evidence && !touchdown_evidence;
  if (in_flight_ && !was_in_flight) {
    has_landed_after_climb_ = false;
    climb_position_x_ = odom_position.getX();
    climb_position_y_ = odom_position.getY();
    has_climb_position_ = true;
    update_home_position_parameters(
      return_home_x_configured_ ? return_home_x_ : climb_position_x_,
      return_home_y_configured_ ? return_home_y_ : climb_position_y_);
    configure_px4_home_position();
  }
}

void WaywiserCopter::publish_odom()
{
  if (!odom_pub_) {
    return;
  }

  PosPoint odom_position = mCopterState->getPosition(PosType::odom);

  tf2::Quaternion q_odom;
  q_odom.setRPY(
    odom_position.getRoll() * DEG2RAD,
    odom_position.getPitch() * DEG2RAD,
    odom_position.getYaw() * DEG2RAD);

  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_msg.pose.pose.position.x = odom_position.getX();
  odom_msg.pose.pose.position.y = odom_position.getY();
  odom_msg.pose.pose.position.z = odom_position.getHeight();
  odom_msg.pose.pose.orientation = tf2::toMsg(q_odom);

  const auto velocity = mCopterState->getVelocity();
  odom_msg.twist.twist.linear.x = velocity.x;
  odom_msg.twist.twist.linear.y = velocity.y;
  odom_msg.twist.twist.linear.z = velocity.z;
  odom_msg.twist.twist.angular.z = mCopterState->getSteering();

  odom_pub_->publish(odom_msg);
}

void WaywiserCopter::publish_quadcopter_state()
{
  if (!quadcopter_state_pub_) {
    return;
  }

  current_state_ = derive_quadcopter_state();

  waywiser_core::msg::QuadcopterState msg;
  msg.stamp = now();
  msg.state_code = static_cast<uint8_t>(current_state_);

  switch (current_state_) {
    case HighLevelState::STARTING_UP:
      msg.state_str = "Starting up";
      break;
    case HighLevelState::READY_TO_ARM:
      msg.state_str = "Ready to arm";
      break;
    case HighLevelState::ARMING:
      msg.state_str = "Arming";
      break;
    case HighLevelState::ARMED:
      msg.state_str = "Armed";
      break;
    case HighLevelState::IN_FLIGHT:
      msg.state_str = "In flight";
      break;
    case HighLevelState::HOVERING:
      msg.state_str = "Hovering";
      break;
    case HighLevelState::LANDING:
      msg.state_str = "Landing";
      break;
    case HighLevelState::CLIMBING:
      msg.state_str = "Climbing";
      break;
    case HighLevelState::EMERGENCY:
      msg.state_str = "EMERGENCY STOP";
      break;
    case HighLevelState::ON_MISSION:
      msg.state_str = "On Mission";
      break;
    case HighLevelState::LANDED:
      msg.state_str = "Landed";
      break;
    case HighLevelState::RETURNING_HOME:
      msg.state_str = "Returning home";
      break;
  }

  quadcopter_state_pub_->publish(msg);
}

WaywiserCopter::HighLevelState WaywiserCopter::derive_quadcopter_state()
{
  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  const double landed_height_threshold = std::max(
    0.0, static_cast<double>(in_flight_range_threshold_ + kLandingHeightMargin));
  const bool odom_near_ground = odom_position.getHeight() <= landed_height_threshold;
  const bool vertical_motion_inactive = !has_px4_vertical_velocity_ ||
    std::fabs(latest_px4_vertical_velocity_) <= kVerticalMotionInAirThreshold;
  const bool landed_evidence = px4_landed_ || (px4_ground_contact_ && px4_maybe_landed_) ||
    (odom_near_ground && vertical_motion_inactive);
  const bool landed_after_flight = landed_evidence &&
    (has_climb_position_ || has_landed_after_climb_);
  const bool ready_for_arm = px4_ready_for_arm_command(
    ready_for_takeoff_, ready_for_offboard_, ready_to_arm_);
  const bool keep_landed_state = landed_after_flight && (armed_ || !ready_for_arm);
  const bool internal_mission_active =
    enable_autopilot_component_ &&
    mCopterAutopilotComponent &&
    mission_state_is_route_active(mCopterAutopilotComponent->getCurrentMissionState());
  const bool external_mission_active_recent =
    !enable_autopilot_component_ &&
    received_active_mission_status_ &&
    (get_clock()->now() - last_active_mission_status_time_).seconds() <= mission_state_timeout_;
  const bool mission_active = armed_ && (internal_mission_active || external_mission_active_recent);
  const double now_monotonic = steady_time_seconds();
  const bool input_command_active =
    last_input_command_time_ > 0.0 &&
    (now_monotonic - last_input_command_time_) <= kInputCommandTimeout;
  const bool climb_command_active =
    armed_ &&
    ((input_command_active &&
     (current_cmd_vel_in_.linear.z > 0.01 ||
     current_cmd_vel_out_.linear.z > 0.01)) ||
     climb_active_ ||
     (mCopterAutopilotComponent &&
     (mCopterAutopilotComponent->getAutoClimbActive() ||
     mCopterAutopilotComponent->getRouteClimbActive())));

  if (mEmergencyStopState && mEmergencyStopState->is_active()) {
    return HighLevelState::EMERGENCY;
  } else if (is_return_home_active()) {
    return HighLevelState::RETURNING_HOME;
  } else if (auto_landing_active_) {
    return HighLevelState::LANDING;
  } else if (mission_active) {
    return HighLevelState::ON_MISSION;
  } else if (climb_command_active) {
    return HighLevelState::CLIMBING;
  } else if (keep_landed_state) {
    return HighLevelState::LANDED;
  } else if (armed_) {
    if (mCopterAutopilotComponent &&
      (mCopterAutopilotComponent->getAutoClimbActive() ||
      mCopterAutopilotComponent->getRouteClimbActive()))
    {
      return HighLevelState::CLIMBING;
    } else if (in_flight_) {
      if (input_command_active && current_cmd_vel_out_.linear.z > 0.01) {
        return HighLevelState::CLIMBING;
      }
      if (!input_command_active) {
        return feature_hover_hold_enabled_ ? HighLevelState::HOVERING : HighLevelState::IN_FLIGHT;
      } else {
        return HighLevelState::IN_FLIGHT;
      }
    } else if ((input_command_active && current_cmd_vel_out_.linear.z > 0.01) || climb_active_) {
      return HighLevelState::CLIMBING;
    } else {
      return HighLevelState::ARMED;
    }
  } else {
    double now_monotonic = steady_time_seconds();
    if (last_arm_request_value_ &&
      (now_monotonic - last_arm_request_time_) < arm_transition_grace_period_)
    {
      return HighLevelState::ARMING;
    } else if (ready_for_arm) {
      return HighLevelState::READY_TO_ARM;
    }
  }

  return HighLevelState::STARTING_UP;
}

waywiser_core::msg::HeartbeatRxState WaywiserCopter::derive_control_tower_heartbeat_rx_state()
{
  waywiser_core::msg::HeartbeatRxState msg;
  msg.stamp = get_clock()->now();

  if (received_control_tower_heartbeat_) {
    msg.last_heartbeat_stamp = last_control_tower_heartbeat_stamp_;
    const auto now_time = get_clock()->now();
    msg.age_s = static_cast<float>(
      std::max(0.0, (now_time - last_control_tower_heartbeat_stamp_).seconds()));
  } else {
    msg.last_heartbeat_stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    msg.age_s = -1.0F;
  }

  const bool heartbeat_missing_timed_out = !received_control_tower_heartbeat_ &&
    heartbeat_monitor_started_ &&
    control_tower_heartbeat_monitor_age() > control_tower_heartbeat_timeout_;

  if (heartbeat_missing_timed_out) {
    msg.state = waywiser_core::msg::HeartbeatRxState::TIMEOUT;
  } else if (!received_control_tower_heartbeat_) {
    msg.state = waywiser_core::msg::HeartbeatRxState::NO_HEARTBEAT;
  } else if (control_tower_timeout_return_home_active_ || control_tower_heartbeat_timeout_emergency_stop_active_) {
    msg.state = waywiser_core::msg::HeartbeatRxState::TIMEOUT;
  } else if (msg.age_s > control_tower_heartbeat_timeout_) {
    msg.state = waywiser_core::msg::HeartbeatRxState::TIMEOUT;
  } else if (manual_arm_heartbeat_grace_active()) {
    msg.state = waywiser_core::msg::HeartbeatRxState::ACTIVE;
  } else {
    msg.state = waywiser_core::msg::HeartbeatRxState::ACTIVE;
  }

  return msg;
}

bool WaywiserCopter::control_tower_heartbeat_timed_out()
{
  return derive_control_tower_heartbeat_rx_state().state ==
         waywiser_core::msg::HeartbeatRxState::TIMEOUT;
}

double WaywiserCopter::control_tower_heartbeat_monitor_age()
{
  if (received_control_tower_heartbeat_) {
    return std::max(0.0, (get_clock()->now() - last_control_tower_heartbeat_stamp_).seconds());
  }

  if (!heartbeat_monitor_started_) {
    return 0.0;
  }

  return std::max(0.0, (get_clock()->now() - heartbeat_monitor_start_time_).seconds());
}

bool WaywiserCopter::manual_arm_heartbeat_grace_active()
{
  return last_arm_request_value_ &&
       (steady_time_seconds() - last_arm_request_time_) <=
         manual_arm_heartbeat_grace_period_;
}

void WaywiserCopter::publish_control_tower_heartbeat_rx_state()
{
  if (!control_tower_heartbeat_rx_state_pub_) {
    return;
  }

  const auto msg = derive_control_tower_heartbeat_rx_state();

  control_tower_heartbeat_rx_state_pub_->publish(msg);
}

void WaywiserCopter::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  if (mCopterAutopilotComponent && mCopterAutopilotComponent->isActive()) {
    mCopterAutopilotComponent->stopWaypointFollower();
  }
  process_twist_msg(twist_msg);
}

void WaywiserCopter::autopilot_state_control_callback(
  const std_msgs::msg::Bool::SharedPtr bool_msg)
{
  if (bool_msg->data && control_tower_heartbeat_timed_out()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring autopilot enable request: Control Tower heartbeat is timed out.");
    return;
  }
  mCopterAutopilotComponent->switchAutopilot(bool_msg->data);
}

void WaywiserCopter::mission_status_callback(
  const waywiser_core::msg::MissionState::SharedPtr msg)
{
  const auto mission_state = static_cast<MissionState>(msg->state);
  const bool external_mission_active = mission_state_is_active(mission_state);
  const bool is_rth_state = mission_state_is_return_home_active(mission_state);

  if (external_mission_active && is_rth_state &&
    (is_return_home_active() || return_home_completed_ ||
    current_state_ == HighLevelState::RETURNING_HOME ||
    current_state_ == HighLevelState::LANDED))
  {
    return;
  }
  if (external_mission_active && !is_rth_state &&
    ignore_non_rth_mission_status_until_idle_ &&
    is_return_home_active())
  {
    return;
  }
  if (external_mission_active && !is_rth_state) {
    if (inhibit_auto_arm_after_auto_land_) {
      RCLCPP_INFO(
        get_logger(),
        "Active mission status received. Re-enabling auto arm after auto landing.");
    }
    inhibit_auto_arm_after_auto_land_ = false;
  }

  if (external_mission_active && auto_landing_active_) {
    auto_landing_active_ = false;
    auto_landing_disarm_requested_ = false;
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    RCLCPP_INFO(get_logger(), "Active mission status received. Cancelling auto landing.");
  }

  received_active_mission_status_ = external_mission_active;
  if (!received_active_mission_status_) {
    ignore_non_rth_mission_status_until_idle_ = false;
  }
  if (received_active_mission_status_) {
    last_active_mission_status_time_ = get_clock()->now();

    // If the primary node receives an active mission state from the waypoint follower
    // that is NOT an RTH state (meaning a new actual route mission was started),
    // we clear all RTH flags so the vehicle can auto-arm and follow the route.
    if (!is_rth_state && (is_return_home_active() || return_home_completed_)) {
      clear_return_home_failsafe();
      ignore_non_rth_mission_status_until_idle_ = false;
      manual_return_home_active_ = false;
      return_home_completed_ = false;
      return_home_disarm_requested_ = false;
      has_last_return_home_disarm_request_time_ = false;
      if (mRthAutopilotComponent) {
        mRthAutopilotComponent->clearWaypointFollowerRoute();
      }
      current_cmd_vel_in_ = geometry_msgs::msg::Twist();
      current_cmd_vel_out_ = geometry_msgs::msg::Twist();
      if (enable_px4_bridge_ && armed_) {
        retry_offboard_for_mission_after_rth_land_ = true;
        send_offboard_mode_command();
        last_mode_request_time_ = get_clock()->now();
        has_last_mode_request_time_ = true;
        RCLCPP_INFO(
          get_logger(),
          "New mission interrupted return-home landing. Requested PX4 Offboard mode.");
      }
      RCLCPP_INFO(get_logger(), "Active mission status received. Resetting RTH state.");
    }
  }
  publish_quadcopter_state();
}

void WaywiserCopter::control_tower_heartbeat_callback(
  const std_msgs::msg::Header::SharedPtr msg)
{
  if (msg->stamp.sec == 0 && msg->stamp.nanosec == 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring Control Tower heartbeat without a valid generation timestamp.");
    return;
  }
  const auto now_time = get_clock()->now();
  last_control_tower_heartbeat_stamp_ =
    rclcpp::Time(msg->stamp, get_clock()->get_clock_type());
  received_control_tower_heartbeat_ = true;

  const double heartbeat_age =
    std::max(0.0, (now_time - last_control_tower_heartbeat_stamp_).seconds());
  // Only cancel the heartbeat-timeout RTH when the heartbeat-owned flags are active.
  // RTH completion state is independent and not touched here.
  if ((control_tower_timeout_return_home_active_ || control_tower_heartbeat_timeout_emergency_stop_active_) &&
    heartbeat_age <= control_tower_heartbeat_timeout_)
  {
    cancel_return_home_failsafe_on_heartbeat_restore();
    RCLCPP_INFO(
      get_logger(),
      "Control tower heartbeat restored. Cancelling heartbeat return-to-home failsafe.");
    publish_quadcopter_state();
  }
  publish_control_tower_heartbeat_rx_state();
}

void WaywiserCopter::path_with_twists_callback(
  const waywiser_core::msg::PathWithTwists::SharedPtr msg)
{
  if (control_tower_heartbeat_timed_out()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring mission route: Heartbeat timed out.");
    return;
  }

  if (!enable_px4_bridge_) {
    observed_drone_has_been_on_mission_ = false;
    mission_idle_published_after_observed_drone_left_mission_ = false;
  }
  inhibit_auto_arm_after_auto_land_ = false;

  // If RTH was active or completed, a new incoming mission means the user wants
  // to start a fresh flight or interrupt the current RTH.
  // We clear all RTH flags so the route and auto-arm logic proceed normally.
  if (is_return_home_active() || return_home_completed_) {
    clear_return_home_failsafe();
    manual_return_home_active_ = false;
    return_home_completed_ = false;
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    if (enable_px4_bridge_ && armed_) {
      retry_offboard_for_mission_after_rth_land_ = true;
      send_offboard_mode_command();
      last_mode_request_time_ = get_clock()->now();
      has_last_mode_request_time_ = true;
      RCLCPP_INFO(
        get_logger(),
        "New mission interrupted return-home landing. Requested PX4 Offboard mode.");
    }
    RCLCPP_INFO(get_logger(), "New mission received. Resetting RTH state and overriding with new mission.");
  }

  if (auto_landing_active_) {
    auto_landing_active_ = false;
    auto_landing_disarm_requested_ = false;
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    RCLCPP_INFO(get_logger(), "New mission received. Cancelling auto landing and overriding with new mission.");
  }
  QList<PosPoint> waypointList;
  for (size_t i = 0; i < msg->path.poses.size(); ++i) {
    PosPoint currentPoint;
    const auto & pose = msg->path.poses[i].pose;
    currentPoint.setX(pose.position.x);
    currentPoint.setY(pose.position.y);
    currentPoint.setHeight(pose.position.z);
    currentPoint.setYaw(tf2::getYaw(pose.orientation) * 180.0 / M_PI);
    if (i < msg->twists.size()) {
      currentPoint.setSpeed(std::hypot(
          msg->twists[i].linear.x,
          msg->twists[i].linear.y,
          msg->twists[i].linear.z));
    } else {
      currentPoint.setSpeed(mCopterAutopilotComponent->getCruiseSpeed());
    }

    waypointList.append(currentPoint);
  }

  bool is_new_route = true;
  const auto existing_list = mCopterAutopilotComponent->getWaypointList();
  if (existing_list.size() == waypointList.size()) {
    is_new_route = false;
    for (int i = 0; i < existing_list.size(); ++i) {
      if (std::abs(existing_list[i].getX() - waypointList[i].getX()) > 0.01 ||
          std::abs(existing_list[i].getY() - waypointList[i].getY()) > 0.01 ||
          std::abs(existing_list[i].getHeight() - waypointList[i].getHeight()) > 0.01) {
        is_new_route = true;
        break;
      }
    }
  }

  if (is_new_route) {
    mCopterAutopilotComponent->updateWaypointFollowerRoute(waypointList);

    if (waypoint_follower_route_activation_allowed()) {
      // Guarantee the autopilot activates when a new route is explicitly pushed.
      // This bypasses potential ROS 2 message deduplication if the UI sends True repeatedly.
      mCopterAutopilotComponent->switchAutopilot(true);
      mCopterAutopilotComponent->processMissionStateMachine();
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Mission route received and stored. Waiting for armed vehicle state before activating waypoint follower.");
    }
  }
  if (mission_status_pub_) {
    waywiser_core::msg::MissionState missionStateMsg;
    missionStateMsg.state = static_cast<uint8_t>(
      waypoint_follower_route_activation_allowed() ?
      mCopterAutopilotComponent->getCurrentMissionState() :
      MissionState::WaitingForVehicleInit);
    mission_status_pub_->publish(missionStateMsg);
  }
  publish_quadcopter_state();
}

void WaywiserCopter::fused_nav_sat_fix_extended_callback(
  const waywiser_core::msg::NavSatFixExtended::SharedPtr msg)
{
  if (!mCopterState->isEnuReferenceSet()) {
    return;
  }

  xyz_t xyz = coordinateTransforms::llhToEnu(
    mCopterState->getEnuRef(), {msg->latitude, msg->longitude, msg->altitude});

  PosPoint pos_point = mCopterState->getPosition(PosType::fused);
  pos_point.setType(PosType::fused);
  pos_point.setXYZ(xyz);
  pos_point.setYaw(coordinateTransforms::yawNEDtoENU(msg->yaw));
  pos_point.setRoll(msg->roll);
  pos_point.setPitch(-msg->pitch);
  pos_point.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
  mCopterState->setPosition(pos_point);
  GnssFixStatus gnssFixStatus;
  gnssFixStatus.isFusedOnChip = msg->is_fused_on_chip;
  gnssFixStatus.fixType = static_cast<GNSS_FIX_TYPE>(msg->fix_type);
  gnssFixStatus.horizontalAccuracy = msg->horizontal_accuracy;
  gnssFixStatus.verticalAccuracy = msg->vertical_accuracy;
  gnssFixStatus.headingAccuracy = msg->heading_accuracy;
  gnssFixStatus.lastRtcmCorrectionAge = msg->last_rtcm_correction_age;
  gnssFixStatus.numSatellites = msg->num_satellites;

  if (mCopterAutopilotComponent) {
    mCopterAutopilotComponent->setGnssFixStatus(gnssFixStatus);
    if (mCopterAutopilotComponent->getMissionPosTypeUsed() == PosType::fused) {
      mCopterAutopilotComponent->setVehicleInitialized(true);
    }
  }
  if (mRthAutopilotComponent) {
    mRthAutopilotComponent->setGnssFixStatus(gnssFixStatus);
    if (mRthAutopilotComponent->getMissionPosTypeUsed() == PosType::fused) {
      mRthAutopilotComponent->setVehicleInitialized(true);
    }
  }
}

void WaywiserCopter::range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (std::isfinite(msg->range)) {
    received_range_data_ = true;
    px4_dist_bottom_ = msg->range;
    px4_dist_bottom_valid_ = true;
    refresh_in_flight_status();
    publish_quadcopter_state();
  }
}

void WaywiserCopter::process_twist_msg(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  auto output = *twist_msg;
  if ((mEmergencyStopState && mEmergencyStopState->is_active()) ||
    is_return_home_active() || control_tower_heartbeat_timed_out())
  {
    output = geometry_msgs::msg::Twist();
  }

  bool manual_input = std::abs(output.linear.x) > kInputCommandThreshold ||
    std::abs(output.linear.y) > kInputCommandThreshold ||
    std::abs(output.linear.z) > kInputCommandThreshold ||
    std::abs(output.angular.z) > kInputCommandThreshold;

  if (manual_input) {
    last_input_command_time_ = steady_time_seconds();
    if (mCopterAutopilotComponent && mCopterAutopilotComponent->getAutoClimbActive()) {
      RCLCPP_INFO(get_logger(), "Auto climb CANCELLED by manual input.");
      (void)mCopterAutopilotComponent->setAutoClimbActive(false);
    }
  }

  if (!auto_landing_active_ &&
    !auto_landing_disarm_requested_ &&
    output.linear.z > kInputCommandThreshold)
  {
    if (inhibit_auto_arm_after_auto_land_) {
      inhibit_auto_arm_after_auto_land_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Climb command received. Re-enabling auto arm after auto landing.");
    }
    // If a prior RTH completed (return_home_completed_ == true) but no RTH is currently active,
    // a climb request should reset the RTH-completed state so auto-arm is not blocked.
    // This handles the case where the auto_climb service targets the waypoint follower node only,
    // leaving this (drone) node's return_home_completed_ flag set after a manual RTH.
    if (return_home_completed_ && !is_return_home_active()) {
      clear_return_home_failsafe();
      manual_return_home_active_ = false;
      return_home_completed_ = false;
      return_home_disarm_requested_ = false;
      has_last_return_home_disarm_request_time_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Climb command received after RTH. Clearing RTH-completed state to allow re-arm.");
    }
  }

  // When auto_landing is active, zero the incoming velocity so the landing override wins.
  // When auto_landing was just disabled, the follower may still be publishing its last
  // descent velocity for a brief window. Suppress purely-downward cmd_vel in this case
  // so hover hold can engage cleanly instead of the stale descent passing through.
  const bool auto_climb_requested =
    mCopterAutopilotComponent && mCopterAutopilotComponent->getAutoClimbActive();
  const bool incoming_is_pure_descent =
    !auto_climb_requested &&
    output.linear.z < -kInputCommandThreshold &&
    std::abs(output.linear.x) <= kInputCommandThreshold &&
    std::abs(output.linear.y) <= kInputCommandThreshold &&
    std::abs(output.angular.z) <= kInputCommandThreshold;
  if (auto_landing_active_ || (!auto_landing_active_ && incoming_is_pure_descent)) {
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
  } else {
    current_cmd_vel_in_ = output;
  }
}

void WaywiserCopter::publish_command()
{
  if (!cmd_vel_out_pub_) {
    return;
  }

  if (!enable_px4_bridge_ &&
    observed_quadcopter_state_received_ &&
    observed_drone_has_been_on_mission_ &&
    !observed_drone_on_mission_ &&
    !(mCopterAutopilotComponent && mCopterAutopilotComponent->getAutoClimbActive()))
  {
    const auto current_mission_state = mCopterAutopilotComponent ?
      mCopterAutopilotComponent->getCurrentMissionState() :
      MissionState::Idle;
    const bool route_mission_active =
      mCopterAutopilotComponent &&
      mission_state_is_route_active(current_mission_state);
    if (!route_mission_active) {
      current_cmd_vel_out_ = geometry_msgs::msg::Twist();
      current_cmd_vel_in_ = geometry_msgs::msg::Twist();
      return;
    }
  }

  const bool autopilot_active = mCopterAutopilotComponent && mCopterAutopilotComponent->isActive();
  const bool external_command_stale =
    last_input_command_time_ > 0.0 &&
    (steady_time_seconds() - last_input_command_time_) > command_timeout_;
  if (external_command_stale &&
    !autopilot_active &&
    !is_return_home_active() &&
    !auto_landing_active_)
  {
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    last_input_command_time_ = 0.0;
    climb_active_ = false;
    climb_saved_v_up_ = 0.0F;
  }

  auto output = current_cmd_vel_in_;

  if (mCopterAutopilotComponent) {
    static bool was_autopilot_active = false;
    bool is_autopilot_active = mCopterAutopilotComponent->isActive();

    if (is_autopilot_active) {
      output = mCopterAutopilotComponent->getAutopilotTwistCommand();
      {
        // Suppress horizontal motion and yaw until drone is above min_steering_height_ AGL.
        const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
        const double odom_height = odom_position.getHeight();
        float height_agl = 0.0F;
        if (px4_dist_bottom_valid_ && std::isfinite(px4_dist_bottom_)) {
          height_agl = std::max(height_agl, px4_dist_bottom_);
        }
        if (has_px4_altitude_ && std::isfinite(latest_px4_altitude_)) {
          height_agl = std::max(height_agl, latest_px4_altitude_);
        }
        if (std::isfinite(odom_height)) {
          height_agl = std::max(height_agl, static_cast<float>(odom_height));
        }
        if (height_agl < static_cast<float>(min_steering_height_)) {
          output.linear.x = 0.0;
          output.linear.y = 0.0;
          output.angular.z = 0.0;
        }
      }
      last_input_command_time_ = steady_time_seconds();
      if (feature_auto_arm_enabled_ && !return_home_completed_ && !control_tower_heartbeat_timed_out() &&
        !armed_ && output.linear.z > 0.001)
      {
        request_arm_state(true);
      }
    } else if (was_autopilot_active) {
      output = geometry_msgs::msg::Twist();
      last_input_command_time_ = 0.0;

      if (mCopterAutopilotComponent->getCurrentMissionState() == MissionState::Idle) {
        const auto & waypointList = mCopterAutopilotComponent->getWaypointList();
        if (!waypointList.isEmpty()) {
          const PosPoint goal = waypointList.last();
          xyz_t enu = {goal.getX(), goal.getY(), goal.getHeight()};
          xyz_t ned = coordinateTransforms::enuToNED(enu);
          hold_position_ned_ = {static_cast<float>(ned.x), static_cast<float>(ned.y), static_cast<float>(ned.z)};
          has_hold_position_ = true;
          was_command_active_ = false;
          RCLCPP_INFO(get_logger(), "Mission finished. Snapping hover hold to final goal coordinates.");
        }
      }
    }
    was_autopilot_active = is_autopilot_active;
  }

  const bool suppress_for_heartbeat_timeout =
    control_tower_heartbeat_timed_out() && !control_tower_timeout_return_home_active_;

  bool rth_twist_applied = false;
  if (is_return_home_active() && mRthAutopilotComponent && mRthAutopilotComponent->isActive()) {
    output = mRthAutopilotComponent->getAutopilotTwistCommand();
    {
      const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
      const double odom_height = odom_position.getHeight();
      float height_agl = 0.0F;
      if (px4_dist_bottom_valid_ && std::isfinite(px4_dist_bottom_)) {
        height_agl = std::max(height_agl, px4_dist_bottom_);
      }
      if (has_px4_altitude_ && std::isfinite(latest_px4_altitude_)) {
        height_agl = std::max(height_agl, latest_px4_altitude_);
      }
      if (std::isfinite(odom_height)) {
        height_agl = std::max(height_agl, static_cast<float>(odom_height));
      }
      if (height_agl < static_cast<float>(min_steering_height_)) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        output.angular.z = 0.0;
      }
    }
    rth_twist_applied = true;
    last_input_command_time_ = steady_time_seconds();
  }

  if (is_return_home_active() && !rth_twist_applied &&
    (return_home_landing_phase_active_ || return_home_ready_to_land()))
  {
    const float descent_velocity = constrained_descent_velocity(
      auto_landing_descent_velocity_, has_px4_vertical_velocity_, latest_px4_vertical_velocity_);
    output.linear.x = 0.0;
    output.linear.y = 0.0;
    output.linear.z = -descent_velocity;
    output.angular.z = 0.0;
  }
  if ((mEmergencyStopState && mEmergencyStopState->is_active()) || suppress_for_heartbeat_timeout)
  {
    output = geometry_msgs::msg::Twist();
  }

  if (auto_landing_active_) {
    output.linear.x = 0.0;
    output.linear.y = 0.0;
    output.linear.z = -auto_landing_descent_velocity_;
    output.angular.z = 0.0;

    const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
    const double landed_height_threshold = std::max(
      0.0, static_cast<double>(in_flight_range_threshold_ + kLandingHeightMargin));
    const bool odom_near_ground = odom_position.getHeight() <= landed_height_threshold;
    const bool vertical_motion_inactive = !has_px4_vertical_velocity_ ||
      std::fabs(latest_px4_vertical_velocity_) <= kVerticalMotionInAirThreshold;
    const bool px4_landed_evidence = px4_landed_ || (px4_ground_contact_ && px4_maybe_landed_);
    const bool landed_evidence = px4_landed_evidence || (odom_near_ground && vertical_motion_inactive);

    if (armed_ && landed_evidence && !auto_landing_disarm_requested_) {
      RCLCPP_INFO(get_logger(), "Vehicle landed. Requesting disarm...");
      send_arm_command(false, true);
      last_arm_request_value_ = false;
      auto_landing_disarm_requested_ = true;
    }
  }

  current_cmd_vel_out_ = output;

  // Handle Auto Arm/Disarm logic
  if (feature_auto_arm_enabled_ &&
    !auto_landing_active_ &&
    !auto_landing_disarm_requested_ &&
    !inhibit_auto_arm_after_auto_land_ &&
    !return_home_completed_)
  {
    if (!armed_ && output.linear.z > 0.001) {
      request_arm_state(true);
    } else if (armed_ && !in_flight_ && output.linear.z < -0.001) {
      // Only auto-disarm if climb has not been initiated (prevent disarm on cmd gaps)
      if (!climb_active_) {
        request_arm_state(false);
      }
    }
  }

  // Track climb: once we see an upward command while armed-but-not-yet-in-flight,
  // save the velocity so publish_trajectory_setpoint can continue commanding it
  // even if the external cmd_vel pipeline goes momentarily silent (mux timeouts, etc.)
  if (armed_ && !in_flight_) {
    if (output.linear.z > 0.01F) {
      climb_active_ = true;
      climb_saved_v_up_ = static_cast<float>(output.linear.z);
    }
  } else {
    // Once airborne, clear the climb override
    climb_active_ = false;
    climb_saved_v_up_ = 0.0F;
  }

  auto movement_controller = mCopterInterfaceComponent->getMovementController();
  if (movement_controller) {
    movement_controller->setDesiredSpeed(output.linear.x);
    movement_controller->setDesiredSteering(output.angular.z);
  }

  cmd_vel_out_pub_->publish(output);
}

void WaywiserCopter::update_control_tower_heartbeat_failsafe()
{
  if (!enable_px4_bridge_) {
    return;
  }

  const bool heartbeat_timed_out = control_tower_heartbeat_timed_out();
  const bool failsafe_eligible = ready_to_arm_ || armed_ || in_flight_ ||
    control_tower_timeout_return_home_active_ || return_home_completed_;

  const auto now_time = get_clock()->now();
  const double heartbeat_age = control_tower_heartbeat_monitor_age();

  if (!manual_return_home_active_) {
    if (!received_control_tower_heartbeat_ && !heartbeat_timed_out) {
      clear_return_home_failsafe();
      return_home_completed_ = false;
      return_home_disarm_requested_ = false;
      has_last_return_home_disarm_request_time_ = false;
      return;
    }

    // Heartbeat is not yet timed out (or grace active): only cancel the
    // heartbeat-timeout RTH flag. RTH completion state is independent and
    // must not be touched here (RTH can be requested manually).
    if (!heartbeat_timed_out || manual_arm_heartbeat_grace_active()) {
      if (control_tower_timeout_return_home_active_) {
        cancel_return_home_failsafe_on_heartbeat_restore();
        RCLCPP_INFO(
          get_logger(),
          "Control tower heartbeat restored. Cancelling heartbeat return-to-home failsafe.");
      }
      return;
    }

    if (!failsafe_eligible) {
      clear_return_home_failsafe();
      return_home_disarm_requested_ = false;
      has_last_return_home_disarm_request_time_ = false;
      return;
    }

    if (!control_tower_timeout_return_home_active_ && !return_home_completed_) {
      const bool mission_active_before_rth =
        enable_autopilot_component_
        ? (mCopterAutopilotComponent &&
        mission_state_is_route_active(mCopterAutopilotComponent->getCurrentMissionState()))
        : received_active_mission_status_;
      control_tower_timeout_return_home_active_ = true;
      ignore_non_rth_mission_status_until_idle_ = true;
      received_active_mission_status_ = false;
      if (auto_landing_active_) {
        auto_landing_active_ = false;
        auto_landing_disarm_requested_ = false;
        RCLCPP_WARN(get_logger(), "Auto landing cancelled because heartbeat return-to-home is active.");
      }
      current_cmd_vel_out_ = geometry_msgs::msg::Twist();
      current_cmd_vel_in_ = geometry_msgs::msg::Twist();
      waiting_for_heartbeat_mission_active_ = mission_active_before_rth;
      RCLCPP_WARN(
        get_logger(),
        "Control tower heartbeat timed out after %.2f s. Starting return-home failsafe.",
        heartbeat_age);
      publish_quadcopter_state();
    }
  }

  if (!is_return_home_active() && !return_home_completed_) {
    return;
  }

  if (return_home_landed_at_home()) {
    const bool was_heartbeat_failsafe =
      control_tower_timeout_return_home_active_ && !manual_return_home_active_;
    clear_return_home_failsafe();
    return_home_completed_ = true;
    manual_return_home_active_ = false;
    if (was_heartbeat_failsafe) {
      publish_control_tower_timeout_emergency_stop(control_tower_heartbeat_monitor_age());
    }
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    const auto now_time = get_clock()->now();
    const bool disarm_request_due = !has_last_return_home_disarm_request_time_ ||
      (now_time - last_return_home_disarm_request_time_).seconds() >= return_home_command_retry_period_;
    if (armed_ && disarm_request_due) {
      return_home_disarm_requested_ = true;
      has_last_return_home_disarm_request_time_ = true;
      last_return_home_disarm_request_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Return-home landing complete. Requesting forced disarm.");
      send_arm_command(false, true);
      last_arm_request_value_ = false;
    }
    RCLCPP_INFO(get_logger(), "Return-home completed: vehicle landed at home.");
    return;
  }

  if (is_return_home_active() &&
    return_home_request_sent_ &&
    !return_home_land_request_sent_ &&
    return_home_ready_to_land())
  {
    send_px4_land_at_current_position_command();
    return;
  }

  if (return_home_completed_) {
    return;
  }

  if (is_return_home_active() &&
    !return_home_request_sent_ &&
    !return_home_land_request_sent_ &&
    return_home_position_reached())
  {
    send_px4_land_at_current_position_command();
    return_home_landing_phase_active_ = true;
    return;
  }

  if (is_return_home_active() &&
    return_home_request_sent_ && return_home_ready_to_land())
  {
    return_home_landing_phase_active_ = true;
  }

  if (!in_flight_) {
    if (control_tower_timeout_return_home_active_ && !manual_return_home_active_) {
      publish_control_tower_timeout_emergency_stop(heartbeat_age);
    }
    if (armed_) {
      const bool disarm_request_due = !has_last_return_home_disarm_request_time_ ||
        (now_time - last_return_home_disarm_request_time_).seconds() >= return_home_command_retry_period_;
      if (disarm_request_due) {
        return_home_disarm_requested_ = true;
        has_last_return_home_disarm_request_time_ = true;
        last_return_home_disarm_request_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Heartbeat timeout while landed. Requesting forced disarm.");
        send_arm_command(false, true);
        last_arm_request_value_ = false;
      }
    }
    return_home_completed_ = true;
    manual_return_home_active_ = false;
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    return;
  }

  const bool request_due =
    !return_home_request_sent_ &&
    (!has_last_return_home_request_time_ ||
    (now_time - last_return_home_request_time_).seconds() >= return_home_command_retry_period_);

  if (request_due) {
    if (send_return_home_command()) {
      return_home_request_sent_ = true;
    }
    last_return_home_request_time_ = now_time;
    has_last_return_home_request_time_ = true;
  }
}

void WaywiserCopter::publish_control_tower_timeout_emergency_stop(double heartbeat_age)
{
  if (control_tower_heartbeat_timeout_emergency_stop_active_) {
    return;
  }

  control_tower_heartbeat_timeout_emergency_stop_active_ = true;

  if (!emergency_stop_update_pub_) {
    RCLCPP_WARN(
      get_logger(),
      "Control tower heartbeat timed out and return-home completed, but emergency stop publisher is unavailable.");
    return;
  }

  std::stringstream emergency_stop_reason;
  emergency_stop_reason << "Control tower heartbeat timed out after " <<
    std::fixed << std::setprecision(2) << heartbeat_age <<
    " s; vehicle returned home and landed.";

  auto emergency_stop_msg = waywiser_twist_safety::msg::EmergencyStopState();
  emergency_stop_msg.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;
  emergency_stop_msg.sender_id = this->get_name();
  emergency_stop_msg.stamp = this->get_clock()->now();
  emergency_stop_msg.reason = emergency_stop_reason.str();
  emergency_stop_update_pub_->publish(emergency_stop_msg);

  RCLCPP_WARN(
    get_logger(),
    "Control tower heartbeat timed out and vehicle reached home. Activating emergency stop.");
}

rcl_interfaces::msg::SetParametersResult WaywiserCopter::on_parameter_set(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "feature_auto_arm_enabled") {
      feature_auto_arm_enabled_ = parameter.as_bool();
    } else if (parameter.get_name() == "feature_auto_land_enabled") {
      feature_auto_land_enabled_ = parameter.as_bool();
      if (!feature_auto_land_enabled_) {
        auto_landing_active_ = false;
        auto_landing_disarm_requested_ = false;
      }
    } else if (parameter.get_name() == "auto_landing_descent_velocity") {
      auto_landing_descent_velocity_ = std::max(0.0, parameter.as_double());
      if (mCopterAutopilotComponent) {
        mCopterAutopilotComponent->setDescentSpeed(auto_landing_descent_velocity_);
      }
    } else if (parameter.get_name() == "feature_hover_hold_enabled") {
      feature_hover_hold_enabled_ = parameter.as_bool();
    } else if (parameter.get_name() == "require_motion_before_engage") {
      require_motion_before_engage_ = parameter.as_bool();
    } else if (parameter.get_name() == "idle_descent_rate") {
      idle_descent_rate_ = std::max(0.0, parameter.as_double());
    } else if (parameter.get_name() == "hold_velocity_epsilon") {
      hold_velocity_epsilon_ = parameter.as_double();
    } else if (parameter.get_name() == "command_timeout") {
      command_timeout_ = parameter.as_double();
    } else if (parameter.get_name() == "request_retry_period") {
      request_retry_period_ = parameter.as_double();
    } else if (parameter.get_name() == "return_home_on_control_tower_timeout") {
      return_home_on_control_tower_timeout_ = parameter.as_bool();
    } else if (parameter.get_name() == "configure_px4_home_on_climb") {
      configure_px4_home_on_climb_ = parameter.as_bool();
    } else if (parameter.get_name() == "return_home_x") {
      return_home_x_configured_ =
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
      if (return_home_x_configured_) {
        return_home_x_ = parameter.as_double();
        home_position_x_ = return_home_x_;
        home_position_initialized_ = home_position_initialized_ ||
          ((return_home_x_configured_ || home_position_x_configured_) &&
          (return_home_y_configured_ || home_position_y_configured_));
        px4_home_position_set_ = false;
        publish_home_pose();
      }
    } else if (parameter.get_name() == "return_home_y") {
      return_home_y_configured_ =
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
      if (return_home_y_configured_) {
        return_home_y_ = parameter.as_double();
        home_position_y_ = return_home_y_;
        home_position_initialized_ = home_position_initialized_ ||
          ((return_home_x_configured_ || home_position_x_configured_) &&
          (return_home_y_configured_ || home_position_y_configured_));
        px4_home_position_set_ = false;
        publish_home_pose();
      }
    } else if (parameter.get_name() == "home_position_x") {
      home_position_x_ = parameter.as_double();
      if (!updating_home_position_parameters_) {
        home_position_x_configured_ = true;
        home_position_initialized_ =
          (return_home_x_configured_ || home_position_x_configured_) &&
          (return_home_y_configured_ || home_position_y_configured_);
      }
      px4_home_position_set_ = false;
      publish_home_pose();
    } else if (parameter.get_name() == "home_position_y") {
      home_position_y_ = parameter.as_double();
      if (!updating_home_position_parameters_) {
        home_position_y_configured_ = true;
        home_position_initialized_ =
          (return_home_x_configured_ || home_position_x_configured_) &&
          (return_home_y_configured_ || home_position_y_configured_);
      }
      px4_home_position_set_ = false;
      publish_home_pose();
    } else if (parameter.get_name() == "return_home_height") {
      return_home_height_ = std::max(0.0, parameter.as_double());
    } else if (parameter.get_name() == "control_tower_heartbeat_timeout") {
      control_tower_heartbeat_timeout_ = std::max(0.1, parameter.as_double());
    } else if (parameter.get_name() == "return_home_command_retry_period") {
      return_home_command_retry_period_ = std::max(0.1, parameter.as_double());
    } else if (parameter.get_name() == "feature_auto_climb_enabled") {
      mCopterAutopilotComponent->setAutoClimbEnabled(parameter.as_bool());
    } else if (parameter.get_name() == "auto_climb_height") {
      mCopterAutopilotComponent->setAutoClimbHeight(parameter.as_double());
    }
  }

  publish_quadcopter_state();
  return result;
}

void WaywiserCopter::publish_tfs()
{
  const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
  tf2::Quaternion odom_q;
  odom_q.setRPY(
    odom_position.getRoll() * DEG2RAD,
    odom_position.getPitch() * DEG2RAD,
    odom_position.getYaw() * DEG2RAD);

  if (publish_odom_to_baselink_tf_) {
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = now();
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = odom_position.getX();
    odom_to_base.transform.translation.y = odom_position.getY();
    odom_to_base.transform.translation.z = odom_position.getHeight();
    odom_to_base.transform.rotation = tf2::toMsg(odom_q);
    tf_pub_->sendTransform(odom_to_base);
  }

  if (publish_world_to_odom_tf_) {
    geometry_msgs::msg::TransformStamped world_to_odom;
    world_to_odom.header.stamp = now();
    world_to_odom.header.frame_id = world_frame_;
    world_to_odom.child_frame_id = odom_frame_;
    world_to_odom.transform.rotation.w = 1.0;

    PosPoint world_position = mCopterState->getPosition(PosType::fused);
    if (!world_position.getTime().isNull()) {
      geometry_msgs::msg::Transform odom_to_base_msg;
      odom_to_base_msg.translation.x = odom_position.getX();
      odom_to_base_msg.translation.y = odom_position.getY();
      odom_to_base_msg.translation.z = odom_position.getHeight();
      odom_to_base_msg.rotation = tf2::toMsg(odom_q);

      tf2::Transform odom_to_base_tf;
      tf2::fromMsg(odom_to_base_msg, odom_to_base_tf);

      tf2::Quaternion world_q;
      world_q.setRPY(
        world_position.getRoll() * DEG2RAD,
        world_position.getPitch() * DEG2RAD,
        world_position.getYaw() * DEG2RAD);

      geometry_msgs::msg::Transform world_to_base_msg;
      world_to_base_msg.translation.x = world_position.getX();
      world_to_base_msg.translation.y = world_position.getY();
      world_to_base_msg.translation.z = world_position.getHeight();
      world_to_base_msg.rotation = tf2::toMsg(world_q);

      tf2::Transform world_to_base_tf;
      tf2::fromMsg(world_to_base_msg, world_to_base_tf);

      tf2::toMsg(world_to_base_tf * odom_to_base_tf.inverse(), world_to_odom.transform);
    }

    tf_pub_->sendTransform(world_to_odom);
  }
}

void WaywiserCopter::publish_world_pose()
{
  if (!vehicle_pose_pub_) {
    return;
  }

  PosPoint world_position = mCopterState->getPosition(PosType::fused);
  if (world_position.getTime().isNull()) {
    world_position = mCopterState->getPosition(PosType::odom);
  }

  tf2::Quaternion world_q;
  world_q.setRPY(
    world_position.getRoll() * DEG2RAD,
    world_position.getPitch() * DEG2RAD,
    world_position.getYaw() * DEG2RAD);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = world_frame_;
  pose_msg.pose.position.x = world_position.getX();
  pose_msg.pose.position.y = world_position.getY();
  pose_msg.pose.position.z = world_position.getHeight();
  pose_msg.pose.orientation = tf2::toMsg(world_q);
  vehicle_pose_pub_->publish(pose_msg);
}

void WaywiserCopter::qtMessageHandler(
  QtMsgType type,
  const QMessageLogContext &,
  const QString & msg)
{
  CoreUtils::qtMessageToLogger(node_logger_, type, msg);
}

// ── PX4 offboard setpoint bridge ──────────────────────────────────────────────

void WaywiserCopter::px4_vehicle_command_ack_callback(
  const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME &&
    msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
  {
    px4_home_position_hpos_valid_ = true;
    px4_home_position_alt_valid_ = true;
    px4_home_position_set_ = true;
  }

  if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
    msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM ||
    msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN ||
    msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME ||
    msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND ||
    msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
  {
    RCLCPP_INFO(
      get_logger(),
      "PX4 command ack: command=%d result=%d result_param1=%d",
      static_cast<int>(msg->command),
      static_cast<int>(msg->result),
      static_cast<int>(msg->result_param1));
  }
}

void WaywiserCopter::setpoint_timer_callback()
{
  const bool motion_requested = command_requests_motion();
  const bool force_rth_landing_velocity = is_return_home_active() &&
    (return_home_landing_phase_active_ || return_home_ready_to_land());

  if (feature_hover_hold_enabled_ && was_command_active_ && !motion_requested &&
    has_vehicle_local_position_)
  {
    hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
    has_hold_position_ = true;
  }
  was_command_active_ = motion_requested;

  const bool use_position_hold =
    feature_hover_hold_enabled_ && !motion_requested && has_vehicle_local_position_ &&
    !force_rth_landing_velocity;
  publish_offboard_control_mode(use_position_hold);
  publish_trajectory_setpoint();

  setpoint_count_++;

  // Auto offboard mode switching
  const bool mode_request_due =
    !has_last_mode_request_time_ ||
    (get_clock()->now() - last_mode_request_time_).seconds() >= request_retry_period_;

  if (setpoint_count_ >= required_setpoint_count_ &&
    engagement_requested() &&
    local_position_stable() &&
    !is_offboard_px4() &&
    mode_request_due)
  {
    send_offboard_mode_command();
    last_mode_request_time_ = get_clock()->now();
    has_last_mode_request_time_ = true;
    RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode.");
  }

  if (retry_offboard_for_mission_after_rth_land_) {
    if (!armed_ || !received_active_mission_status_ || is_offboard_px4()) {
      retry_offboard_for_mission_after_rth_land_ = false;
    } else {
      const bool retry_mode_request_due =
        !has_last_mode_request_time_ ||
        (get_clock()->now() - last_mode_request_time_).seconds() >= request_retry_period_;
      if (setpoint_count_ >= required_setpoint_count_ &&
        local_position_stable() &&
        retry_mode_request_due)
      {
        send_offboard_mode_command();
        last_mode_request_time_ = get_clock()->now();
        has_last_mode_request_time_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Retrying PX4 Offboard mode request for mission after interrupted return-home landing.");
      }
    }
  }

  // Auto arm from setpoint pre-stream
  if (feature_auto_arm_enabled_ &&
    !auto_landing_active_ &&
    !auto_landing_disarm_requested_ &&
    !inhibit_auto_arm_after_auto_land_ &&
    !return_home_completed_ &&
    setpoint_count_ >= required_setpoint_count_ &&
    engagement_requested() &&
    is_offboard_px4() &&
    local_position_stable() &&
    !armed_)
  {
    request_arm_state(true);
  }
}

void WaywiserCopter::publish_offboard_control_mode(bool use_position_hold)
{
  if (!offboard_control_mode_pub_) {return;}

  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  msg.position = use_position_hold;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_pub_->publish(msg);
}

void WaywiserCopter::publish_trajectory_setpoint()
{
  if (!trajectory_setpoint_pub_) {return;}

  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);

  const float yaw = has_vehicle_local_position_ ? vehicle_heading_ : 0.0F;
  const bool motion_requested = command_requests_motion();
  const bool force_rth_landing_velocity = is_return_home_active() &&
    (return_home_landing_phase_active_ || return_home_ready_to_land());

  if (force_rth_landing_velocity) {
    const float descent_velocity = constrained_descent_velocity(
      auto_landing_descent_velocity_, has_px4_vertical_velocity_, latest_px4_vertical_velocity_);
    msg.position = {kNaN, kNaN, kNaN};
    msg.velocity = {0.0F, 0.0F, descent_velocity};
    msg.yawspeed = 0.0F;
    msg.yaw = kNaN;
    msg.acceleration = {kNaN, kNaN, kNaN};
    msg.jerk = {kNaN, kNaN, kNaN};
    trajectory_setpoint_pub_->publish(msg);
    return;
  }

  if (feature_hover_hold_enabled_ && !motion_requested && has_vehicle_local_position_) {
    if (!has_hold_position_) {
      hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
      has_hold_position_ = true;
    }
    msg.position = {hold_position_ned_[0], hold_position_ned_[1], hold_position_ned_[2]};
    msg.velocity = {kNaN, kNaN, kNaN};
    msg.yawspeed = kNaN;
    msg.yaw = yaw;
  } else {
    msg.position = {kNaN, kNaN, kNaN};

    const bool command_active =
      last_input_command_time_ > 0.0 &&
      (steady_time_seconds() - last_input_command_time_) <= command_timeout_;

    float v_forward = command_active ?
      static_cast<float>(current_cmd_vel_out_.linear.x) : 0.0F;
    float v_left = command_active ?
      static_cast<float>(current_cmd_vel_out_.linear.y) : 0.0F;
    float v_up;
    if (climb_active_ && !in_flight_) {
      // During climb, always command the last seen upward velocity regardless of
      // command_active, so brief gaps in the external cmd_vel pipeline (mux timeouts)
      // cannot stall the ascent or cause CLIMBING<->ARMED state flapping.
      v_up = climb_saved_v_up_;
    } else if (feature_hover_hold_enabled_ || motion_requested) {
      v_up = command_active ? static_cast<float>(current_cmd_vel_out_.linear.z) : 0.0F;
    } else {
      v_up = -static_cast<float>(idle_descent_rate_);
    }

    const float v_north = v_forward * std::cos(yaw) - v_left * std::sin(yaw);
    const float v_east = v_forward * std::sin(yaw) + v_left * std::cos(yaw);
    const float v_down = -v_up;

    msg.velocity = {v_north, v_east, v_down};
    msg.yawspeed = command_active ? static_cast<float>(-current_cmd_vel_out_.angular.z) : 0.0F;
    msg.yaw = kNaN;
  }

  msg.acceleration = {kNaN, kNaN, kNaN};
  msg.jerk = {kNaN, kNaN, kNaN};
  trajectory_setpoint_pub_->publish(msg);
}

bool WaywiserCopter::local_position_ready() const
{
  if (!require_local_position_before_arm_) {return true;}
  return local_position_xy_valid_ && local_position_z_valid_ &&
         local_position_v_xy_valid_ && local_position_v_z_valid_;
}

bool WaywiserCopter::local_position_stable()
{
  if (!require_local_position_before_arm_) {return true;}
  if (!local_position_ready() || !has_local_position_ready_since_) {return false;}
  return (get_clock()->now() - local_position_ready_since_).seconds() >=
         local_position_ready_duration_;
}

bool WaywiserCopter::command_requests_motion()
{
  if (last_input_command_time_ <= 0.0) {return false;}
  if ((steady_time_seconds() - last_input_command_time_) > command_timeout_) {return false;}
  return std::abs(current_cmd_vel_out_.linear.x) > hold_velocity_epsilon_ ||
         std::abs(current_cmd_vel_out_.linear.y) > hold_velocity_epsilon_ ||
         std::abs(current_cmd_vel_out_.linear.z) > hold_velocity_epsilon_ ||
         std::abs(current_cmd_vel_out_.angular.z) > hold_velocity_epsilon_;
}

bool WaywiserCopter::engagement_requested()
{
  if (!require_motion_before_engage_) {return true;}
  return command_requests_motion();
}

bool WaywiserCopter::is_offboard_px4() const
{
  return px4_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

void WaywiserCopter::publish_route_markers()
{
  if (!route_marker_pub_ || !publish_waypoint_markers_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const std::string marker_ns = std::string(this->get_name()) + "/route_markers";

  // Delete previous markers
  visualization_msgs::msg::Marker del_marker;
  del_marker.header.frame_id = world_frame_;
  del_marker.header.stamp = this->get_clock()->now();
  del_marker.ns = marker_ns;
  del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del_marker);
  route_marker_pub_->publish(marker_array);

  marker_array.markers.clear();
  const double proximity = mCopterAutopilotComponent->getWaypointProximityXY();
  const auto & waypointList = mCopterAutopilotComponent->getWaypointList();
  int marker_id = 0;
  for (int i = 0; i < waypointList.size(); ++i) {
    const PosPoint & wp = waypointList.at(i);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = marker_ns;
    marker.id = marker_id++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = wp.getX();
    marker.pose.position.y = wp.getY();
    marker.pose.position.z = wp.getHeight();
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, wp.getYaw() * M_PI / 180.0);
    marker.pose.orientation = tf2::toMsg(orientation);

    const double base_scale = proximity;
    if (i == 0) {
      // Orient the start arrow toward the next waypoint, matching control tower behaviour.
      if (waypointList.size() > 1) {
        const double dx = waypointList.at(1).getX() - wp.getX();
        const double dy = waypointList.at(1).getY() - wp.getY();
        orientation.setRPY(0.0, 0.0, std::atan2(dy, dx));
        marker.pose.orientation = tf2::toMsg(orientation);
      }
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.55f;
      marker.scale.x = base_scale * 1.5;
      marker.scale.y = base_scale * 0.5;
      marker.scale.z = base_scale * 0.5;
    } else if (i == waypointList.size() - 1) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.55f;
      marker.scale.x = base_scale * 0.75;
      marker.scale.y = base_scale * 0.75;
      marker.scale.z = base_scale * 0.75;
    } else {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.scale.x = base_scale * 0.5;
      marker.scale.y = base_scale * 0.5;
      marker.scale.z = base_scale * 0.5;
    }
    marker_array.markers.push_back(marker);
  }

  route_marker_pub_->publish(marker_array);
}

void WaywiserCopter::publish_autopilot_markers()
{
  if (!autopilot_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const std::string marker_ns = std::string(this->get_name()) + "/autopilot_markers";

  // Delete previous markers
  visualization_msgs::msg::Marker del_marker;
  del_marker.header.frame_id = world_frame_;
  del_marker.header.stamp = this->get_clock()->now();
  del_marker.ns = marker_ns;
  del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del_marker);
  autopilot_marker_pub_->publish(marker_array);

  if (!mission_state_is_route_active(mCopterAutopilotComponent->getCurrentMissionState()) ||
    mCopterAutopilotComponent->getAutoClimbActive())
  {
    return;
  }

  marker_array.markers.clear();
  const double proximity = mCopterAutopilotComponent->getWaypointProximityXY();
  const double approach_radius = mCopterAutopilotComponent->getApproachSlowdownRadius();
  const PosPoint currentPos = mCopterState->getPosition(PosType::fused);
  int marker_id = 0;

  // Proximity acceptance circle drawn at the vehicle's current altitude
  visualization_msgs::msg::Marker circle_marker;
  circle_marker.header.frame_id = world_frame_;
  circle_marker.header.stamp = this->get_clock()->now();
  circle_marker.ns = marker_ns;
  circle_marker.id = marker_id++;
  circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle_marker.action = visualization_msgs::msg::Marker::ADD;
  circle_marker.pose.orientation.w = 1.0;
  // line thickness scaled to the approach radius for consistent visualization
  circle_marker.scale.x = std::max(0.01, approach_radius * 0.03);
  circle_marker.color.r = 0.0;
  circle_marker.color.g = 0.0;
  circle_marker.color.b = 1.0;
  circle_marker.color.a = 1.0;
  constexpr int kNumPoints = 36;
  for (int i = 0; i <= kNumPoints; ++i) {
    const double angle = 2.0 * M_PI * i / kNumPoints;
    geometry_msgs::msg::Point p;
    // draw the circle at the approach slowdown radius so the autopilot target
    // (which is placed on that circle) appears on the circumference
    p.x = currentPos.getX() + approach_radius * std::cos(angle);
    p.y = currentPos.getY() + approach_radius * std::sin(angle);
    p.z = currentPos.getHeight();
    circle_marker.points.push_back(p);
  }
  marker_array.markers.push_back(circle_marker);

  // Target waypoint sphere — position from current 3D goal, small dot matching the car node style
  const PosPoint currentGoal = mCopterAutopilotComponent->getCurrentGoal();
  const QPointF targetXY = mCopterState->getAutopilotTargetPoint();
  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = world_frame_;
  target_marker.header.stamp = this->get_clock()->now();
  target_marker.ns = marker_ns;
  target_marker.id = marker_id++;
  target_marker.type = visualization_msgs::msg::Marker::SPHERE;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.position.x = targetXY.x();
  target_marker.pose.position.y = targetXY.y();
  target_marker.pose.position.z = currentGoal.getHeight();
  target_marker.pose.orientation.w = 1.0;
  const double dot_size = proximity * 0.2;
  target_marker.scale.x = dot_size;
  target_marker.scale.y = dot_size;
  target_marker.scale.z = dot_size;
  target_marker.color.r = 1.0f;
  target_marker.color.g = 0.0f;
  target_marker.color.b = 0.0f;
  target_marker.color.a = 1.0f;
  marker_array.markers.push_back(target_marker);

  autopilot_marker_pub_->publish(marker_array);
}

void WaywiserCopter::publish_home_pose()
{
  if (!home_pose_pub_ || !home_position_initialized_) {
    return;
  }

  geometry_msgs::msg::PoseStamped home_pose;
  home_pose.header.frame_id = world_frame_;
  home_pose.header.stamp = this->get_clock()->now();
  home_pose.pose.orientation.w = 1.0;
  home_pose.pose.position.x = home_position_x_;
  home_pose.pose.position.y = home_position_y_;
  home_pose.pose.position.z = 0.0;
  home_pose_pub_->publish(home_pose);
}

void WaywiserCopter::handle_return_home_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data && !manual_return_home_active_) {
    clear_return_home_failsafe();
    control_tower_heartbeat_timeout_emergency_stop_active_ = false;
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    const bool mission_active_before_rth =
      enable_autopilot_component_
      ? (mCopterAutopilotComponent &&
      mission_state_is_route_active(mCopterAutopilotComponent->getCurrentMissionState()))
      : received_active_mission_status_;
    manual_return_home_active_ = true;
    ignore_non_rth_mission_status_until_idle_ = true;
    received_active_mission_status_ = false;
    return_home_completed_ = false;
    return_home_disarm_requested_ = false;
    has_last_return_home_disarm_request_time_ = false;
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    if (auto_landing_active_) {
      auto_landing_active_ = false;
      auto_landing_disarm_requested_ = false;
      RCLCPP_WARN(get_logger(), "Auto landing cancelled because manual return-to-home is active.");
    }
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    waiting_for_heartbeat_mission_active_ = mission_active_before_rth;
    RCLCPP_WARN(get_logger(), "Manual return to home REQUESTED via service");
    publish_quadcopter_state();
  } else if (!request->data && manual_return_home_active_) {
    manual_return_home_active_ = false;
    ignore_non_rth_mission_status_until_idle_ = false;
    return_home_request_sent_ = false;
    return_home_land_request_sent_ = false;
    return_home_landing_phase_active_ = false;
    RCLCPP_WARN(get_logger(), "Manual return to home CANCELLED via service");
  }
  response->success = true;
}

void WaywiserCopter::handle_auto_land_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data && !feature_auto_land_enabled_) {
    auto_landing_active_ = false;
    auto_landing_disarm_requested_ = false;
    response->success = false;
    response->message = "Auto landing is disabled by feature_auto_land_enabled.";
    RCLCPP_WARN(get_logger(), "Ignoring auto landing request: feature_auto_land_enabled is false.");
    return;
  }

  auto_landing_active_ = request->data;
  auto_landing_disarm_requested_ = false;
  inhibit_auto_arm_after_auto_land_ = request->data;
  if (auto_landing_active_) {
    received_active_mission_status_ = false;
    ignore_non_rth_mission_status_until_idle_ = false;
    if (mCopterAutopilotComponent) {
      mCopterAutopilotComponent->clearWaypointFollowerRoute();
    }
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    clear_return_home_failsafe();
    manual_return_home_active_ = false;
    return_home_completed_ = false;
    return_home_disarm_requested_ = false;
    has_last_return_home_disarm_request_time_ = false;
    retry_offboard_for_mission_after_rth_land_ = false;
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    if (mission_status_pub_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    }
    RCLCPP_INFO(get_logger(), "Auto landing ENABLED via service");
  } else {
    // Snap hover hold to current position so the drone holds immediately
    // instead of continuing to fall after landing is cancelled.
    if (has_vehicle_local_position_) {
      hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
      has_hold_position_ = true;
    }
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();
    last_input_command_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "Auto landing DISABLED via service");
  }
  publish_quadcopter_state();
  response->success = true;
}

void WaywiserCopter::handle_arm_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const bool accepted = request_arm_state(request->data);
  if (request->data) {
    RCLCPP_INFO(get_logger(), "Arm REQUESTED via service");
  } else {
    RCLCPP_INFO(get_logger(), "Disarm REQUESTED via service");
  }
  response->success = accepted;
  if (!accepted) {
    response->message = request->data ?
      "Arm request rejected by current vehicle state." :
      "Disarm request rejected by current vehicle state.";
  }
}

void WaywiserCopter::handle_auto_climb_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!mCopterAutopilotComponent) {
    response->success = false;
    response->message = "Auto climb is unavailable because the autopilot component is not initialized.";
    return;
  }

  if (request->data && !mCopterAutopilotComponent->getAutoClimbEnabled()) {
    response->success = false;
    response->message = "Auto climb is disabled by feature_auto_climb_enabled.";
    RCLCPP_WARN(get_logger(), "Ignoring auto climb request: feature_auto_climb_enabled is false.");
    return;
  }

  if (request->data) {
    inhibit_auto_arm_after_auto_land_ = false;
    const bool auto_climb_active = mCopterAutopilotComponent->getAutoClimbActive();
    const bool route_climb_active = mCopterAutopilotComponent->getRouteClimbActive();
    if (auto_climb_active || route_climb_active) {
      const char * active_climb_message = auto_climb_active ?
        "Vehicle is already auto climbing." :
        "Vehicle is already climbing on an active route.";
      RCLCPP_INFO(get_logger(), "Ignoring auto climb request: %s", active_climb_message);
      publish_quadcopter_state();
      response->success = auto_climb_active;
      response->message = active_climb_message;
      return;
    }

    const PosPoint odom_position = mCopterState->getPosition(PosType::odom);
    const double current_height = odom_position.getHeight();
    const double auto_climb_height = mCopterAutopilotComponent->getAutoClimbHeight();
    // If the drone is at or above the climb target, do not start a climb route
    // (even from landing) — the follower would navigate downward to reach the target.
    // Instead the landing is cancelled and hover hold activates at the current position.
    const bool already_at_or_above_climb_height = current_height >= auto_climb_height;

    mCopterAutopilotComponent->clearWaypointFollowerRoute();
    if (mRthAutopilotComponent) {
      mRthAutopilotComponent->clearWaypointFollowerRoute();
    }
    clear_return_home_failsafe();
    manual_return_home_active_ = false;
    return_home_completed_ = false;
    return_home_disarm_requested_ = false;
    has_last_return_home_disarm_request_time_ = false;
    retry_offboard_for_mission_after_rth_land_ = false;
    // Clear any active landing so it no longer overrides the climb command output
    if (auto_landing_active_) {
      RCLCPP_INFO(get_logger(), "Auto landing cancelled by climb request.");
    }
    auto_landing_active_ = false;
    auto_landing_disarm_requested_ = false;
    last_arm_request_value_ = false;
    // Snap the hover hold to the current position so that if the climb route
    // hasn't produced output yet, the drone holds rather than falling
    if (has_vehicle_local_position_) {
      hold_position_ned_ = {local_position_x_, local_position_y_, local_position_z_};
      has_hold_position_ = true;
    }
    current_cmd_vel_in_ = geometry_msgs::msg::Twist();
    current_cmd_vel_out_ = geometry_msgs::msg::Twist();

    if (mission_status_pub_) {
      waywiser_core::msg::MissionState missionStateMsg;
      missionStateMsg.state = static_cast<uint8_t>(MissionState::Idle);
      mission_status_pub_->publish(missionStateMsg);
    }

    if (already_at_or_above_climb_height) {
      mCopterAutopilotComponent->setAutoClimbActive(false);
      last_input_command_time_ = -1.0;
      RCLCPP_INFO(
        get_logger(),
        "Ignoring auto climb request: current height %.2f m is already at or above configured climb height %.2f m. No climb started.",
        current_height, auto_climb_height);
      publish_quadcopter_state();
      response->success = true;
      response->message = "Vehicle is already at or above auto climb height. Climb not started, but request accepted.";
      return;
    }

  }

  const bool auto_climb_request_applied =
    mCopterAutopilotComponent->setAutoClimbActive(request->data);
  if (request->data && auto_climb_request_applied) {
    auto_landing_active_ = false;
    auto_landing_disarm_requested_ = false;
  }
  if (mCopterAutopilotComponent->getAutoClimbActive()) {
    RCLCPP_INFO(get_logger(), "Auto climb ENABLED via service");
  } else {
    const char * log_message = request->data && !auto_climb_request_applied ?
      "Auto climb request was not accepted by the current vehicle state." :
      "Auto climb DISABLED via service";
    if (request->data && !auto_climb_request_applied) {
      RCLCPP_WARN(get_logger(), "%s", log_message);
    } else {
      RCLCPP_INFO(get_logger(), "%s", log_message);
    }
  }
  publish_quadcopter_state();
  response->success = auto_climb_request_applied;
  if (!auto_climb_request_applied) {
    response->message = "Auto climb request was not accepted by the current vehicle state.";
  }
}
