#ifndef WAYWISER_CORE_UTILS_HPP_
#define WAYWISER_CORE_UTILS_HPP_

#include <QDebug>
#include <QSharedPointer>
#include <QTime>
#include <QDateTime>
#include <cstdint>
#include <string>

#include <boost/algorithm/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "waywiser/waywiser_utils.hpp"
#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/sensors/gnss/gnssreceiver.h"
#include "WayWise/vehicles/objectstate.h"
#include "WayWise/core/pospoint.h"
#include "WayWise/core/coordinatetransforms.h"

#include "waywiser_core/msg/mission_state.hpp"
#include "waywiser_core/msg/car_control_command.hpp"
#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"


enum class VehicleInterfaceType {WAYWISE_SIMULATED, EXT_SIMULATED, VESC};
enum class ImuVariant {UNKNOWN, BNO055, VESC, WAYWISE_SIMULATED};
enum class SpeedControlType {OPEN_LOOP_ERPM_CONTROL, CLOSED_LOOP_PID_SPEED_CONTROL};

struct CarControlCommand
{
  float throttle = 0.0;
  float brake = 0.0;
  float steering = 0.0;

  waywiser_core::msg::CarControlCommand to_msg() const;
};

class PIDController
{
public:
  PIDController(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt);

  void reset();

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};

enum class MissionState : int8_t
{
  Idle                          = waywiser_core::msg::MissionState::IDLE,
  WaitingForVehicleInit         = waywiser_core::msg::MissionState::WAITING_FOR_VEHICLE_INIT,
  WaitingForRoute               = waywiser_core::msg::MissionState::WAITING_FOR_ROUTE,
  WaitingForEmergencyStopClear  =
    waywiser_core::msg::MissionState::WAITING_FOR_EMERGENCY_STOP_CLEAR,
  WaitingForGnssAccuracy        = waywiser_core::msg::MissionState::WAITING_FOR_GNSS_ACCURACY,
  WaitingForHeartbeat           = waywiser_core::msg::MissionState::WAITING_FOR_HEARTBEAT,
  FollowRouteInit               = waywiser_core::msg::MissionState::FOLLOW_ROUTE_INIT,
  FollowRouteClimb            = waywiser_core::msg::MissionState::FOLLOW_ROUTE_CLIMB,
  FollowRouteGotoBegin          = waywiser_core::msg::MissionState::FOLLOW_ROUTE_GOTO_BEGIN,
  FollowRouteFollowing          = waywiser_core::msg::MissionState::FOLLOW_ROUTE_FOLLOWING,
  FollowRouteApproachingEndGoal =
    waywiser_core::msg::MissionState::FOLLOW_ROUTE_APPROACHING_END_GOAL,
  FollowRouteApproachingEndGoalZ =
    waywiser_core::msg::MissionState::FOLLOW_ROUTE_APPROACHING_END_GOAL_Z,
  FollowRouteFinished           = waywiser_core::msg::MissionState::FOLLOW_ROUTE_FINISHED,
  ReturnHomeInit                = waywiser_core::msg::MissionState::RETURN_HOME_INIT,
  ReturnHomeClimb             = waywiser_core::msg::MissionState::RETURN_HOME_CLIMB,
  ReturnHomeCruising            = waywiser_core::msg::MissionState::RETURN_HOME_CRUISING,
  ReturnHomeLanding             = waywiser_core::msg::MissionState::RETURN_HOME_LANDING
};

class EmergencyStopState
{
public:
  void set_active() {state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;}
  void set_clear() {state = waywiser_twist_safety::msg::EmergencyStopState::CLEAR;}

  bool is_active() {return state == waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;}
  bool is_clear() {return state == waywiser_twist_safety::msg::EmergencyStopState::CLEAR;}

private:
  int8_t state = waywiser_twist_safety::msg::EmergencyStopState::UNKNOWN;
};

class CoreUtils
{
public:
  static std::string waywiseObjectTypeToString(WAYWISE_OBJECT_TYPE object_type);

  static std::string declare_read_only_waywise_object_type_param(
    rclcpp::Node * node, const QSharedPointer<ObjectState> & object_state,
    const std::string & param_name = "waywise_object_type");

  static void update_pospoint_from_pose(
    QSharedPointer<ObjectState> objectState,
    const xyz_t pose_frame_to_reference_frame_offset,
    const geometry_msgs::msg::Pose pose,
    const PosType posType);

  static std::string missionStateToString(MissionState state);

  static MissionState convertToMissionState(WayPointFollowerSTMstates state);

  static WayPointFollowerSTMstates convertToWayPointFollowerSTMstates(MissionState state);

  static ImuVariant get_imu_variant_param(
    rclcpp::Node * node,
    const std::string & param_name);

  static VehicleInterfaceType get_vehicle_interface_type_param(
    rclcpp::Node * node, const std::string & param_name);

  static RECEIVER_VARIANT get_receiver_variant_param(
    rclcpp::Node * node, const std::string & param_name);

  static SpeedControlType get_speed_control_type_param(
    rclcpp::Node * node, const std::string & param_name);

  static void qtMessageToLogger(
    const rclcpp::Logger & logger, QtMsgType type, const QString & msg);
};

class URDFUtils
{
public:
  static QSharedPointer<urdf::Model> getURDFModel(const std::string & urdf_file_);

  static vector3_t getFramePosition(
    QSharedPointer<urdf::Model> urdfModel,
    const std::string & link_name,
    const bool logWarning = true);

  static vector3_t getFramePositionOffset(
    QSharedPointer<urdf::Model> urdfModel, const std::string & frame_A,
    const std::string & frame_B, const bool logWarning = true);

  static urdf::Rotation getFrameRotation(
    QSharedPointer<urdf::Model> urdfModel,
    const std::string & link_name, const bool logWarning = true);

  static vector3_t getFrameRotationOffset(
    QSharedPointer<urdf::Model> urdfModel,
    const std::string & frame_A,
    const std::string & frame_B, const bool logWarning = true);
};


#endif  // WAYWISER_CORE_UTILS_HPP_
