#ifndef WAYWISER_CORE_UTILS_HPP_
#define WAYWISER_CORE_UTILS_HPP_

#include <optional>
#include <QDebug>
#include <limits>

#include "boost/algorithm/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "WayWise/autopilot/purepursuitwaypointfollower.h"

#include "waywiser_core/msg/mission_state.hpp"
#include "waywiser_core/msg/car_control_command.hpp"
#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"


enum class VehicleInterfaceType {WAYWISE_SIMULATED, EXT_SIMULATED, VESC};
enum class ImuVariant {UNKNOWN, BNO055, VESC};
enum class SpeedControlType {OPEN_LOOP_ERPM_CONTROL, CLOSED_LOOP_PID_SPEED_CONTROL};

struct CarControlCommand
{
  float throttle = 0.0;
  float brake = 0.0;
  float steering = 0.0;

  waywiser_core::msg::CarControlCommand to_msg() const
  {
    waywiser_core::msg::CarControlCommand car_control_command_msg;
    car_control_command_msg.throttle = throttle;
    car_control_command_msg.brake = brake;
    car_control_command_msg.steering = steering;
    return car_control_command_msg;
  }
};

class PIDController
{
public:
  PIDController(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt)
  {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

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
  FollowRouteInit               = waywiser_core::msg::MissionState::FOLLOW_ROUTE_INIT,
  FollowRouteGotoBegin          = waywiser_core::msg::MissionState::FOLLOW_ROUTE_GOTO_BEGIN,
  FollowRouteFollowing          = waywiser_core::msg::MissionState::FOLLOW_ROUTE_FOLLOWING,
  FollowRouteApproachingEndGoal =
    waywiser_core::msg::MissionState::FOLLOW_ROUTE_APPROACHING_END_GOAL,
  FollowRouteFinished           = waywiser_core::msg::MissionState::FOLLOW_ROUTE_FINISHED
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

inline std::string missionStateToString(MissionState state)
{
  switch (state) {
    case MissionState::Idle:
      return "Idle";
    case MissionState::WaitingForRoute:
      return "Waiting For Route";
    case MissionState::WaitingForVehicleInit:
      return "Waiting For Vehicle Init";
    case MissionState::WaitingForEmergencyStopClear:
      return "Waiting For Emergency Stop Clear";
    case MissionState::WaitingForGnssAccuracy:
      return "Waiting For GNSS Accuracy";
    case MissionState::FollowRouteInit:
      return "Follow Route Init";
    case MissionState::FollowRouteGotoBegin:
      return "Follow Route Goto Begin";
    case MissionState::FollowRouteFollowing:
      return "Follow Route Following";
    case MissionState::FollowRouteApproachingEndGoal:
      return "Follow Route Approaching End Goal";
    case MissionState::FollowRouteFinished:
      return "Follow Route Finished";
    default:
      return "Unknown MissionState";
  }
}

inline MissionState convertToMissionState(WayPointFollowerSTMstates state)
{
  switch (state) {
    case WayPointFollowerSTMstates::NONE:
      return MissionState::Idle;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
      return MissionState::FollowRouteInit;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN:
      return MissionState::FollowRouteGotoBegin;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING:
      return MissionState::FollowRouteFollowing;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL:
      return MissionState::FollowRouteApproachingEndGoal;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
      return MissionState::FollowRouteFinished;
    default:
      // Handle unknown conversion:
      return MissionState::Idle;
  }
}

inline WayPointFollowerSTMstates convertToWayPointFollowerSTMstates(MissionState state)
{
  switch (state) {
    case MissionState::Idle:
      return WayPointFollowerSTMstates::NONE;
    case MissionState::FollowRouteInit:
      return WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
    case MissionState::FollowRouteGotoBegin:
      return WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
    case MissionState::FollowRouteFollowing:
      return WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
    case MissionState::FollowRouteApproachingEndGoal:
      return WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
    case MissionState::FollowRouteFinished:
      return WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
    default:
      // Handle unknown conversion:
      return WayPointFollowerSTMstates::NONE;
  }
}

inline ImuVariant get_imu_variant_param(rclcpp::Node * node, const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "vesc") {
    return ImuVariant::VESC;
  } else if (str == "bno055") {
    return ImuVariant::BNO055;
  }
  return ImuVariant::UNKNOWN;
}

inline VehicleInterfaceType get_vehicle_interface_type_param(
  rclcpp::Node * node, const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "waywise_simulated") {
    return VehicleInterfaceType::WAYWISE_SIMULATED;
  } else if (str == "ext_simulated") {
    return VehicleInterfaceType::EXT_SIMULATED;
  }
  return VehicleInterfaceType::VESC;
}

inline RECEIVER_VARIANT get_receiver_variant_param(
  rclcpp::Node * node, const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "ublox_zed_f9p") {
    return RECEIVER_VARIANT::UBLX_ZED_F9P;
  } else if (str == "ublox_zed_f9r") {
    return RECEIVER_VARIANT::UBLX_ZED_F9R;
  } else if (str == "external") {
    return RECEIVER_VARIANT::EXTERNAL;
  }
  return RECEIVER_VARIANT::WAYWISE_SIMULATED;
}

inline SpeedControlType get_speed_control_type_param(
  rclcpp::Node * node, const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "closed_loop_pid_speed_control") {
    return SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL;
  }
  return SpeedControlType::OPEN_LOOP_ERPM_CONTROL;
}

#endif  // WAYWISER_CORE_UTILS_HPP_
