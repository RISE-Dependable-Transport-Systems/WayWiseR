#include <QDateTime>
#include <QTime>

#include "waywiser_core_utils.hpp"

std::string CoreUtils::waywiseObjectTypeToString(WAYWISE_OBJECT_TYPE object_type)
{
  switch (object_type) {
    case WAYWISE_OBJECT_TYPE_CAR:
      return "car";
    case WAYWISE_OBJECT_TYPE_TRUCK:
      return "truck";
    case WAYWISE_OBJECT_TYPE_TRAILER:
      return "trailer";
    case WAYWISE_OBJECT_TYPE_QUADCOPTER:
      return "quadcopter";
    case WAYWISE_OBJECT_TYPE_GENERIC:
    default:
      return "generic";
  }
}

std::string CoreUtils::declare_read_only_waywise_object_type_param(
  rclcpp::Node * node, const QSharedPointer<ObjectState> & object_state,
  const std::string & param_name)
{
  const auto object_type = object_state ? object_state->getWaywiseObjectType() :
    WAYWISE_OBJECT_TYPE_GENERIC;
  const auto object_type_string = waywiseObjectTypeToString(object_type);

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "WayWise vehicle/object type derived from the node's ObjectState. "
    "This parameter is read-only and ignores startup overrides.";
  descriptor.read_only = true;

  return node->declare_parameter<std::string>(
    param_name, object_type_string, descriptor, true);
}

waywiser_core::msg::CarControlCommand CarControlCommand::to_msg() const
{
  waywiser_core::msg::CarControlCommand car_control_command_msg;
  car_control_command_msg.throttle = throttle;
  car_control_command_msg.brake = brake;
  car_control_command_msg.steering = steering;
  return car_control_command_msg;
}

double PIDController::compute(double error, double dt)
{
  integral_ += error * dt;
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;
  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  integral_ = 0.0;
}

void CoreUtils::update_pospoint_from_pose(
  QSharedPointer<ObjectState> objectState,
  const xyz_t pose_frame_to_reference_frame_offset,
  const geometry_msgs::msg::Pose pose,
  const PosType posType)
{
  PosPoint posPoint = objectState->getPosition(posType);
  posPoint.setX(pose.position.x);
  posPoint.setY(pose.position.y);
  posPoint.setHeight(pose.position.z);
  posPoint.updateWithOffsetAndYawRotation(
    -pose_frame_to_reference_frame_offset, tf2::getYaw(pose.orientation));
  posPoint.setTime(
    QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
  objectState->setPosition(posPoint);
}

std::string CoreUtils::missionStateToString(MissionState state)
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

MissionState CoreUtils::convertToMissionState(WayPointFollowerSTMstates state)
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

WayPointFollowerSTMstates CoreUtils::convertToWayPointFollowerSTMstates(MissionState state)
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

ImuVariant CoreUtils::get_imu_variant_param(
  rclcpp::Node * node,
  const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "vesc") {
    return ImuVariant::VESC;
  } else if (str == "bno055") {
    return ImuVariant::BNO055;
  } else if (str == "waywise_simulated") {
    return ImuVariant::WAYWISE_SIMULATED;
  }
  return ImuVariant::UNKNOWN;
}

VehicleInterfaceType CoreUtils::get_vehicle_interface_type_param(
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

RECEIVER_VARIANT CoreUtils::get_receiver_variant_param(
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

SpeedControlType CoreUtils::get_speed_control_type_param(
  rclcpp::Node * node, const std::string & param_name)
{
  auto str = node->declare_parameter(param_name, "");
  boost::to_lower(str);

  if (str == "closed_loop_pid_speed_control") {
    return SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL;
  }
  return SpeedControlType::OPEN_LOOP_ERPM_CONTROL;
}

void CoreUtils::qtMessageToLogger(
  const rclcpp::Logger & logger, QtMsgType type, const QString & msg)
{
  std::string text = msg.toStdString();
  switch (type) {
    case QtCriticalMsg:
      RCLCPP_ERROR(logger, "%s", text.c_str());
      break;
    case QtWarningMsg:
      RCLCPP_WARN(logger, "%s", text.c_str());
      break;
    case QtInfoMsg:
      RCLCPP_INFO(logger, "%s", text.c_str());
      break;
    case QtDebugMsg:
      RCLCPP_DEBUG(logger, "%s", text.c_str());
      break;
    default:
      break;
  }
}

QSharedPointer<urdf::Model> URDFUtils::getURDFModel(const std::string & urdf_file_)
{
  QSharedPointer<urdf::Model> urdfModel = nullptr;

  if (!urdf_file_.empty()) {
    urdfModel = QSharedPointer<urdf::Model>::create();
    if (urdf_file_.find("xml version=") != std::string::npos) {
      urdfModel->initString(urdf_file_);
    } else {
      urdfModel->initFile(urdf_file_);
    }
  }

  return urdfModel;
}

vector3_t URDFUtils::getFramePosition(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & link_name, const bool logWarning)
{
  std::string normalized_link_name = link_name;
  if (!normalized_link_name.empty() && normalized_link_name.front() == '/') {
    normalized_link_name.erase(0, 1);
  }

  urdf::Vector3 position(0, 0, 0);
  urdf::LinkConstSharedPtr link = urdfModel->getLink(normalized_link_name);
  if (!link) {
    if (urdfModel->getRoot()->name != normalized_link_name && logWarning) {
      qWarning() << "Link not found:" << QString::fromStdString(normalized_link_name) <<
        " in model:" <<
        QString::fromStdString(urdfModel->getName());
    }
  } else {
    while (link && link->parent_joint) {
      const urdf::Pose & joint_pose = link->parent_joint->parent_to_joint_origin_transform;
      position = joint_pose.rotation * position + joint_pose.position;
      link = urdfModel->getLink(link->getParent()->name);
    }
  }
  return vector3_t{position.x, position.y, position.z};
}

vector3_t URDFUtils::getFramePositionOffset(
  QSharedPointer<urdf::Model> urdfModel, const std::string & frame_A,
  const std::string & frame_B, const bool logWarning)
{
  return getFramePosition(urdfModel, frame_A, logWarning) -
         getFramePosition(urdfModel, frame_B, logWarning);
}

urdf::Rotation URDFUtils::getFrameRotation(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & link_name, const bool logWarning)
{
  std::string normalized_link_name = link_name;
  if (!normalized_link_name.empty() && normalized_link_name.front() == '/') {
    normalized_link_name.erase(0, 1);
  }

  urdf::Rotation rotation(0, 0, 0, 1);  // identity quaternion
  urdf::LinkConstSharedPtr link = urdfModel->getLink(normalized_link_name);
  if (!link && logWarning) {
    qWarning() << "Link not found:" << QString::fromStdString(normalized_link_name);
  } else {
    while (link && link->parent_joint) {
      const urdf::Pose & joint_pose = link->parent_joint->parent_to_joint_origin_transform;
      rotation = joint_pose.rotation * rotation;
      link = urdfModel->getLink(link->getParent()->name);
    }
  }
  return rotation;
}

vector3_t URDFUtils::getFrameRotationOffset(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & frame_A,
  const std::string & frame_B, const bool logWarning)
{
  urdf::Rotation rot_A = getFrameRotation(urdfModel, frame_A, logWarning);
  urdf::Rotation rot_B = getFrameRotation(urdfModel, frame_B, logWarning);

  urdf::Rotation rot_offset = rot_B.GetInverse() * rot_A;

  double roll, pitch, yaw;
  rot_offset.getRPY(roll, pitch, yaw);

  return vector3_t{roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG};
}
