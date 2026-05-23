#include "copter_autopilot_component.hpp"
#include "moc_copter_autopilot_component.cpp"

#include <algorithm>

#include "WayWise/core/pospoint.h"

void CopterAutopilotComponent::reset()
{
  if (mWaypointFollower) {
    mWaypointFollower->clearRoute();
    mWaypointFollower->resetState();
  }
  mWaypointList.clear();
  updateMissionState(MissionState::WaitingForVehicleInit);

  if (mAutopilotMovementController) {
    mAutopilotMovementController->setDesiredSpeed(0.0);
    mAutopilotMovementController->setDesiredSteering(0.0);
  }
}

void CopterAutopilotComponent::setAutoLiftOffEnabled(bool value)
{
  mAutoLiftOffEnabled = value;
  if (!mAutoLiftOffEnabled) {
    cancelAutoLiftOff();
  }
}

void CopterAutopilotComponent::setAutoLiftOffActive(bool value)
{
  if (value && !mAutoLiftOffEnabled) {
    return;
  }
  mAutoLiftOffActive = value;
}

void CopterAutopilotComponent::cancelAutoLiftOff()
{
  mAutoLiftOffActive = false;
}

bool CopterAutopilotComponent::updateAutoLiftOffCommand(
  geometry_msgs::msg::Twist & output, bool armed, bool inFlight)
{
  if (!mAutoLiftOffActive) {
    return false;
  }

  (void)armed;

  if (!mAutoLiftOffEnabled) {
    cancelAutoLiftOff();
    return false;
  }

  const PosPoint odomPosition = mCopterState->getPosition(PosType::odom);
  if (odomPosition.getHeight() >= mAutoLiftOffHeight - mAutoLiftOffTolerance) {
    cancelAutoLiftOff();
    output = geometry_msgs::msg::Twist();
    return true;
  }

  output = geometry_msgs::msg::Twist();
  output.linear.z = std::max(mAutoLiftOffSpeed, 0.0);

  if (!inFlight && output.linear.z <= 0.0) {
    cancelAutoLiftOff();
    return false;
  }

  return true;
}

void CopterAutopilotComponent::setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState)
{
  mEmergencyStopState = emergencyStopState;

  mAutopilotMovementController.reset(new MovementController(mCopterState));

  mWaypointFollower.reset(new CopterWaypointFollower(mAutopilotMovementController, mMissionPosTypeUsed));
  mWaypointFollower->setRepeatRoute(false);
  mWaypointFollower->setWaypointProximity(mWaypointProximity);
  mWaypointFollower->setEndGoalAlignmentThreshold(mEndGoalAlignmentThreshold);
  mWaypointFollower->setCruiseSpeed(mCruiseSpeed);
  mWaypointFollower->setMaxSpeed(mMaxMissionSpeed);
  mWaypointFollower->setMinApproachSpeed(mMinApproachSpeed);
  mWaypointFollower->setApproachSlowdownRadius(mApproachSlowdownRadius);
  mWaypointFollower->setFaceTravelDirection(mFaceTravelDirection);
  mWaypointFollower->setYawGain(mYawGain);
  mWaypointFollower->setMaxYawRate(mMaxYawRate);

  // -- MAVLINK communication towards ControlTower --
  if (mEnableMavlinkInterface) {
    mMavsdkVehicleServer.reset(
      new MavsdkVehicleServer(
        mCopterState,
        QHostAddress(QString::fromStdString(mWaywiseControlTowerAddress)),
        mWaywiseControlTowerPort));
    mMavsdkVehicleServer->setMovementController(mAutopilotMovementController);
    mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);
    mMavsdkVehicleServer->setTransferLogs(false);
  }
}

void CopterAutopilotComponent::provideParametersToParameterServer()
{
  if (mCopterState) {
    mCopterState->provideParametersToParameterServer();
  }
  if (mMavsdkVehicleServer) {
    mMavsdkVehicleServer->provideParametersToParameterServer();
  }
}

void CopterAutopilotComponent::processMissionStateMachine()
{
  const bool vehicleInitialized =
    mVehicleInitialized ||
    (!mRequireGnssForMission &&
    !mCopterState->getPosition(mMissionPosTypeUsed).getTime().isNull());

  if (!vehicleInitialized) {
    if (currentMissionState != MissionState::WaitingForVehicleInit) {
      updateMissionState(MissionState::WaitingForVehicleInit);
    }
    return;
  }

  if (mRequireGnssForMission) {
    switch (mGnssFixStatus.fixType) {
      case GNSS_FIX_TYPE::NO_FIX:
      case GNSS_FIX_TYPE::TIME_ONLY_FIX:
        if (currentMissionState != MissionState::WaitingForVehicleInit) {
          updateMissionState(MissionState::WaitingForVehicleInit);
        }
        return;
      default:
        break;
    }
  }

  switch (currentMissionState) {
    case MissionState::WaitingForVehicleInit:
      updateMissionState(MissionState::Idle);
      break;

    case MissionState::Idle:
      if (mWaypointFollower->isActive()) {
        if (mWaypointFollower->getCurrentRoute().size() > 0) {
          mWaypointList = mWaypointFollower->getCurrentRoute();
          updateMissionState(CoreUtils::convertToMissionState(
              mWaypointFollower->getCurrentState().stmState));
        } else {
          updateMissionState(MissionState::WaitingForRoute);
        }
      }
      break;

    case MissionState::WaitingForRoute:
      if (!mWaypointList.isEmpty()) {
        if (mEmergencyStopState->is_active()) {
          updateMissionState(MissionState::WaitingForEmergencyStopClear);
        } else if (mRequireGnssForMission && !assertGnssFixAccuracy()) {
          updateMissionState(MissionState::WaitingForGnssAccuracy);
          emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
        } else {
          startWaypointFollower(mWaypointList);
          updateMissionState(MissionState::FollowRouteInit);
        }
      } else if (mWaypointFollower->getCurrentRoute().size() > 0) {
        mWaypointList = mWaypointFollower->getCurrentRoute();
        updateMissionState(CoreUtils::convertToMissionState(
            mWaypointFollower->getCurrentState().stmState));
      }
      break;

    case MissionState::WaitingForEmergencyStopClear:
      if (mEmergencyStopState->is_clear()) {
        updateMissionState(MissionState::Idle);
      }
      break;

    case MissionState::WaitingForGnssAccuracy:
      if (!mRequireGnssForMission || assertGnssFixAccuracy()) {
        if (mWaypointList.isEmpty()) {
          updateMissionState(MissionState::WaitingForRoute);
        } else if (mEmergencyStopState->is_active()) {
          updateMissionState(MissionState::WaitingForEmergencyStopClear);
        } else {
          startWaypointFollower(mWaypointList);
          updateMissionState(MissionState::FollowRouteInit);
        }
      } else if (mWaypointFollower->isActive()) {
        stopWaypointFollower();
        emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
      }
      break;

    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
      if (mEmergencyStopState->is_active()) {
        stopWaypointFollower();
        updateMissionState(MissionState::WaitingForEmergencyStopClear);
      } else if (mRequireGnssForMission && !assertGnssFixAccuracy()) {
        stopWaypointFollower();
        updateMissionState(MissionState::WaitingForGnssAccuracy);
        emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
      } else {
        WayPointFollowerSTMstates wayPointFollowerSTMstate =
          mWaypointFollower->getCurrentState().stmState;
        if (CoreUtils::convertToWayPointFollowerSTMstates(currentMissionState) !=
          wayPointFollowerSTMstate)
        {
          updateMissionState(CoreUtils::convertToMissionState(wayPointFollowerSTMstate));
        }

        if (wayPointFollowerSTMstate == WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED ||
          (currentMissionState == MissionState::FollowRouteApproachingEndGoal &&
          !mWaypointFollower->isActive()))
        {
          updateMissionState(MissionState::FollowRouteFinished);
        }
      }
      break;

    case MissionState::FollowRouteFinished:
      stopWaypointFollower();
      break;

    default:
      break;
  }
}

void CopterAutopilotComponent::switchAutopilot(bool enable)
{
  if (enable) {
    if (currentMissionState == MissionState::Idle) {
      updateMissionState(MissionState::WaitingForRoute);
    } else if (currentMissionState == MissionState::WaitingForVehicleInit) {
      qDebug() << "Waiting for vehicle init, ignoring autopilot start request.";
    }
  } else if (currentMissionState != MissionState::Idle) {
    stopWaypointFollower();
    updateMissionState(MissionState::Idle);
  }
}

void CopterAutopilotComponent::startWaypointFollower(QList<PosPoint> & waypointList)
{
  mWaypointFollower->clearRoute();
  mWaypointFollower->resetState();
  mWaypointFollower->addRoute(waypointList);
  mWaypointFollower->startFollowingRoute(false);
  qDebug() << "Started copter waypoint follower with a route of " << waypointList.size() <<
    " waypoints";
}

void CopterAutopilotComponent::stopWaypointFollower()
{
  mWaypointFollower->stop();
  updateMissionState(MissionState::Idle);
  if (mAutopilotMovementController) {
    mAutopilotMovementController->setDesiredSpeed(0.0);
    mAutopilotMovementController->setDesiredSteering(0.0);
  }
  qDebug() << "Copter waypoint follower is stopped.";
}

bool CopterAutopilotComponent::isActive()
{
  return mWaypointFollower && mWaypointFollower->isActive();
}

PosPoint CopterAutopilotComponent::getCurrentGoal() const
{
  if (mWaypointFollower && mWaypointFollower->isActive()) {
    return mWaypointFollower->getCurrentGoal();
  }
  return PosPoint{};
}

void CopterAutopilotComponent::updateWaypointFollowerRoute(QList<PosPoint> & waypointList)
{
  switch (currentMissionState) {
    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
      stopWaypointFollower();
      startWaypointFollower(waypointList);
      mWaypointList = mWaypointFollower->getCurrentRoute();
      break;
    default:
      mWaypointList = waypointList;
      break;
  }
}

geometry_msgs::msg::Twist CopterAutopilotComponent::getAutopilotTwistCommand() const
{
  geometry_msgs::msg::Twist twist;
  if (!mWaypointFollower) {
    return twist;
  }

  const CopterVelocityCommand command = mWaypointFollower->getDesiredVelocityCommand();
  twist.linear.x = command.forward;
  twist.linear.y = -command.left;  // left→right sign: trajectory setpoint uses NED body-frame y = right
  twist.linear.z = command.up;
  twist.angular.z = command.yawRate;
  return twist;
}

void CopterAutopilotComponent::updateMissionState(MissionState state)
{
  currentMissionState = state;
  qDebug() << "MissionState: " << CoreUtils::missionStateToString(currentMissionState).c_str();
  emit updatedMissionState(currentMissionState);
}

bool CopterAutopilotComponent::assertGnssFixAccuracy()
{
  if (mGnssFixStatus.horizontalAccuracy > mPositionAccuracyThresholdForMission ||
    mGnssFixStatus.verticalAccuracy > mYawAccuracyThresholdForMission)
  {
    return false;
  }

  return true;
}
