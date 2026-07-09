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
  cancelAutoClimb();
  mWaypointList.clear();
  updateMissionState(MissionState::WaitingForVehicleInit);

  if (mAutopilotMovementController) {
    mAutopilotMovementController->setDesiredSpeed(0.0);
    mAutopilotMovementController->setDesiredSteering(0.0);
  }
}

void CopterAutopilotComponent::setAutoClimbEnabled(bool value)
{
  mAutoClimbEnabled = value;
  if (!mAutoClimbEnabled) {
    if (mAutoClimbActive) {
      stopWaypointFollower();
    }
    cancelAutoClimb();
  }
}

bool CopterAutopilotComponent::setAutoClimbActive(bool value)
{
  if (!value) {
    if (mAutoClimbActive) {
      stopWaypointFollower();
    }
    cancelAutoClimb();
    return true;
  }

  if (!mAutoClimbEnabled) {
    return false;
  }

  return startAutoClimbWithWaypointFollower();
}

void CopterAutopilotComponent::cancelAutoClimb()
{
  mAutoClimbActive = false;
  applyMissionWaypointFollowerTuning();
}

bool CopterAutopilotComponent::startAutoClimbWithWaypointFollower()
{
  if (!mWaypointFollower) {
    return false;
  }
  if (mEmergencyStopState && mEmergencyStopState->is_active()) {
    return false;
  }
  if (currentMissionState == MissionState::WaitingForVehicleInit) {
    updateMissionState(MissionState::Idle);
  }

  cancelAutoClimb();

  if (!mWaypointList.isEmpty()) {
    mWaypointList.clear();
  }

  PosPoint autoClimbGoal = mCopterState->getPosition(mMissionPosTypeUsed);
  autoClimbGoal.setHeight(mAutoClimbHeight);
  QList<PosPoint> autoClimbRoute;
  autoClimbRoute.append(autoClimbGoal);
  mWaypointFollower->clearRoute();
  mWaypointFollower->resetState();
  mWaypointFollower->setFinishRouteAfterInitialClimb(true);
  mWaypointFollower->addRoute(autoClimbRoute);
  mWaypointFollower->startFollowingRoute(false);
  mAutoClimbActive = true;

  updateMissionState(deriveWaypointFollowerMissionState());
  applyAutoClimbWaypointFollowerTuning();
  return true;
}

void CopterAutopilotComponent::applyMissionWaypointFollowerTuning()
{
  if (!mWaypointFollower) {
    return;
  }
  mWaypointFollower->setWaypointProximityXY(mWaypointProximityXY);
  mWaypointFollower->setWaypointProximityZ(mWaypointProximityZ);
  mWaypointFollower->setEndGoalAlignmentThresholdXY(mEndGoalAlignmentThresholdXY);
  mWaypointFollower->setEndGoalAlignmentThresholdZ(mEndGoalAlignmentThresholdZ);
  mWaypointFollower->setStopSpeedThreshold(mStopSpeedThreshold);
  mWaypointFollower->setVerticalHeightTolerance(mVerticalHeightTolerance);
}

void CopterAutopilotComponent::applyAutoClimbWaypointFollowerTuning()
{
  if (!mWaypointFollower) {
    return;
  }
  mWaypointFollower->setWaypointProximityXY(mWaypointProximityXY);
  mWaypointFollower->setWaypointProximityZ(mWaypointProximityZ);
  mWaypointFollower->setEndGoalAlignmentThresholdXY(mEndGoalAlignmentThresholdXY);
  mWaypointFollower->setEndGoalAlignmentThresholdZ(mEndGoalAlignmentThresholdZ);
  mWaypointFollower->setStopSpeedThreshold(mStopSpeedThreshold);
  mWaypointFollower->setVerticalHeightTolerance(mWaypointProximityZ);
}

void CopterAutopilotComponent::setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState)
{
  mEmergencyStopState = emergencyStopState;

  mAutopilotMovementController.reset(new MovementController(mCopterState));

  mWaypointFollower.reset(new CopterWaypointFollower(mAutopilotMovementController, mMissionPosTypeUsed));
  mWaypointFollower->setRepeatRoute(false);
  applyMissionWaypointFollowerTuning();
  mWaypointFollower->setCruiseSpeed(mCruiseSpeed);
  mWaypointFollower->setMaxSpeed(mMaxMissionSpeed);
  mWaypointFollower->setDescentSpeed(mDescentSpeed);
  mWaypointFollower->setMinApproachSpeed(mMinApproachSpeed);
  mWaypointFollower->setApproachSlowdownRadius(mApproachSlowdownRadius);
  mWaypointFollower->setFaceTravelDirection(mFaceTravelDirection);
  mWaypointFollower->setYawGain(mYawGain);
  mWaypointFollower->setMaxYawRate(mMaxYawRate);
  mWaypointFollower->setVerticalProportionalGain(mVerticalProportionalGain);
  mWaypointFollower->setVerticalIntegralGain(mVerticalIntegralGain);
  mWaypointFollower->setVerticalDerivativeGain(mVerticalDerivativeGain);
  mWaypointFollower->setVerticalIntegralLimit(mVerticalIntegralLimit);

  // -- MAVLINK communication towards ControlTowerNode --
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
        if (mAutoClimbActive) {
          updateMissionState(deriveWaypointFollowerMissionState());
        } else if (mWaypointFollower->getCurrentRoute().size() > 0) {
          mWaypointList = mWaypointFollower->getCurrentRoute();
          updateMissionState(deriveWaypointFollowerMissionState());
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
          updateMissionState(deriveWaypointFollowerMissionState());
        }
      } else if (mWaypointFollower->getCurrentRoute().size() > 0) {
        if (!mAutoClimbActive) {
          mWaypointList = mWaypointFollower->getCurrentRoute();
          updateMissionState(deriveWaypointFollowerMissionState());
        }
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
          updateMissionState(deriveWaypointFollowerMissionState());
        }
      } else if (mWaypointFollower->isActive()) {
        stopWaypointFollower();
        emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
      }
      break;

    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteClimb:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
    case MissionState::FollowRouteApproachingEndGoalZ:
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
        const MissionState waypointFollowerMissionState = deriveWaypointFollowerMissionState();
        if (currentMissionState != waypointFollowerMissionState) {
          updateMissionState(waypointFollowerMissionState);
        }

        if (wayPointFollowerSTMstate == WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED ||
          ((currentMissionState == MissionState::FollowRouteApproachingEndGoal ||
            currentMissionState == MissionState::FollowRouteApproachingEndGoalZ) &&
          !mWaypointFollower->isActive()))
        {
          updateMissionState(MissionState::FollowRouteFinished);
        } else if (!mWaypointFollower->isActive()) {
          if (mWaypointList.isEmpty()) {
            updateMissionState(MissionState::WaitingForRoute);
          } else {
            startWaypointFollower(mWaypointList);
            updateMissionState(MissionState::FollowRouteInit);
          }
        }
      }
      break;

    case MissionState::FollowRouteFinished:
      if (mAutoClimbActive) {
        cancelAutoClimb();
        stopWaypointFollower();
        updateMissionState(mWaypointList.isEmpty() ? MissionState::Idle : MissionState::WaitingForRoute);
      } else {
        if (mWaypointFollower && mWaypointFollower->isActive()) {
          mWaypointFollower->stop();
        }
        if (mAutopilotMovementController) {
          mAutopilotMovementController->setDesiredSpeed(0.0);
          mAutopilotMovementController->setDesiredSteering(0.0);
        }
      }
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

void CopterAutopilotComponent::startWaypointFollowerRouteFromBeginning(
  QList<PosPoint> & waypointList)
{
  cancelAutoClimb();
  stopWaypointFollower();
  mWaypointFollower->clearRoute();
  mWaypointFollower->resetState();
  mWaypointFollower->addRoute(waypointList);
  mWaypointFollower->startFollowingRoute(true);
  mWaypointList = mWaypointFollower->getCurrentRoute();
  updateMissionState(MissionState::FollowRouteInit);
  qDebug() << "Started copter waypoint follower from beginning with a route of " <<
    waypointList.size() << " waypoints";
}

void CopterAutopilotComponent::stopWaypointFollower()
{
  if (mWaypointFollower) {
    mWaypointFollower->stop();
  }
  updateMissionState(MissionState::Idle);
  if (mAutopilotMovementController) {
    mAutopilotMovementController->setDesiredSpeed(0.0);
    mAutopilotMovementController->setDesiredSteering(0.0);
  }
  qDebug() << "Copter waypoint follower is stopped.";
}

void CopterAutopilotComponent::clearWaypointFollowerRoute()
{
  cancelAutoClimb();
  stopWaypointFollower();
  if (mWaypointFollower) {
    mWaypointFollower->clearRoute();
    mWaypointFollower->resetState();
  }
  mWaypointList.clear();
  updateMissionState(MissionState::Idle);
}

bool CopterAutopilotComponent::isActive()
{
  return mWaypointFollower && mWaypointFollower->isActive();
}

bool CopterAutopilotComponent::getRouteClimbActive() const
{
  return mWaypointFollower && mWaypointFollower->isActive() &&
    mWaypointFollower->isClimbingToStartWaypointHeight();
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
  if (mAutoClimbActive && mWaypointFollower && mWaypointFollower->isActive()) {
    cancelAutoClimb();
    stopWaypointFollower();
    startWaypointFollower(waypointList);
    mWaypointList = mWaypointFollower->getCurrentRoute();
    return;
  }

  switch (currentMissionState) {
    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteClimb:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
    case MissionState::FollowRouteApproachingEndGoalZ:
      cancelAutoClimb();
      stopWaypointFollower();
      startWaypointFollower(waypointList);
      mWaypointList = mWaypointFollower->getCurrentRoute();
      break;
    default:
      cancelAutoClimb();
      mWaypointList = waypointList;
      break;
  }
}

MissionState CopterAutopilotComponent::deriveWaypointFollowerMissionState() const
{
  if (!mWaypointFollower) {
    return MissionState::Idle;
  }

  if (mWaypointFollower->isClimbingToStartWaypointHeight()) {
    return MissionState::FollowRouteClimb;
  }

  return CoreUtils::convertToMissionState(mWaypointFollower->getCurrentState().stmState);
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
  if (currentMissionState == state) {
    return;
  }
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
