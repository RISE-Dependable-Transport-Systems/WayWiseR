#include "copter_autopilot_component.hpp"
#include "moc_copter_autopilot_component.cpp"

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
    if (mAutoLiftOffActive) {
      stopWaypointFollower();
    }
    cancelAutoLiftOff();
  }
}

void CopterAutopilotComponent::setAutoLiftOffActive(bool value)
{
  if (!value) {
    if (mAutoLiftOffActive) {
      stopWaypointFollower();
    }
    cancelAutoLiftOff();
    return;
  }

  if (!mAutoLiftOffEnabled) {
    return;
  }

  startAutoLiftOffWithWaypointFollower();
}

void CopterAutopilotComponent::cancelAutoLiftOff()
{
  mAutoLiftOffActive = false;
}

bool CopterAutopilotComponent::startAutoLiftOffWithWaypointFollower()
{
  if (!mWaypointFollower) {
    return false;
  }
  if (mEmergencyStopState && mEmergencyStopState->is_active()) {
    return false;
  }
  if (currentMissionState == MissionState::WaitingForVehicleInit) {
    return false;
  }

  cancelAutoLiftOff();

  if (!mWaypointList.isEmpty()) {
    return false;
  }

  PosPoint liftOffGoal = mCopterState->getPosition(mMissionPosTypeUsed);
  liftOffGoal.setHeight(mAutoLiftOffHeight);
  QList<PosPoint> liftOffRoute;
  liftOffRoute.append(liftOffGoal);
  startWaypointFollower(liftOffRoute);
  mAutoLiftOffActive = true;

  updateMissionState(deriveWaypointFollowerMissionState());
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
  mWaypointFollower->setDescentSpeed(mDescentSpeed);
  mWaypointFollower->setMinApproachSpeed(mMinApproachSpeed);
  mWaypointFollower->setApproachSlowdownRadius(mApproachSlowdownRadius);
  mWaypointFollower->setFaceTravelDirection(mFaceTravelDirection);
  mWaypointFollower->setYawGain(mYawGain);
  mWaypointFollower->setMaxYawRate(mMaxYawRate);
  mWaypointFollower->setVerticalHeightTolerance(mVerticalHeightTolerance);
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
        if (mAutoLiftOffActive) {
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
        if (!mAutoLiftOffActive) {
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
    case MissionState::FollowRouteLiftOff:
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
      } else if (!mWaypointFollower->isActive()) {
        if (mWaypointList.isEmpty()) {
          updateMissionState(MissionState::WaitingForRoute);
        } else {
          startWaypointFollower(mWaypointList);
          updateMissionState(MissionState::FollowRouteInit);
        }
      } else {
        WayPointFollowerSTMstates wayPointFollowerSTMstate =
          mWaypointFollower->getCurrentState().stmState;
        const MissionState waypointFollowerMissionState = deriveWaypointFollowerMissionState();
        if (currentMissionState != waypointFollowerMissionState) {
          updateMissionState(waypointFollowerMissionState);
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
      if (mAutoLiftOffActive) {
        cancelAutoLiftOff();
        stopWaypointFollower();
        updateMissionState(mWaypointList.isEmpty() ? MissionState::Idle : MissionState::WaitingForRoute);
      } else {
        stopWaypointFollower();
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
  cancelAutoLiftOff();
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

bool CopterAutopilotComponent::getRouteLiftOffActive() const
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
  if (mAutoLiftOffActive && mWaypointFollower && mWaypointFollower->isActive()) {
    cancelAutoLiftOff();
    stopWaypointFollower();
    startWaypointFollower(waypointList);
    mWaypointList = mWaypointFollower->getCurrentRoute();
    return;
  }

  switch (currentMissionState) {
    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteLiftOff:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
      cancelAutoLiftOff();
      stopWaypointFollower();
      startWaypointFollower(waypointList);
      mWaypointList = mWaypointFollower->getCurrentRoute();
      break;
    default:
      cancelAutoLiftOff();
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
    return MissionState::FollowRouteLiftOff;
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
