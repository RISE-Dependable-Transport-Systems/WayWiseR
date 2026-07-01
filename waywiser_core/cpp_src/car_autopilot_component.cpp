#include "car_autopilot_component.hpp"
#include "moc_car_autopilot_component.cpp"

void CarAutopilotComponent::reset()
{
  mWaypointFollower->clearRoute();
  mWaypointFollower->resetState();
  updateMissionState(MissionState::WaitingForVehicleInit);
  mAutopilotMovementController->setDesiredSpeed(0.0);
  mAutopilotMovementController->setDesiredSteering(0.0);
  mWaypointList.clear();
}

void CarAutopilotComponent::setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState)
{
  mEmergencyStopState = emergencyStopState;

  mCarState->setEndGoalAlignmentType(mEndGoalAlignmentType);
  mAutopilotMovementController.reset(new MovementController(mCarState));

  // -- MAVLINK communication towards ControlTowerNode --
  if (mEnableMavlinkInterface) {
    mMavsdkVehicleServer.reset(
      new MavsdkVehicleServer(
        mCarState,
        QHostAddress(QString::fromStdString(mWaywiseControlTowerAddress)),
        mWaywiseControlTowerPort));
    mMavsdkVehicleServer->setMovementController(mAutopilotMovementController);
    mMavsdkVehicleServer->setTransferLogs(false); // TODO: make this configurable
  }

  // --- Autopilot ---
  mFollowPoint.reset(new FollowPoint(mAutopilotMovementController));
  mWaypointFollower.reset(new PurepursuitWaypointFollower(mAutopilotMovementController));
  mWaypointFollower->setPurePursuitRadius(mPurePursuitRadius);
  mWaypointFollower->setRepeatRoute(false);
  mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);
  mWaypointFollower->setEndGoalAlignmentThreshold(mEndGoalAlignmentThreshold);
  mWaypointFollower->setAdaptiveApproachSpeedEnabled(mAdaptiveApproachSpeedEnabled);
  mWaypointFollower->setMinApproachSpeed(mMinApproachSpeed);
  if (mEnableMavlinkInterface) {
    mMavsdkVehicleServer->setWaypointFollower(mWaypointFollower);
  }
}

void CarAutopilotComponent::provideParametersToParameterServer()
{
  mCarState->provideParametersToParameterServer();
  mWaypointFollower->provideParametersToParameterServer();
  mFollowPoint->provideParametersToParameterServer();
  mMavsdkVehicleServer->provideParametersToParameterServer();
}

void CarAutopilotComponent::processMissionStateMachine()
{
  switch (mGnssFixStatus.fixType) {
    case GNSS_FIX_TYPE::NO_FIX:
    case GNSS_FIX_TYPE::TIME_ONLY_FIX:
      {
        if (currentMissionState != MissionState::WaitingForVehicleInit) {
          updateMissionState(MissionState::WaitingForVehicleInit);
        }
      } break;
    default:
      {
        switch (currentMissionState) {
          case MissionState::WaitingForVehicleInit: {
              updateMissionState(MissionState::Idle);
            } break;
          case MissionState::Idle: {
              // Check if mWaypointFollower is started via MAVLINK
              if (mWaypointFollower->isActive()) {
                if (mWaypointFollower->getCurrentRoute().size() > 0) {
                  mWaypointList = mWaypointFollower->getCurrentRoute();
                  updateMissionState(
                    CoreUtils::convertToMissionState(
                      mWaypointFollower->getCurrentState().stmState));
                } else {
                  updateMissionState(MissionState::WaitingForRoute);
                }
              }
            } break;
          case MissionState::WaitingForRoute: {
              if (!mWaypointList.isEmpty()) {
                if (mEmergencyStopState->is_active()) {
                  updateMissionState(MissionState::WaitingForEmergencyStopClear);
                } else if (!assertGnssFixAccuracy()) {
                  updateMissionState(MissionState::WaitingForGnssAccuracy);
                  emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
                } else {
                  startWaypointFollower(mWaypointList);
                  updateMissionState(MissionState::FollowRouteInit);
                }
              } else if (mWaypointFollower->getCurrentRoute().size() > 0) {
                mWaypointList = mWaypointFollower->getCurrentRoute();
                updateMissionState(
                  CoreUtils::convertToMissionState(
                    mWaypointFollower->getCurrentState().stmState));
              }
            } break;
          case MissionState::WaitingForEmergencyStopClear: {
              if (mEmergencyStopState->is_clear()) {
                updateMissionState(MissionState::Idle);
              }
            } break;
          case MissionState::WaitingForGnssAccuracy: {
              if (assertGnssFixAccuracy()) {
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
            } break;
          case MissionState::FollowRouteInit:
          case MissionState::FollowRouteGotoBegin:
          case MissionState::FollowRouteFollowing:
          case MissionState::FollowRouteApproachingEndGoal:
            {
              if (mEmergencyStopState->is_active()) {
                stopWaypointFollower();
                updateMissionState(MissionState::WaitingForEmergencyStopClear);
              } else if (!assertGnssFixAccuracy()) {
                stopWaypointFollower();
                updateMissionState(MissionState::WaitingForGnssAccuracy);
                emit gnssFixAccuracyAssertionFailed(mGnssFixStatus);
              } else {
                // Synchronize mission state with waypoint follower state
                WayPointFollowerSTMstates wayPointFollowerSTMstate =
                  mWaypointFollower->getCurrentState().stmState;
                if (CoreUtils::convertToWayPointFollowerSTMstates(currentMissionState) !=
                  wayPointFollowerSTMstate)
                {
                  updateMissionState(CoreUtils::convertToMissionState(wayPointFollowerSTMstate));

                  if (currentMissionState == MissionState::Idle && mWaypointFollower->isActive()) {
                    stopWaypointFollower();
                  }
                }

                // Check if the route is finished by overshooting the end goal
                if (currentMissionState == MissionState::FollowRouteApproachingEndGoal &&
                  !mWaypointFollower->isActive())
                {
                  updateMissionState(MissionState::FollowRouteFinished);
                }
              }
            } break;
          case MissionState::FollowRouteFinished:
            {
              stopWaypointFollower();
            } break;
          default:
            break;
        }
      } break;
  }
}

void CarAutopilotComponent::switchAutopilot(bool enable)
{
  if (enable) {
    if (currentMissionState == MissionState::Idle) {
      updateMissionState(MissionState::WaitingForRoute);
    } else if (currentMissionState == MissionState::WaitingForVehicleInit) {
      qDebug() <<
        "Waiting for vehicle init, ignoring autopilot start request.";
    }
  } else {
    if (currentMissionState != MissionState::Idle) {
      stopWaypointFollower();
      updateMissionState(MissionState::Idle);
    }
  }
}

void CarAutopilotComponent::startWaypointFollower(QList<PosPoint> & waypointList)
{
  mWaypointFollower->clearRoute();
  mWaypointFollower->resetState();
  mWaypointFollower->addRoute(waypointList);
  mWaypointFollower->startFollowingRoute(false);
  qDebug() << "Started waypoint follower with a route of " << waypointList.size() << " waypoints";
}

void CarAutopilotComponent::stopWaypointFollower()
{
  // mWaypointFollower->clearRoute();
  // mWaypointList.clear();
  // qDebug() << "Waypoint follower is stopped and route cleared.";
  mWaypointFollower->stop();
  updateMissionState(MissionState::Idle);
  mAutopilotMovementController->setDesiredSpeed(0.0);
  mAutopilotMovementController->setDesiredSteering(0.0);
  qDebug() << "Waypoint follower is stopped.";
}

bool CarAutopilotComponent::isActive()
{
  return mWaypointFollower->isActive();
}

std::optional<PosPoint> CarAutopilotComponent::getVehicleAlignmentReferencePosPoint()
{
  if (mWaypointFollower->getCurrentRoute().isEmpty()) {
    return std::nullopt;
  }

  return mWaypointFollower->getVehicleAlignmentReferencePosPoint();
}

QSharedPointer<VehicleState> CarAutopilotComponent::getReferenceVehicleState()
{
  return mWaypointFollower->getReferenceVehicleState();
}

void CarAutopilotComponent::updateWaypointFollowerRoute(QList<PosPoint> & waypointList)
{
  switch (currentMissionState) {
    case MissionState::FollowRouteInit:
    case MissionState::FollowRouteGotoBegin:
    case MissionState::FollowRouteFollowing:
    case MissionState::FollowRouteApproachingEndGoal:
      {
        stopWaypointFollower();
        startWaypointFollower(waypointList);
        mWaypointList = mWaypointFollower->getCurrentRoute();
      } break;
    default:
      {
        mWaypointList = waypointList;
      } break;
  }
}

void CarAutopilotComponent::updateMissionState(MissionState state)
{
  currentMissionState = state;
  qDebug() << "MissionState: " << CoreUtils::missionStateToString(currentMissionState).c_str();
  emit updatedMissionState(currentMissionState);
}

bool CarAutopilotComponent::assertGnssFixAccuracy()
{
  if (mGnssFixStatus.horizontalAccuracy > mPositionAccuracyThresholdForMission ||
    mGnssFixStatus.verticalAccuracy > mYawAccuracyThresholdForMission)
  {
    return false;
  }

  return true;
}
