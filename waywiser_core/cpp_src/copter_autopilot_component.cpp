#include "copter_autopilot_component.hpp"
#include "moc_copter_autopilot_component.cpp"

#include <algorithm>

#include "WayWise/core/pospoint.h"

void CopterAutopilotComponent::reset()
{
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

  // -- MAVLINK communication towards ControlTower --
  if (mEnableMavlinkInterface) {
    mMavsdkVehicleServer.reset(
      new MavsdkVehicleServer(
        mCopterState,
        QHostAddress(QString::fromStdString(mWaywiseControlTowerAddress)),
        mWaywiseControlTowerPort));
    mMavsdkVehicleServer->setMovementController(mAutopilotMovementController);
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
