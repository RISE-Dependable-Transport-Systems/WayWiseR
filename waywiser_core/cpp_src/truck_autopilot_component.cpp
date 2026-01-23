#include "truck_autopilot_component.hpp"
#include "moc_truck_autopilot_component.cpp"


void TruckAutopilotComponent::setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState)
{
  CarAutopilotComponent::setupAutopilot(emergencyStopState);

  mTruckState->setPurePursuitForwardGain(mPurePursuitForwardGain);
  mTruckState->setPurePursuitReverseGain(mPurePursuitReverseGain);
}
